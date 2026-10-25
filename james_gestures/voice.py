"""Voice commands for the arm, offline (Vosk), so it works on bad hackathon wifi.

    listener = VoiceListener()      # starts listening on the default mic in a background thread
    for cmd in listener.poll():     # call once per camera frame
        arm.voice(cmd)

Say the wake word first, then the command: "james stop", "james grab", "james raise". Words it understands
(anything else is ignored):

    stop, freeze, halt         -> "stop"     (acted on mid-word, before you finish talking)
    grab, close, grip          -> "grab"     claw closes
    release, open, drop, let go -> "release" claw opens
    raise, lift, up            -> "raise"    preset
    reach, forward             -> "reach"    preset
    home, reset                -> "home"     preset
    tuck, rest, down           -> "tuck"     preset
    follow, resume, move       -> "follow"   hand steering back on after a preset

The recognizer is limited to these words, which makes it fast and accurate. The catch: in a loud room it
forces crowd noise into them. Measured in the Cal Hacks hall with nobody talking to the laptop: 6.6 false
commands per minute (release, raise, follow, stop) with just the word list and min_conf=0.6, and noise words
came back with confidence 0.8-1.0, so confidence alone cannot fix it. Hence the wake word: a command only
counts when it comes within WAKE_WINDOW words after a confident "james" in the same utterance. The same
100 s of noise then gave 0 commands. "james" was picked because noise produced it once in 100 s, versus
"arm" 6 times and "robot" 3 times. Stop is still acted on from the partial result ("james stop" fires before
the pause after it), but the stop word must survive two consecutive partials: noise stops flickered in
for a single 0.25 s partial and vanished. Pass wake=None to go back to bare words.

Needs: pip install vosk sounddevice. The first run downloads the small English model (about 40 MB) once,
or pass model_path to a folder you unzipped from
https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
"""
from __future__ import annotations

import json
import queue
import threading

WORDS = {
    "stop": "stop", "freeze": "stop", "halt": "stop",
    "grab": "grab", "close": "grab", "grip": "grab",
    "release": "release", "open": "release", "drop": "release", "let go": "release",
    "raise": "raise", "lift": "raise", "up": "raise",
    "reach": "reach", "forward": "reach",
    "home": "home", "reset": "home",
    "tuck": "tuck", "rest": "tuck", "down": "tuck",
    "follow": "follow", "resume": "follow", "move": "follow",
    "left shoulder": "left shoulder", "right shoulder": "right shoulder",     # named poses (arm.py PRESETS)
    "left hip": "left hip", "right hip": "right hip",
}
POSES = ("home", "raise", "reach", "tuck", "left shoulder", "right shoulder", "left hip", "right hip")
SAVE_WORD = "save"           # "james save left shoulder": store the current angles under that name
STOP_WORDS = {w for w, c in WORDS.items() if c == "stop"}
WAKE_WORDS = ("james",)      # must be in the Vosk model's vocabulary (it warns and drops the word otherwise)
WAKE_WINDOW = 3              # the command has to be within this many words after the wake word ("james let go")


def after_wake(tokens: list, wake, window: int = WAKE_WINDOW, confident=lambda t: True) -> list | None:
    """The up-to-`window` tokens that follow the first (confident) wake word, or None if there is no wake word.
    `tokens` are strings or Vosk word dicts; with wake=None everything is returned unchanged."""
    if not wake:
        return list(tokens)
    for i, t in enumerate(tokens):
        word = t["word"] if isinstance(t, dict) else t
        if word in wake and confident(t):
            return list(tokens[i + 1:i + 1 + window])
    return None


def parse(text: str) -> list[str]:
    """Words heard -> commands, in order. 'stop' anywhere wins and is the only command returned."""
    t = " ".join((text or "").lower().replace("[unk]", " ").split())
    if not t:
        return []
    words = t.split()
    if any(w in STOP_WORDS for w in words):
        return ["stop"]
    out, i = [], 0
    while i < len(words):
        if words[i] == SAVE_WORD:                       # "save <pose name>" -> "save:<pose name>"
            for n in (2, 1):
                name = " ".join(words[i + 1:i + 1 + n])
                if name in POSES:
                    out.append(f"save:{name}")
                    i += 1 + n
                    break
            else:
                i += 1
            continue
        two = " ".join(words[i:i + 2])
        if two in WORDS:
            out.append(WORDS[two])
            i += 2
            continue
        if words[i] in WORDS:
            out.append(WORDS[words[i]])
        i += 1
    return out


def parse_result(result: dict, min_conf: float = 0.6, wake=WAKE_WORDS, window: int = WAKE_WINDOW) -> list[str]:
    """A final Vosk result (with word confidences) -> commands. Only the words right after a confident wake
    word count (none without it). Low-confidence words are dropped, except stop words: once the wake word has
    been heard, a wrong stop is safe and a missed stop is not."""
    words = result.get("result")
    if not words:                                   # no per-word info: fall back to the text
        toks = after_wake((result.get("text") or "").lower().split(), wake, window)
        return parse(" ".join(toks)) if toks else []
    toks = after_wake(words, wake, window, confident=lambda w: w.get("conf", 1.0) >= min_conf)
    if not toks:
        return []
    kept = [w["word"] for w in toks if w.get("conf", 1.0) >= min_conf or w["word"] in STOP_WORDS]
    return parse(" ".join(kept))


class VoiceListener:
    """Listens in a background thread. poll() returns the commands heard since the last call."""

    def __init__(self, model_path: str | None = None, device=None, min_conf: float = 0.6,
                 sample_rate: int = 16000, start: bool = True, wake=WAKE_WORDS, stop_streak: int = 2):
        import sounddevice as sd
        import vosk

        vosk.SetLogLevel(-1)
        self._sd = sd
        self.model = vosk.Model(model_path) if model_path else vosk.Model(lang="en-us")
        self.wake = tuple(wake) if wake else None
        grammar = json.dumps(sorted(WORDS) + [SAVE_WORD] + list(self.wake or ()) + ["[unk]"])
        self.rec = vosk.KaldiRecognizer(self.model, sample_rate, grammar)
        self.rec.SetWords(True)
        self.min_conf, self.rate, self.device = min_conf, sample_rate, device
        self.stop_streak = stop_streak       # consecutive partials that must show "stop" before acting early
        self.heard = ""                      # last thing recognised, for the HUD
        self._audio: queue.Queue[bytes] = queue.Queue()
        self._cmds: queue.Queue[str] = queue.Queue()
        self._stop_sent = False              # one early stop per utterance
        self._stop_seen = 0                  # how many partials in a row showed a stop word
        self._running = False
        self._thread = None
        if start:
            self.start()

    def start(self) -> None:
        self._running = True
        self._stream = self._sd.RawInputStream(samplerate=self.rate, blocksize=4000, device=self.device,
                                               dtype="int16", channels=1, callback=self._on_audio)
        self._stream.start()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _on_audio(self, data, frames, time_info, status) -> None:
        self._audio.put(bytes(data))

    def _loop(self) -> None:
        while self._running:
            try:
                data = self._audio.get(timeout=0.2)
            except queue.Empty:
                continue
            if self.rec.AcceptWaveform(data):
                res = json.loads(self.rec.Result())
                self.heard = res.get("text", "")
                for c in parse_result(res, self.min_conf, self.wake):
                    if c == "stop" and self._stop_sent:
                        continue                 # already sent from the partial result
                    self._cmds.put(c)
                self._stop_sent, self._stop_seen = False, 0
            else:
                part = json.loads(self.rec.PartialResult()).get("partial", "")
                self._stop_seen = self._stop_seen + 1 if self._partial_stop(part) else 0
                if not self._stop_sent and self._stop_seen >= self.stop_streak:
                    self._stop_sent = True       # react to "stop" without waiting for the pause after it
                    self.heard = part
                    self._cmds.put("stop")

    def _partial_stop(self, partial: str) -> bool:
        """Does this partial text have a stop word (after the wake word, if one is required)?"""
        toks = after_wake(partial.split(), self.wake)
        return bool(toks) and any(w in STOP_WORDS for w in toks)

    def poll(self) -> list[str]:
        out = []
        while True:
            try:
                out.append(self._cmds.get_nowait())
            except queue.Empty:
                return out

    def close(self) -> None:
        self._running = False
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:
            pass
