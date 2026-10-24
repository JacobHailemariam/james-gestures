"""Voice commands for the arm, offline (Vosk), so it works on bad hackathon wifi.

    listener = VoiceListener()      # starts listening on the default mic in a background thread
    for cmd in listener.poll():     # call once per camera frame
        arm.voice(cmd)

Words it understands (anything else is ignored):

    stop, freeze, halt         -> "stop"     (acted on mid-word, before you finish talking)
    grab, close, grip          -> "grab"     claw closes
    release, open, drop, let go -> "release" claw opens
    raise, lift, up            -> "raise"    preset
    reach, forward             -> "reach"    preset
    home, reset                -> "home"     preset
    tuck, rest, down           -> "tuck"     preset
    follow, resume, move       -> "follow"   hand steering back on after a preset

The recognizer is limited to these words, which makes it fast and accurate. The catch: in a loud room it can
force someone else's speech into one of them, so every command except stop needs a confident match
(`min_conf`). A false "stop" only stops the arm, which is safe.

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
}
STOP_WORDS = {w for w, c in WORDS.items() if c == "stop"}


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
        two = " ".join(words[i:i + 2])
        if two in WORDS:
            out.append(WORDS[two])
            i += 2
            continue
        if words[i] in WORDS:
            out.append(WORDS[words[i]])
        i += 1
    return out


def parse_result(result: dict, min_conf: float = 0.6) -> list[str]:
    """A final Vosk result (with word confidences) -> commands. Low-confidence words are dropped,
    except stop words: a wrong stop is safe, a missed stop is not."""
    words = result.get("result")
    if not words:                                   # no per-word info: fall back to the text
        return parse(result.get("text", ""))
    kept = [w["word"] for w in words if w.get("conf", 1.0) >= min_conf or w["word"] in STOP_WORDS]
    return parse(" ".join(kept))


class VoiceListener:
    """Listens in a background thread. poll() returns the commands heard since the last call."""

    def __init__(self, model_path: str | None = None, device=None, min_conf: float = 0.6,
                 sample_rate: int = 16000, start: bool = True):
        import sounddevice as sd
        import vosk

        vosk.SetLogLevel(-1)
        self._sd = sd
        self.model = vosk.Model(model_path) if model_path else vosk.Model(lang="en-us")
        grammar = json.dumps(sorted(WORDS) + ["[unk]"])
        self.rec = vosk.KaldiRecognizer(self.model, sample_rate, grammar)
        self.rec.SetWords(True)
        self.min_conf, self.rate, self.device = min_conf, sample_rate, device
        self.heard = ""                      # last thing recognised, for the HUD
        self._audio: queue.Queue[bytes] = queue.Queue()
        self._cmds: queue.Queue[str] = queue.Queue()
        self._stop_sent = False              # one early stop per utterance
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
                for c in parse_result(res, self.min_conf):
                    if c == "stop" and self._stop_sent:
                        continue                 # already sent from the partial result
                    self._cmds.put(c)
                self._stop_sent = False
            else:
                part = json.loads(self.rec.PartialResult()).get("partial", "")
                if not self._stop_sent and any(w in STOP_WORDS for w in part.split()):
                    self._stop_sent = True       # react to "stop" without waiting for the pause after it
                    self.heard = part
                    self._cmds.put("stop")

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
