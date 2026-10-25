"""Measure the voice recognizer on real audio: how many commands it emits from ambient noise (false positives)
and whether it hears the command words when they are actually said.

    python tools/eval_voice.py                      # record 40 s from the default mic, report
    python tools/eval_voice.py -n 60 --wav amb.wav  # also save the audio, so before/after use the same sound
    python tools/eval_voice.py --replay amb.wav     # re-run the current code on saved audio (no mic)
    python tools/eval_voice.py --tts --wav t.wav    # Windows TTS says "james stop", "james grab", ... 3x each
                                                    # through the speakers while recording; per-word hit table
    python tools/eval_voice.py --tts --no-wake --tts-words stop,grab,release,raise,reach,home,tuck,follow

The audio goes through a real VoiceListener (its own _loop, grammar, min_conf, wake word, partial-result
stop), so the numbers are what demo.py would have seen. Nothing here touches the arm. With --tts the spoken
phrases are timestamped against the recording (saved as <wav>.json next to --wav) and every command the
listener emitted is matched to the phrase being spoken at that moment.
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
import wave

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from james_gestures.voice import WORDS, VoiceListener, parse_result  # noqa: E402

COMMANDS = ["stop", "grab", "release", "raise", "reach", "home", "tuck", "follow"]
RATE, BLOCK = 16000, 4000


class _Tap:
    """Wraps the KaldiRecognizer: keeps every final result and tracks how much audio has gone in."""

    def __init__(self, rec):
        self._rec, self.finals, self.audio_t = rec, [], 0.0

    def __getattr__(self, name):
        return getattr(self._rec, name)

    def AcceptWaveform(self, data):
        self.audio_t += len(data) / 2 / RATE
        return self._rec.AcceptWaveform(data)

    def Result(self):
        raw = self._rec.Result()
        self.finals.append(json.loads(raw))
        return raw

    def FinalResult(self):
        raw = self._rec.FinalResult()
        self.finals.append(json.loads(raw))
        return raw


class _CmdLog(queue.Queue):
    """The listener's command queue, but it also remembers at what audio time each command came out."""

    def __init__(self, tap: _Tap):
        super().__init__()
        self.tap, self.log = tap, []

    def put(self, item, *a, **kw):
        self.log.append((self.tap.audio_t, item))
        super().put(item, *a, **kw)


def record(seconds: float, device, tts_phrases=None, tts_reps=3):
    import sounddevice as sd

    chunks: list[bytes] = []
    q: queue.Queue[bytes] = queue.Queue()

    def cb(data, frames, time_info, status):
        q.put(bytes(data))

    stream = sd.RawInputStream(samplerate=RATE, blocksize=BLOCK, device=device, dtype="int16", channels=1,
                               callback=cb)
    stream.start()
    t_rec0 = time.time()
    events: list[dict] = []
    tts = None
    if tts_phrases:
        tts = threading.Thread(target=speak, args=(tts_phrases, tts_reps, t_rec0, events), daemon=True)
        tts.start()
    try:
        while time.time() - t_rec0 < seconds or (tts is not None and tts.is_alive()):
            try:
                chunks.append(q.get(timeout=0.2))
            except queue.Empty:
                pass
    finally:
        stream.stop()
        stream.close()
    return b"".join(chunks), events


def speak(phrases, reps: int, t_rec0: float, events: list, gap_s: float = 1.5) -> None:
    """Windows text-to-speech through the speakers, one PowerShell process for the whole run so the timing of
    each phrase is known to ~0.1 s. Each phrase `reps` times with a pause so Vosk gets a final result between."""
    ps = subprocess.Popen(["powershell", "-NoProfile", "-NonInteractive", "-Command", "-"], stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
    ps.stdin.write("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                   "$s.Volume = 70; $s.Rate = -1; [Console]::Out.WriteLine('ready')\n")
    ps.stdin.flush()
    ps.stdout.readline()
    time.sleep(1.5)                                 # let the stream settle
    for i in range(reps):
        for p in phrases:
            t0 = time.time() - t_rec0
            ps.stdin.write("$s.Speak('" + p.replace("'", "") + "'); [Console]::Out.WriteLine('done')\n")
            ps.stdin.flush()
            ps.stdout.readline()
            t1 = time.time() - t_rec0
            events.append({"phrase": p, "t0": round(t0, 2), "t1": round(t1, 2)})
            print(f"  [tts {t0:6.1f}-{t1:6.1f}s] {p}", flush=True)
            time.sleep(gap_s)
    ps.stdin.write("exit\n")
    ps.stdin.flush()
    ps.wait(timeout=10)


def run_listener(audio: bytes, min_conf: float | None, model_path: str | None, no_wake: bool = False,
                 stop_streak: int | None = None):
    kw = {} if min_conf is None else {"min_conf": min_conf}
    if no_wake:
        kw["wake"] = None
    if stop_streak is not None:
        kw["stop_streak"] = stop_streak
    lst = VoiceListener(model_path, device=None, sample_rate=RATE, start=False, **kw)
    tap = _Tap(lst.rec)
    lst.rec = tap
    lst._cmds = _CmdLog(tap)
    lst._running = True
    for i in range(0, len(audio), BLOCK * 2):
        lst._audio.put(audio[i:i + BLOCK * 2])
    t = threading.Thread(target=lst._loop, daemon=True)
    t.start()
    while not lst._audio.empty():
        time.sleep(0.05)
    time.sleep(0.5)                                 # let the last chunk be processed
    lst._running = False
    t.join(timeout=2)
    # flush whatever is still buffered in the recognizer, the way a pause in the room would
    res = json.loads(tap.FinalResult())
    wake = getattr(lst, "wake", None)
    for c in parse_result(res, lst.min_conf, wake):
        if not (c == "stop" and lst._stop_sent):
            lst._cmds.put(c)
    return lst, tap.finals, lst._cmds.log


def expected_command(phrase: str) -> str | None:
    """'james let go' -> 'release'; the command word is whatever is in WORDS, last match wins."""
    toks = phrase.lower().split()
    for i in range(len(toks) - 1, -1, -1):
        if " ".join(toks[i:i + 2]) in WORDS:
            return WORDS[" ".join(toks[i:i + 2])]
        if toks[i] in WORDS:
            return WORDS[toks[i]]
    return None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--seconds", type=float, default=40.0)
    ap.add_argument("--device", default=None, help="sounddevice input index/name (default mic if omitted)")
    ap.add_argument("--min-conf", type=float, default=None, help="override VoiceListener min_conf")
    ap.add_argument("--vosk-model", default=None)
    ap.add_argument("--wav", default=None, help="save the recorded audio here (+ <wav>.json with TTS timing)")
    ap.add_argument("--replay", default=None, help="use this 16 kHz mono int16 WAV instead of the mic")
    ap.add_argument("--tts", action="store_true", help="speak each phrase via Windows TTS while recording")
    ap.add_argument("--tts-words", default=",".join("james " + c for c in COMMANDS),
                    help='comma-separated phrases to speak, e.g. "stop,grab" or "james stop,james grab"')
    ap.add_argument("--tts-reps", type=int, default=3)
    ap.add_argument("--no-wake", action="store_true", help="run the listener with wake=None (bare command words)")
    ap.add_argument("--stop-streak", type=int, default=None,
                    help="partials in a row that must show stop before the early stop fires (1 = old behaviour)")
    a = ap.parse_args()
    device = a.device
    if device is not None and device.isdigit():
        device = int(device)

    events: list[dict] = []
    if a.replay:
        with wave.open(a.replay, "rb") as w:
            assert w.getframerate() == RATE and w.getnchannels() == 1 and w.getsampwidth() == 2, "need 16k mono int16"
            audio = w.readframes(w.getnframes())
        print(f"replaying {a.replay}: {len(audio) / 2 / RATE:.1f} s")
        if os.path.exists(a.replay + ".json"):
            with open(a.replay + ".json") as f:
                events = json.load(f)
    else:
        phrases = [w.strip() for w in a.tts_words.split(",") if w.strip()] if a.tts else None
        print(f"recording {a.seconds:.0f} s from device {device if device is not None else 'default'}"
              + (f", TTS {a.tts_reps}x each of {phrases}" if phrases else "") + " ...", flush=True)
        audio, events = record(a.seconds, device, phrases, a.tts_reps)
        if a.wav:
            with wave.open(a.wav, "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(RATE)
                w.writeframes(audio)
            print("saved", a.wav)
            if events:
                with open(a.wav + ".json", "w") as f:
                    json.dump(events, f)

    import numpy as np
    pcm = np.frombuffer(audio, dtype=np.int16).astype(np.float32)
    dur = len(pcm) / RATE
    rms = float(np.sqrt(np.mean(pcm ** 2))) if len(pcm) else 0.0
    print(f"audio: {dur:.1f} s, rms {rms:.0f} / 32768, peak {int(np.abs(pcm).max()) if len(pcm) else 0}")

    lst, finals, cmdlog = run_listener(audio, a.min_conf, a.vosk_model, a.no_wake, a.stop_streak)
    cmds = [c for _, c in cmdlog]
    texts = [f.get("text", "") for f in finals if f.get("text")]
    words = [w for f in finals for w in f.get("result", [])]
    real_words = [w for w in words if w["word"] != "[unk]"]
    print()
    print(f"min_conf={lst.min_conf}  wake={getattr(lst, 'wake', None)}  finals={len(finals)}  non-empty={len(texts)}")
    print(f"recognized words (excluding [unk]): {len(real_words)}")
    if real_words:
        confs = sorted(w.get("conf", 1.0) for w in real_words)
        print(f"  word conf: min {confs[0]:.2f}  median {confs[len(confs) // 2]:.2f}  max {confs[-1]:.2f}")
    print(f"commands emitted: {len(cmds)}  ->  {len(cmds) / dur * 60:.1f} per minute")
    counts = {c: cmds.count(c) for c in COMMANDS if cmds.count(c)}
    print(f"  by command: {counts}")
    print("raw vosk text (one line per final result):")
    for f in finals:
        if f.get("text"):
            ws = " ".join(f"{w['word']}:{w.get('conf', 1.0):.2f}" for w in f.get("result", []))
            print(f"  {f['text']!r:40}  {ws}")
    if not texts:
        print("  (nothing)")

    hits = {}
    if events:
        print()
        print("TTS true positives (command emitted while the phrase was being spoken, +1.5 s):")
        used = set()
        for e in events:
            want = expected_command(e["phrase"])
            got = [(i, c) for i, (t, c) in enumerate(cmdlog) if e["t0"] - 0.3 <= t <= e["t1"] + 1.5]
            ok = any(c == want for _, c in got)
            used.update(i for i, _ in got)
            hits.setdefault(e["phrase"], [0, 0])
            hits[e["phrase"]][1] += 1
            hits[e["phrase"]][0] += int(ok)
            print(f"  {e['t0']:6.1f}s {e['phrase']:16} want {want:8} got {[c for _, c in got]}  {'HIT' if ok else 'miss'}")
        stray = [c for i, (_, c) in enumerate(cmdlog) if i not in used]
        print("  per phrase:", {p: f"{h}/{n}" for p, (h, n) in hits.items()})
        print(f"  commands outside any TTS window (false positives during the run): {len(stray)} {stray}")
    print()
    print("JSON:", json.dumps({"seconds": round(dur, 1), "rms": round(rms), "min_conf": lst.min_conf,
                               "wake": getattr(lst, "wake", None), "words": len(real_words),
                               "commands": len(cmds), "cmds_per_min": round(len(cmds) / dur * 60, 2),
                               "by_command": counts, "tts_hits": hits, "texts": texts}))


if __name__ == "__main__":
    main()
