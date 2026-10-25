"""Issue 5: voice must not die silently. No mic, no vosk: a fake recognizer, a fake mic and a fake clock."""
import json
import threading
import time

from james_gestures.voice import VoiceListener


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakeRec:
    """Says 'james grab' on every chunk, or raises when the chunk is b'bad'."""

    def AcceptWaveform(self, data):
        if data == b"bad":
            raise RuntimeError("corrupt audio")
        return True

    def Result(self):
        return json.dumps({"text": "james grab", "result": [{"word": "james", "conf": 1.0},
                                                            {"word": "grab", "conf": 1.0}]})

    def PartialResult(self):
        return json.dumps({"partial": ""})


class FakeStream:
    opened = 0

    def __init__(self, **kw):
        FakeStream.opened += 1

    def start(self):
        pass

    def stop(self):
        pass

    def close(self):
        pass


class FakeSD:
    RawInputStream = FakeStream


def listener():
    clock = Clock()
    v = VoiceListener.__new__(VoiceListener)     # skip __init__: no vosk model, no real mic
    v._init_state(clock=clock)
    v.rec, v.wake, v._sd = FakeRec(), ("james",), FakeSD()
    return v, clock


def test_bad_chunk_does_not_kill_voice():
    v, _ = listener()
    v._running = True
    v._audio.put(b"bad")                        # the recognizer raises on this one
    v._audio.put(b"good")
    v._thread = threading.Thread(target=v._loop, daemon=True)
    v._thread.start()
    deadline, cmds = time.monotonic() + 2, []
    while time.monotonic() < deadline and not cmds:
        cmds = v.poll()
        time.sleep(0.02)
    v._running = False
    v._thread.join(1)
    assert cmds == ["grab"]                     # the good chunk after the bad one still worked
    assert v.problem == ""                      # and voice reports healthy again


def test_silent_mic_is_reported_and_reopened():
    v, clock = listener()
    v.start()
    opened = FakeStream.opened
    clock.t += 3.0                              # no audio callbacks for 3 s
    assert v.poll() == []
    assert v.problem.startswith("mic silent")
    assert FakeStream.opened == opened + 1      # it tried to reopen the mic
    v._on_audio(b"x", 4000, None, None)         # audio flows again
    v.check()
    assert v.problem == ""
    v.close()


def test_dead_thread_is_reported():
    v, clock = listener()
    v._running = True
    v._thread = threading.Thread(target=lambda: None)
    v._thread.start()
    v._thread.join()
    assert "stopped" in v.check()


def test_fatal_error_text_is_shown():
    v, _ = listener()
    v._running = True
    v._thread = threading.Thread(target=lambda: None)
    v._thread.start()
    v._thread.join()
    v._fatal = "voice thread died: boom"
    assert v.check() == "voice thread died: boom"
