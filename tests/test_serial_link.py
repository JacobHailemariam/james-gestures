"""SerialLink must survive the board dropping off USB: no exception, reconnect, PONG before trusting it."""
import time

import pytest

serial = pytest.importorskip("serial")

from james_gestures.arm import SerialLink


class FakeSerial:
    """Stands in for serial.Serial. `alive` False makes write/open fail like a vanished COM port."""
    alive = True
    pong = False
    opened = 0
    written: list = []

    def __init__(self, port, baud, timeout=0):
        if not FakeSerial.alive:
            raise serial.SerialException("could not open port")
        FakeSerial.opened += 1

    def write(self, b):
        if not FakeSerial.alive:
            raise serial.SerialException("WriteFile failed")
        FakeSerial.written.append(b)

    def read(self, n):
        return b"PONG\r\n" if FakeSerial.pong else b""

    def close(self):
        pass


@pytest.fixture
def link(monkeypatch):
    FakeSerial.alive, FakeSerial.pong, FakeSerial.opened, FakeSerial.written = True, False, 0, []
    monkeypatch.setattr(serial, "Serial", FakeSerial)
    monkeypatch.setattr(time, "sleep", lambda s: None)
    return SerialLink("COMX")


def test_send_does_not_raise_when_board_vanishes(link):
    FakeSerial.alive = False
    link.send("ARM 1")                       # would have raised SerialException before
    assert link.lost and link.dropped


def test_reconnects_only_after_pong(link, monkeypatch):
    FakeSerial.alive = False
    link.send("ARM 1")
    FakeSerial.alive = True                  # port is back, sketch not answering yet
    link._next_try = 0
    assert link.poll() is True               # opened the port, sent PING, no PONG: still lost
    assert FakeSerial.written[-1] == b"PING\n"
    link._next_try = 0
    FakeSerial.pong = True
    assert link.poll() is False              # PONG seen: usable again
    link.send("PING")
    assert FakeSerial.written[-1] == b"PING\n" and not link.lost


def test_send_is_dropped_while_lost(link):
    FakeSerial.alive = False
    link.send("ARM 1")
    before = len(FakeSerial.written)
    link.send("J base 100")
    assert len(FakeSerial.written) == before  # nothing queued, nothing raised
