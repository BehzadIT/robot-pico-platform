"""Minimal host-side utime shim for PID regression tests."""

_now_us = 0


def reset():
    global _now_us
    _now_us = 0


def set_us(value):
    global _now_us
    _now_us = int(value)


def advance_us(delta):
    global _now_us
    _now_us += int(delta)


def ticks_us():
    return _now_us


def ticks_ms():
    return _now_us // 1000


def ticks_cpu():
    return _now_us


def time():
    return _now_us / 1_000_000


def time_ns():
    return _now_us * 1000


def ticks_diff(new, old):
    return int(new) - int(old)
