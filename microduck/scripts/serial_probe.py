#!/usr/bin/env python3
"""Probe a serial port across baud rates and report which one yields readable text.

Use when a console prints garbage: the baud rate is wrong, or the USB-UART
adapter cannot produce the board's rate (e.g. CP2102 maxes out around 921600,
so a Taishan Pi debug console at 1500000 will never read cleanly on one).

Usage:
    serial_probe.py /dev/ttyUSB0
    serial_probe.py /dev/ttyUSB0 --seconds 2.0
"""

import argparse
import time

import serial

DEFAULT_BAUDS = [
    1500000,
    1000000,
    921600,
    500000,
    460800,
    250000,
    230400,
    115200,
    76800,
    57600,
    38400,
    9600,
]


def printable_ratio(data):
    if not data:
        return 0.0
    ok = sum(1 for b in data if 32 <= b < 127 or b in (9, 10, 13))
    return ok / len(data)


def sample_text(data, width=100):
    return "".join(chr(b) if 32 <= b < 127 else "." for b in data[:width])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--bauds", default=",".join(str(b) for b in DEFAULT_BAUDS))
    ap.add_argument("--seconds", type=float, default=1.5)
    args = ap.parse_args()
    bauds = [int(b) for b in args.bauds.split(",")]

    for baud in bauds:
        try:
            ser = serial.Serial(args.port, baud, timeout=0.2)
        except Exception as e:
            print(f"{baud:>7}: open failed: {e}")
            continue
        ser.reset_input_buffer()
        buf = bytearray()
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            buf += ser.read(4096)
        ser.close()
        ratio = printable_ratio(buf)
        print(f"{baud:>7}: bytes={len(buf):>6} printable={ratio * 100:5.1f}%  {sample_text(buf)}")


if __name__ == "__main__":
    main()
