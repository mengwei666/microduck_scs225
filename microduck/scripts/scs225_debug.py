#!/usr/bin/env python3
"""Feetech SCS/SCSCL bus servo bench tool (SCS225 and friends).

No third-party dependency: the serial port is driven through termios, so it
runs on a bare Buildroot board exactly as it does on a laptop.

Wiring: Waveshare Bus Servo Adapter (A), jumper on B, USB-C to the host,
DC jack powered to match the servos (SCS225: 5-8.4 V). Servos are daisy
chained on the 3-pin bus and must have unique IDs.

Examples:
    scs225_debug.py scan
    scs225_debug.py read --ids 1-13
    scs225_debug.py demo --ids 1-13
    scs225_debug.py move --ids 1-13 --position 511
"""

import argparse
import glob
import os
import select
import termios
import time
import tty

REG_VERSION = 0x03
REG_ID = 0x05
REG_BAUD = 0x06
REG_MIN_ANGLE = 0x09
REG_MAX_ANGLE = 0x0B
REG_TORQUE_ENABLE = 0x28
REG_GOAL_POSITION = 0x2A
REG_GOAL_TIME = 0x2C
REG_GOAL_SPEED = 0x2E
REG_PRESENT_POSITION = 0x38
REG_PRESENT_SPEED = 0x3A
REG_PRESENT_LOAD = 0x3C
REG_PRESENT_VOLTAGE = 0x3E
REG_PRESENT_TEMPERATURE = 0x3F
REG_MOVING = 0x42

POS_MIN, POS_MAX, POS_CENTER = 0, 1024, 511


def baud_const(baud):
    name = "B%d" % baud
    if not hasattr(termios, name):
        raise SystemExit("baud %d is not usable through termios" % baud)
    return getattr(termios, name)


def autodetect_port():
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        ports = sorted(glob.glob(pattern))
        if ports:
            return ports[0]
    raise SystemExit("no serial port found; pass --port /dev/ttyACM0")


class ScsBus:
    def __init__(self, port, baud, timeout=0.05, retries=2):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        speed = baud_const(baud)
        attrs[4] = speed
        attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.timeout = timeout
        self.retries = retries

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def _read_n(self, n, timeout):
        out = bytearray()
        deadline = time.time() + timeout
        while len(out) < n:
            remain = deadline - time.time()
            if remain <= 0:
                break
            try:
                ready, _, _ = select.select([self.fd], [], [], remain)
            except InterruptedError:
                continue
            if not ready:
                break
            try:
                chunk = os.read(self.fd, n - len(out))
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                break
            out += chunk
        return bytes(out)

    def drain(self, timeout):
        out = bytearray()
        deadline = time.time() + timeout
        while True:
            remain = deadline - time.time()
            if remain <= 0:
                break
            try:
                ready, _, _ = select.select([self.fd], [], [], remain)
            except InterruptedError:
                continue
            if not ready:
                break
            try:
                chunk = os.read(self.fd, 4096)
            except (BlockingIOError, OSError):
                break
            if not chunk:
                break
            out += chunk
        return bytes(out)

    def send(self, data):
        os.write(self.fd, data)

    @staticmethod
    def _cks(payload):
        return (~(sum(payload) & 0xFF)) & 0xFF

    def _txn(self, sid, instr, params, resp_len):
        body = [sid, len(params) + 2, instr] + params
        pkt = bytes([0xFF, 0xFF]) + bytes(body) + bytes([self._cks(body)])
        for _ in range(self.retries + 1):
            termios.tcflush(self.fd, termios.TCIFLUSH)
            os.write(self.fd, pkt)
            r = self._read_n(resp_len, self.timeout)
            if len(r) == resp_len and r[0] == 0xFF and r[1] == 0xFF and r[2] == sid:
                return r
        return None

    def ping(self, sid):
        r = self._txn(sid, 0x01, [], 6)
        return r is not None and r[4] == 0

    def read(self, sid, addr, length):
        r = self._txn(sid, 0x02, [addr, length], 6 + length)
        if r is None or r[4] != 0:
            return None
        return list(r[5 : 5 + length])

    def write(self, sid, addr, data):
        self._txn(sid, 0x03, [addr] + list(data), 6)
        return True

    def sync_write(self, addr, per_id_data, data_len):
        params = [addr, data_len]
        for sid in sorted(per_id_data):
            params.append(sid)
            params.extend(per_id_data[sid])
        body = [0xFE, len(params) + 2, 0x83] + params
        pkt = bytes([0xFF, 0xFF]) + bytes(body) + bytes([self._cks(body)])
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, pkt)

    def read_u16(self, sid, addr):
        d = self.read(sid, addr, 2)
        return None if d is None else (d[0] << 8) | d[1]

    def read_u8(self, sid, addr):
        d = self.read(sid, addr, 1)
        return None if d is None else d[0]

    def torque(self, sid, on):
        self.write(sid, REG_TORQUE_ENABLE, [1 if on else 0])

    def goal(self, sid, position, speed=0):
        position = max(POS_MIN, min(POS_MAX, int(position)))
        self.write(sid, REG_GOAL_POSITION, encode_u16(position))
        if speed:
            self.write(sid, REG_GOAL_SPEED, encode_u16(speed))


def encode_u16(value):
    value = max(0, min(0xFFFF, int(value)))
    return [(value >> 8) & 0xFF, value & 0xFF]


def parse_ids(spec):
    ids = []
    for part in spec.split(","):
        if "-" in part:
            lo, hi = part.split("-", 1)
            ids.extend(range(int(lo), int(hi) + 1))
        else:
            ids.append(int(part))
    return [i for i in ids if 0 <= i <= 253]


def cmd_scan(bus, ids, baud):
    print(f"scanning ids {ids[0]}..{ids[-1]} at {baud} baud")
    found = []
    for sid in ids:
        if bus.ping(sid):
            ver = bus.read_u16(sid, REG_VERSION)
            ver = f"0x{ver:04x}" if ver is not None else "?"
            print(f"  id {sid:3d}  OK  version={ver}")
            found.append(sid)
    if not found:
        print("  nothing answered. check power, jumper=B, and baud rate")
    else:
        print(f"found {len(found)}: {found}")
    return found


def cmd_read(bus, ids, _baud):
    print(f"{'id':>4} {'pos':>6} {'volt':>6} {'temp':>5} {'load':>6}")
    for sid in ids:
        pos = bus.read_u16(sid, REG_PRESENT_POSITION)
        if pos is None:
            print(f"{sid:>4}   --")
            continue
        volt = bus.read_u8(sid, REG_PRESENT_VOLTAGE)
        temp = bus.read_u8(sid, REG_PRESENT_TEMPERATURE)
        load = bus.read_u16(sid, REG_PRESENT_LOAD)
        v = f"{volt / 10.0:.1f}V" if volt is not None else "?"
        t = f"{temp}C" if temp is not None else "?"
        print(f"{sid:>4} {pos:>6} {v:>6} {t:>5} {str(load):>6}")


def cmd_move(bus, ids, position, speed):
    bus.sync_write(REG_GOAL_POSITION, {sid: encode_u16(position) for sid in ids}, 2)
    if speed:
        bus.sync_write(REG_GOAL_SPEED, {sid: encode_u16(speed) for sid in ids}, 2)
    print(f"moved {len(ids)} servos to {position}")


def cmd_demo(bus, ids, _baud, amplitude, period, cycles):
    for sid in ids:
        bus.torque(sid, True)
    time.sleep(0.2)
    print(f"demo: {cycles} cycles, center {POS_CENTER} +/- {amplitude}")
    for _ in range(cycles):
        for pos in (POS_CENTER + amplitude, POS_CENTER, POS_CENTER - amplitude, POS_CENTER):
            bus.sync_write(REG_GOAL_POSITION, {sid: encode_u16(pos) for sid in ids}, 2)
            time.sleep(period / 4.0)
    for sid in ids:
        bus.torque(sid, False)
    print("torque released")


def main():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", default=argparse.SUPPRESS, help="serial port (default: first /dev/ttyUSB* or /dev/ttyACM*)")
    common.add_argument("--baud", type=int, default=argparse.SUPPRESS, help="bus baud rate (default 1000000)")
    common.add_argument("--ids", default=argparse.SUPPRESS, help="servo ids, e.g. 1-13 or 1,2,5 (default 1-13)")

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[common],
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", parents=[common])

    sub.add_parser("read", parents=[common])

    p_move = sub.add_parser("move", parents=[common])
    p_move.add_argument("--position", type=int, required=True, help="raw target 0..1024")
    p_move.add_argument("--speed", type=int, default=0, help="raw goal speed, 0 = fastest")

    p_demo = sub.add_parser("demo", parents=[common])
    p_demo.add_argument("--amplitude", type=int, default=120)
    p_demo.add_argument("--period", type=float, default=1.5)
    p_demo.add_argument("--cycles", type=int, default=3)

    args = ap.parse_args()
    ids = parse_ids(getattr(args, "ids", "1-13"))
    if not ids:
        raise SystemExit("no valid ids")
    baud = getattr(args, "baud", 1_000_000)
    port = getattr(args, "port", None) or autodetect_port()
    print(f"port={port} baud={baud}")

    bus = ScsBus(port, baud)
    try:
        if args.cmd == "scan":
            cmd_scan(bus, ids, baud)
        elif args.cmd == "read":
            cmd_read(bus, ids, baud)
        elif args.cmd == "move":
            cmd_move(bus, ids, args.position, args.speed)
        elif args.cmd == "demo":
            cmd_demo(bus, ids, baud, args.amplitude, args.period, args.cycles)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
