#!/usr/bin/env python3
"""Legs-only SCS225 bring-up: centre, limits, stand, walk, IMU.

This is the step between "the leg is built" and "an RL policy walks it".
It drives the ten leg servos directly over the Feetech SCS bus, with no
runtime, policy or third-party Python package required:

    scan -> read -> centre -> limits -> stand -> walk -> imu

The default joint table is the Open Duck Mini layout with the neck/head
servos removed: left 20-24, right 10-14.  IDs are kept identical to the
existing `robotd --bus scs` mapping so a policy can use the same slots.
Edit `legs_config.json` (not this file) after calibrating the robot:

    center    raw count at the joint's mechanical zero (default 511)
    home_deg  stand pose, in degrees from that zero
    min/max   software and SCS EEPROM limits, in degrees from that zero
    sign      -1 if the servo turns opposite to the model

Examples:
    legs_bringup.py scan
    legs_bringup.py read
    legs_bringup.py center --hold
    legs_bringup.py limits --write
    legs_bringup.py stand --hold
    legs_bringup.py walk --direction forward --cycles 5 --imu
    legs_bringup.py walk --march --cycles 3
    legs_bringup.py imu --seconds 5
    legs_bringup.py selftest
"""

import argparse
import fcntl
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from scs225_debug import (  # noqa: E402
    POS_CENTER,
    REG_MAX_ANGLE,
    REG_MIN_ANGLE,
    REG_MOVING,
    REG_PRESENT_LOAD,
    REG_PRESENT_POSITION,
    REG_PRESENT_TEMPERATURE,
    REG_PRESENT_VOLTAGE,
    REG_VERSION,
    ScsBus,
    autodetect_port,
    encode_u16,
)

# The SCS225 covers 300 degrees over 0..=1024 counts.
DEG_PER_COUNT = 300.0 / 1024.0
COUNT_PER_DEG = 1.0 / DEG_PER_COUNT

DEFAULT_CONFIG = os.path.join(HERE, "legs_config.json")


# ---------------------------------------------------------------------------
# configuration and joint model
# ---------------------------------------------------------------------------


class Joint:
    def __init__(self, spec):
        self.name = spec["name"]
        self.id = int(spec["id"])
        self.side = int(spec.get("side", 1))
        self.sign = int(spec.get("sign", 1))
        self.center = int(spec.get("center", POS_CENTER))
        self.home_deg = float(spec.get("home_deg", 0.0))
        self.min_deg = float(spec.get("min_deg", -60.0))
        self.max_deg = float(spec.get("max_deg", 60.0))
        if abs(self.sign) != 1:
            raise ValueError("%s: sign must be +1 or -1" % self.name)

        lo = self.center + self.sign * self.min_deg * COUNT_PER_DEG
        hi = self.center + self.sign * self.max_deg * COUNT_PER_DEG
        self.min_raw = int(round(min(lo, hi)))
        self.max_raw = int(round(max(lo, hi)))
        if self.min_raw < 0 or self.max_raw > 1024:
            raise ValueError(
                "%s: limits %.0f..%.0f deg map outside 0..1024"
                % (self.name, self.min_deg, self.max_deg)
            )
        self.home_raw = self.raw(self.home_deg)

    def raw(self, deg):
        raw = int(round(self.center + self.sign * deg * COUNT_PER_DEG))
        return max(self.min_raw, min(self.max_raw, max(0, min(1024, raw))))

    def deg(self, raw):
        return self.sign * (raw - self.center) * DEG_PER_COUNT


class Config:
    def __init__(self, path):
        with open(path, "r") as f:
            data = json.load(f)
        self.path = path
        self.baud = int(data.get("baud", 1000000))
        self.port = data.get("port")
        self.imu_cfg = data.get("imu", {})
        self.gait = data.get("gait", {})
        self.joints = [Joint(j) for j in data["joints"]]
        self.by_name = {j.name: j for j in self.joints}
        if len(self.by_name) != len(self.joints):
            raise ValueError("duplicate joint name in %s" % path)
        ids = [j.id for j in self.joints]
        if len(set(ids)) != len(ids):
            raise ValueError("duplicate servo id in %s" % path)
        for leg in ("left", "right"):
            for part in ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"):
                name = "%s_%s" % (leg, part)
                if name not in self.by_name:
                    raise ValueError("missing joint %s" % name)


# ---------------------------------------------------------------------------
# MPU6050 over Linux i2c-dev, no third-party dependency
# ---------------------------------------------------------------------------


class Mpu6050:
    I2C_SLAVE = 0x0703

    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_CONFIG = 0x1C
    REG_ACCEL_XOUT_H = 0x3B
    REG_PWR_MGMT_1 = 0x6B
    REG_WHO_AM_I = 0x75

    def __init__(self, bus, addr=0x68):
        self.fd = os.open(bus, os.O_RDWR)
        self.addr = addr
        self.bus = bus
        fcntl.ioctl(self.fd, self.I2C_SLAVE, addr)
        who = self.read(REG_WHO_AM_I, 1)[0]
        if who != 0x68:
            raise IOError(
                "MPU6050 at 0x%02x on %s reports WHO_AM_I 0x%02x"
                % (addr, bus, who)
            )
        for reg, value in (
            (self.REG_PWR_MGMT_1, 0x00),
            (self.REG_SMPLRT_DIV, 0x09),
            (self.REG_CONFIG, 0x03),
            (self.REG_GYRO_CONFIG, 0x00),
            (self.REG_ACCEL_CONFIG, 0x00),
        ):
            self.write(reg, [value])
        time.sleep(0.05)
        self.pitch = 0.0
        self.roll = 0.0
        self.last = None
        self.sample()

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def write(self, reg, data):
        os.write(self.fd, bytes([reg] + list(data)))
        time.sleep(0.001)

    def read(self, reg, length):
        os.write(self.fd, bytes([reg]))
        time.sleep(0.001)
        data = os.read(self.fd, length)
        if len(data) != length:
            raise IOError("short I2C read from %s" % self.bus)
        return data

    @staticmethod
    def _s16(hi, lo):
        value = (hi << 8) | lo
        return value - 65536 if value & 0x8000 else value

    def sample(self):
        d = self.read(self.REG_ACCEL_XOUT_H, 14)
        ax = self._s16(d[0], d[1]) / 16384.0
        ay = self._s16(d[2], d[3]) / 16384.0
        az = self._s16(d[4], d[5]) / 16384.0
        gx = self._s16(d[8], d[9]) / 131.0
        gy = self._s16(d[10], d[11]) / 131.0
        gz = self._s16(d[12], d[13]) / 131.0
        now = time.time()
        dt = 0.0 if self.last is None else min(0.1, max(0.001, now - self.last))
        self.last = now

        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        if dt:
            self.roll = 0.98 * (self.roll + gx * dt) + 0.02 * roll_acc
            self.pitch = 0.98 * (self.pitch + gy * dt) + 0.02 * pitch_acc
        else:
            self.roll = roll_acc
            self.pitch = pitch_acc
        return {
            "accel_g": (ax, ay, az),
            "gyro_dps": (gx, gy, gz),
            "roll_deg": self.roll,
            "pitch_deg": self.pitch,
            "roll_acc_deg": roll_acc,
            "pitch_acc_deg": pitch_acc,
        }


# ---------------------------------------------------------------------------
# bus and rig
# ---------------------------------------------------------------------------


class Rig:
    def __init__(self, cfg, port=None, baud=None, timeout=0.05):
        self.cfg = cfg
        self.port = port or cfg.port or autodetect_port()
        self.baud = baud or cfg.baud
        self.bus = ScsBus(self.port, self.baud, timeout=timeout)
        self.joints = cfg.joints
        self.by_name = cfg.by_name
        self.imu = None
        self.torque_on = False

    def close(self):
        if self.imu:
            self.imu.close()
        self.bus.close()

    def attach_imu(self):
        bus = self.cfg.imu_cfg.get("bus", "/dev/i2c-2")
        addr = int(self.cfg.imu_cfg.get("addr", 0x68))
        self.imu = Mpu6050(bus, addr)
        return self.imu

    def imu_pitch(self, sign=1):
        if not self.imu:
            return 0.0
        return sign * self.imu.sample()["pitch_deg"]

    def ping_all(self):
        missing = []
        for joint in self.joints:
            if not self.bus.ping(joint.id):
                missing.append(joint)
        return missing

    def require_all(self):
        missing = self.ping_all()
        if missing:
            lines = ["no answer from: %s" % ", ".join(
                "%s(id %d)" % (j.name, j.id) for j in missing
            )]
            lines.append("check power, daisy-chain, jumper B and baud rate")
            raise SystemExit("\n".join(lines))

    def read_raw(self):
        out = {}
        for joint in self.joints:
            raw = self.bus.read_u16(joint.id, REG_PRESENT_POSITION)
            if raw is None:
                raise IOError("servo %s (id %d) did not answer"
                              % (joint.name, joint.id))
            out[joint.name] = raw
        return out

    def read_deg(self):
        return {n: self.by_name[n].deg(r) for n, r in self.read_raw().items()}

    def write_raw(self, targets):
        data = {}
        for name, raw in targets.items():
            joint = self.by_name[name]
            data[joint.id] = encode_u16(max(0, min(1024, int(raw))))
        self.bus.sync_write(0x2A, data, 2)

    def write_deg(self, targets):
        data = {}
        for name, deg in targets.items():
            joint = self.by_name[name]
            data[joint.id] = encode_u16(joint.raw(deg))
        self.bus.sync_write(0x2A, data, 2)

    def set_torque(self, on):
        for joint in self.joints:
            self.bus.torque(joint.id, on)
        self.torque_on = on

    def set_speed(self, speed):
        if speed:
            data = {j.id: encode_u16(speed) for j in self.joints}
            self.bus.sync_write(0x2E, data, 2)

    def move_deg(self, targets, duration, step=0.02, smooth=True):
        start = self.read_deg()
        steps = max(1, int(math.ceil(duration / step)))
        for i in range(1, steps + 1):
            t = i / float(steps)
            if smooth:
                t = 0.5 - 0.5 * math.cos(math.pi * t)
            frame = {}
            for name, target in targets.items():
                frame[name] = start[name] + (target - start[name]) * t
            self.write_deg(frame)
            time.sleep(step)

    def write_limits(self):
        for joint in self.joints:
            self.bus.write(joint.id, REG_MIN_ANGLE, encode_u16(joint.min_raw))
            self.bus.write(joint.id, REG_MAX_ANGLE, encode_u16(joint.max_raw))

    def read_limits(self):
        out = {}
        for joint in self.joints:
            lo = self.bus.read_u16(joint.id, REG_MIN_ANGLE)
            hi = self.bus.read_u16(joint.id, REG_MAX_ANGLE)
            out[joint.name] = (lo, hi)
        return out


# ---------------------------------------------------------------------------
# gait
# ---------------------------------------------------------------------------


def gait_offsets(phase, stride_deg, lift_deg, shift_deg, direction):
    """Joint deltas in degrees, around the home pose, for one gait cycle.

    Right leg is in stance on [0, 0.5), left on [0.5, 1).  The swing leg
    lifts with a half-sine and moves from back to front; the stance leg
    travels front to back.  `ankle = knee - hip` keeps the sole level.
    """
    t = phase % 1.0
    out = {}

    # Weight shift: right stance first, so +roll at t=0.
    roll = shift_deg * math.cos(2.0 * math.pi * t)

    for leg in ("left", "right"):
        side = 1.0 if leg == "left" else -1.0
        if leg == "right":
            stance = t < 0.5
            s = (t / 0.5) if stance else ((t - 0.5) / 0.5)
        else:
            stance = t >= 0.5
            s = ((t - 0.5) / 0.5) if stance else (t / 0.5)

        if stance:
            hip = -direction * side * (stride_deg / 2.0) * (1.0 - 2.0 * s)
            knee = 0.0
        else:
            swing = math.sin(math.pi * s)
            hip = -direction * side * (stride_deg / 2.0) * (2.0 * s - 1.0)
            hip += side * 0.5 * lift_deg * swing
            knee = side * lift_deg * swing

        ankle = knee - hip
        out["%s_hip_pitch" % leg] = hip
        out["%s_knee" % leg] = knee
        out["%s_ankle" % leg] = ankle
        out["%s_hip_roll" % leg] = roll
        out["%s_hip_yaw" % leg] = 0.0
    return out


def clamp_targets(cfg, offsets, home_scale=1.0, temperature=0.0):
    """Home pose + scaled gait offsets, clamped to every joint's limits."""
    targets = {}
    for joint in cfg.joints:
        deg = joint.home_deg + home_scale * offsets.get(joint.name, 0.0)
        if temperature:
            deg += temperature
        targets[joint.name] = max(joint.min_deg, min(joint.max_deg, deg))
    return targets


def gait_report(cfg, stride, lift, shift, direction, samples=8):
    print("phase  " + "  ".join("%-10s" % n for n in cfg.by_name))
    for i in range(samples + 1):
        phase = i / float(samples)
        offsets = gait_offsets(phase, stride, lift, shift, direction)
        targets = clamp_targets(cfg, offsets)
        print("%5.2f  " % phase + "  ".join(
            "%10.1f" % targets[n] for n in cfg.by_name
        ))
    print()
    print("joint                 deg range       raw range")
    for joint in cfg.joints:
        print("%-20s %6.1f .. %5.1f  %5d .. %5d"
              % (joint.name, joint.min_deg, joint.max_deg,
                 joint.min_raw, joint.max_raw))


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------


def cmd_scan(cfg, args):
    rig = Rig(cfg, args.port, args.baud)
    try:
        for joint in cfg.joints:
            if rig.bus.ping(joint.id):
                ver = rig.bus.read_u16(joint.id, REG_VERSION)
                print("  id %3d  %-16s OK  version=%s"
                      % (joint.id, joint.name,
                         "0x%04x" % ver if ver is not None else "?"))
            else:
                print("  id %3d  %-16s --" % (joint.id, joint.name))
    finally:
        rig.close()


def cmd_read(cfg, args):
    rig = Rig(cfg, args.port, args.baud)
    try:
        rig.require_all()
        raw = rig.read_raw()
        print("%-16s %4s %6s %8s %7s %6s %6s"
              % ("joint", "id", "raw", "deg", "volt", "temp", "load"))
        for joint in cfg.joints:
            sid = joint.id
            volt = rig.bus.read_u8(sid, REG_PRESENT_VOLTAGE)
            temp = rig.bus.read_u8(sid, REG_PRESENT_TEMPERATURE)
            load = rig.bus.read_u16(sid, REG_PRESENT_LOAD)
            print("%-16s %4d %6d %8.2f %6s %5s %6s"
                  % (joint.name, sid, raw[joint.name],
                     joint.deg(raw[joint.name]),
                     "%.1fV" % (volt / 10.0) if volt is not None else "?",
                     "%dC" % temp if temp is not None else "?",
                     str(load)))
        if args.imu:
            imu = rig.attach_imu()
            s = imu.sample()
            print("\nIMU  pitch=%+.2f deg  roll=%+.2f deg  accel=%s g"
                  % (s["pitch_deg"], s["roll_deg"],
                     tuple(round(v, 3) for v in s["accel_g"])))
    finally:
        rig.close()


def cmd_limits(cfg, args):
    rig = Rig(cfg, args.port, args.baud)
    try:
        rig.require_all()
        if args.write:
            print("writing EEPROM limits (torque is left as it is)")
            rig.write_limits()
            time.sleep(0.1)
        got = rig.read_limits()
        print("%-16s %6s %6s   %6s %6s   %s"
              % ("joint", "want-lo", "want-hi", "read-lo", "read-hi", "ok"))
        for joint in cfg.joints:
            lo, hi = got[joint.name]
            print("%-16s %6d %6d   %6s %6s   %s"
                  % (joint.name, joint.min_raw, joint.max_raw,
                     str(lo), str(hi),
                     "yes" if (lo, hi) == (joint.min_raw, joint.max_raw) else "NO"))
    finally:
        rig.close()


def cmd_center(cfg, args):
    targets = {j.name: 0.0 for j in cfg.joints}
    if args.dry_run:
        print(json.dumps({j.name: j.raw(0.0) for j in cfg.joints}, indent=2))
        return
    rig = Rig(cfg, args.port, args.baud)
    try:
        rig.require_all()
        rig.set_torque(True)
        print("moving all joints to the configured centre")
        rig.move_deg(targets, args.duration)
        raw = rig.read_raw()
        for joint in cfg.joints:
            print("  %-16s raw=%4d" % (joint.name, raw[joint.name]))
        if args.hold:
            print("holding centre; Ctrl-C to stop")
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
        if args.release:
            rig.set_torque(False)
            print("torque released")
    finally:
        rig.close()


def cmd_stand(cfg, args):
    targets = {j.name: j.home_deg for j in cfg.joints}
    if args.dry_run:
        print(json.dumps({j.name: j.raw(j.home_deg) for j in cfg.joints}, indent=2))
        return
    rig = Rig(cfg, args.port, args.baud)
    try:
        rig.require_all()
        rig.set_torque(True)
        print("ramping to home pose over %.1f s" % args.duration)
        rig.move_deg(targets, args.duration)
        if args.hold:
            print("holding home; Ctrl-C to stop")
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
        if args.release:
            rig.set_torque(False)
            print("torque released")
    finally:
        rig.close()


def cmd_move(cfg, args):
    joint = cfg.by_name[args.joint]
    deg = args.deg
    if deg < joint.min_deg or deg > joint.max_deg:
        raise SystemExit("%s target %.2f deg outside %.1f..%.1f"
                         % (joint.name, deg, joint.min_deg, joint.max_deg))
    rig = Rig(cfg, args.port, args.baud)
    try:
        rig.require_all()
        rig.set_torque(True)
        rig.move_deg({joint.name: deg}, args.duration)
        raw = rig.read_raw()
        print("%s: target %.2f deg = raw %d, read raw %d (%.2f deg)"
              % (joint.name, deg, joint.raw(deg), raw[joint.name],
                 joint.deg(raw[joint.name])))
        if args.hold:
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
        if args.release:
            rig.set_torque(False)
    finally:
        rig.close()


def cmd_imu(cfg, args):
    rig = Rig(cfg, args.port, args.baud)
    try:
        imu = rig.attach_imu()
        print("MPU6050 on %s@0x%02x; Ctrl-C to stop"
              % (cfg.imu_cfg.get("bus", "/dev/i2c-2"),
                 int(cfg.imu_cfg.get("addr", 0x68))))
        end = time.time() + args.seconds if args.seconds else None
        while end is None or time.time() < end:
            s = imu.sample()
            print("pitch=%+7.2f  roll=%+7.2f  gyro=(%+7.1f %+7.1f %+7.1f) dps"
                  % (s["pitch_deg"], s["roll_deg"], *s["gyro_dps"]))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rig.close()


def cmd_selftest(cfg, args):
    print("config: %s" % cfg.path)
    print("servos: " + ", ".join("%s=%d" % (j.name, j.id) for j in cfg.joints))
    print()
    gait_report(cfg, args.stride, args.lift, args.shift, 1.0)
    print("\nselftest OK")


def cmd_walk(cfg, args):
    direction = 1.0 if args.direction == "forward" else -1.0
    stride = 0.0 if args.march else args.stride
    lift = args.lift
    shift = args.shift
    period = args.period
    hz = args.hz
    step = 1.0 / hz
    cycles = args.cycles
    duration = period * cycles

    if args.dry_run:
        gait_report(cfg, stride, lift, shift, direction)
        return

    print("gait: %s  stride=%.1f deg  lift=%.1f deg  shift=%.1f deg  "
          "period=%.2f s  cycles=%.1f"
          % (args.direction, stride, lift, shift, period, cycles))

    rig = Rig(cfg, args.port, args.baud)
    try:
        rig.require_all()
        imu_sign = int(cfg.imu_cfg.get("pitch_sign", 1))
        if args.imu:
            try:
                rig.attach_imu()
                print("IMU attached: %s" % cfg.imu_cfg.get("bus", "/dev/i2c-2"))
            except Exception as e:
                print("IMU unavailable (%s); walking open-loop" % e)

        rig.set_torque(True)
        rig.set_speed(args.speed)
        home = {j.name: j.home_deg for j in cfg.joints}
        rig.move_deg(home, args.ramp_in)
        if args.pause:
            time.sleep(args.pause)

        ramp = max(0.001, args.ramp_in)
        t = 0.0
        next_tick = time.monotonic()
        try:
            while t <= duration:
                phase = (t / period) % 1.0
                amp = min(1.0, t / ramp, max(0.0, (duration - t) / ramp))
                offsets = gait_offsets(phase, stride, lift, shift, direction)
                targets = clamp_targets(cfg, offsets, home_scale=amp)

                if args.balance_gain and rig.imu:
                    pitch = rig.imu_pitch(imu_sign)
                    correction = max(-args.max_correction,
                                     min(args.max_correction,
                                         args.balance_gain * pitch))
                    for leg in ("left", "right"):
                        name = "%s_ankle" % leg
                        joint = cfg.by_name[name]
                        targets[name] = max(joint.min_deg,
                                            min(joint.max_deg,
                                                targets[name] + correction))

                rig.write_deg(targets)

                if args.verbose and int(t / step) % int(hz) == 0:
                    line = "t=%5.2f phase=%.2f" % (t, phase)
                    if rig.imu:
                        line += " pitch=%+.1f" % rig.imu_pitch(imu_sign)
                    print(line)

                t += step
                next_tick += step
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            rig.set_torque(False)
            print("\ninterrupted; torque released")
            return

        rig.move_deg(home, args.ramp_out)
        if args.release:
            rig.set_torque(False)
            print("torque released")
        else:
            print("holding home; Ctrl-C or --release to let go")
    finally:
        rig.close()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def add_common(p):
    p.add_argument("--config", default=DEFAULT_CONFIG,
                   help="joint/ID/limit table (default: legs_config.json)")
    p.add_argument("--port", default=None,
                   help="serial port (default: config or first ttyUSB/ttyACM)")
    p.add_argument("--baud", type=int, default=None,
                   help="bus baud rate (default: config, 1000000)")


def build_parser():
    common = argparse.ArgumentParser(add_help=False)
    add_common(common)

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[common],
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("scan", parents=[common],
                   help="ping every configured servo")
    p = sub.add_parser("read", parents=[common],
                       help="read position, voltage, temperature and load")
    p.add_argument("--imu", action="store_true", help="also read the MPU6050")

    p = sub.add_parser("limits", parents=[common],
                       help="show or write SCS min/max angle limits")
    p.add_argument("--write", action="store_true", help="write EEPROM limits")

    p = sub.add_parser("center", parents=[common],
                       help="move every joint to its configured centre")
    p.add_argument("--duration", type=float, default=2.0)
    p.add_argument("--hold", action="store_true", help="keep torque on and stay")
    p.add_argument("--release", action="store_true", help="release torque at exit")
    p.add_argument("--dry-run", action="store_true")

    p = sub.add_parser("stand", parents=[common],
                       help="ramp to the configured home/stand pose")
    p.add_argument("--duration", type=float, default=2.0)
    p.add_argument("--hold", action="store_true")
    p.add_argument("--release", action="store_true")
    p.add_argument("--dry-run", action="store_true")

    p = sub.add_parser("move", parents=[common],
                       help="move one joint to an angle for direction checks")
    p.add_argument("--joint", required=True)
    p.add_argument("--deg", type=float, required=True)
    p.add_argument("--duration", type=float, default=1.0)
    p.add_argument("--hold", action="store_true")
    p.add_argument("--release", action="store_true")

    p = sub.add_parser("walk", parents=[common],
                       help="open-loop forward/backward gait (pre-RL test)")
    p.add_argument("--direction", choices=("forward", "backward"),
                   default="forward")
    p.add_argument("--cycles", type=float, default=5.0)
    p.add_argument("--period", type=float, default=1.2)
    p.add_argument("--stride", type=float, default=10.0,
                   help="total fore/aft hip travel, degrees")
    p.add_argument("--lift", type=float, default=12.0,
                   help="knee lift at mid-swing, degrees")
    p.add_argument("--shift", type=float, default=4.0,
                   help="lateral weight shift, degrees")
    p.add_argument("--march", action="store_true",
                   help="lift legs in place (stride = 0)")
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--speed", type=int, default=0,
                   help="SCS goal speed in counts/s, 0 = fastest")
    p.add_argument("--imu", action="store_true",
                   help="use MPU6050 pitch feedback")
    p.add_argument("--balance-gain", type=float, default=0.0,
                   help="ankle correction per degree of pitch (try 0.1)")
    p.add_argument("--max-correction", type=float, default=5.0)
    p.add_argument("--ramp-in", type=float, default=0.8,
                   help="home ramp and gait amplitude ramp-in, seconds")
    p.add_argument("--ramp-out", type=float, default=0.8)
    p.add_argument("--pause", type=float, default=0.0)
    p.add_argument("--release", action="store_true")
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--dry-run", action="store_true")

    p = sub.add_parser("imu", parents=[common],
                       help="print live MPU6050 pitch/roll")
    p.add_argument("--seconds", type=float, default=5.0)

    p = sub.add_parser("selftest", parents=[common],
                       help="check gait/limit maths without hardware")
    p.add_argument("--stride", type=float, default=10.0)
    p.add_argument("--lift", type=float, default=12.0)
    p.add_argument("--shift", type=float, default=4.0)
    return ap


def main():
    args = build_parser().parse_args()
    cfg = Config(args.config)
    handlers = {
        "scan": cmd_scan,
        "read": cmd_read,
        "limits": cmd_limits,
        "center": cmd_center,
        "stand": cmd_stand,
        "move": cmd_move,
        "walk": cmd_walk,
        "imu": cmd_imu,
        "selftest": cmd_selftest,
    }
    handlers[args.cmd](cfg, args)


if __name__ == "__main__":
    main()
