//! The Feetech SCS bus (SCS225 and the SCS series).
//!
//! Written against `serialport` directly rather than through `rustypot` because the SCS
//! series differs from the Dynamixel protocol in ways the shared layer cannot express:
//! 16-bit registers are high-byte-first, positions run 0..=1023 around a 511 centre, and
//! the sync-write instruction is `0x83`. See [`crate::io::RobotIo`] for the seam a
//! higher layer implements on top of this.
//!
//! [`ScsBus`] is deliberately a plain bus: it knows registers, not joints. Which servo is
//! which joint, how a raw count maps to radians, and where the IMU comes from all live
//! above it.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, SerialPort};

use crate::io::{IoError, JointTargets, Result, RobotIo, Sensors, SlowSensors};
use crate::model::NUM_JOINTS;

pub const REG_VERSION: u8 = 0x03;
pub const REG_ID: u8 = 0x05;
pub const REG_BAUD_RATE: u8 = 0x06;
pub const REG_MIN_ANGLE_LIMIT: u8 = 0x09;
pub const REG_MAX_ANGLE_LIMIT: u8 = 0x0B;
pub const REG_MAX_TEMPERATURE: u8 = 0x0D;
pub const REG_MAX_VOLTAGE: u8 = 0x0E;
pub const REG_MIN_VOLTAGE: u8 = 0x0F;
pub const REG_MAX_TORQUE: u8 = 0x10;
pub const REG_POSITION_P_GAIN: u8 = 0x15;
pub const REG_POSITION_D_GAIN: u8 = 0x16;

pub const REG_TORQUE_ENABLE: u8 = 0x28;
pub const REG_GOAL_POSITION: u8 = 0x2A;
pub const REG_GOAL_TIME: u8 = 0x2C;
pub const REG_GOAL_SPEED: u8 = 0x2E;

pub const REG_PRESENT_POSITION: u8 = 0x38;
pub const REG_PRESENT_SPEED: u8 = 0x3A;
pub const REG_PRESENT_LOAD: u8 = 0x3C;
pub const REG_PRESENT_VOLTAGE: u8 = 0x3E;
pub const REG_PRESENT_TEMPERATURE: u8 = 0x3F;
pub const REG_STATUS: u8 = 0x41;
pub const REG_MOVING: u8 = 0x42;

/// Position counts per revolution of the servo's rated travel, and the centre count.
///
/// The SCS225 spans 300° over 0..=1024 with 511 at mechanical centre. Kept here as plain
/// numbers: the radian conversion belongs to whoever maps a servo onto a joint.
pub const POSITION_MIN: u16 = 0;
pub const POSITION_MAX: u16 = 1024;
pub const POSITION_CENTER: u16 = 511;

/// The broadcast id a `sync_write` addresses.
const BROADCAST_ID: u8 = 0xFE;

/// Instruction set, as used here.
const INST_PING: u8 = 0x01;
const INST_READ: u8 = 0x02;
const INST_WRITE: u8 = 0x03;
const INST_SYNC_WRITE: u8 = 0x83;

pub struct ScsBus {
    port: Box<dyn SerialPort>,
    timeout: Duration,
    retries: u32,
}

impl ScsBus {
    /// Open a bus. `timeout` caps one transaction, not one byte; a bus that answers slowly
    /// but does answer is still a working bus.
    pub fn open(path: &str, baud: u32, timeout: Duration) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(|e| IoError::Bus(format!("open {path}: {e}")))?;
        Ok(Self {
            port,
            timeout,
            retries: 3,
        })
    }

    fn checksum(payload: &[u8]) -> u8 {
        !payload.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
    }

    fn read_within(&mut self, len: usize, deadline: Instant) -> Option<Vec<u8>> {
        let mut buf = vec![0u8; len];
        let mut got = 0;
        while got < len {
            if Instant::now() >= deadline {
                return None;
            }
            match self.port.read(&mut buf[got..]) {
                Ok(0) => return None,
                Ok(n) => got += n,
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(_) => return None,
            }
        }
        Some(buf)
    }

    /// One request/reply, retried. `None` is "no well-formed answer", which on this bus is
    /// what a missing servo, a wrong id or a wrong baud rate all look like.
    fn txn(&mut self, sid: u8, instr: u8, params: &[u8], resp_len: usize) -> Option<Vec<u8>> {
        let mut body = vec![sid, (params.len() + 2) as u8, instr];
        body.extend_from_slice(params);
        let cks = Self::checksum(&body);
        let mut pkt = vec![0xFF, 0xFF];
        pkt.extend_from_slice(&body);
        pkt.push(cks);
        for _ in 0..=self.retries {
            let _ = self.port.clear(ClearBuffer::Input);
            if self.port.write_all(&pkt).is_err() {
                continue;
            }
            if let Some(r) = self.read_within(resp_len, Instant::now() + self.timeout) {
                if r.len() == resp_len && r[0] == 0xFF && r[1] == 0xFF && r[2] == sid {
                    return Some(r);
                }
            }
        }
        None
    }

    pub fn ping(&mut self, sid: u8) -> bool {
        matches!(self.txn(sid, INST_PING, &[], 6), Some(r) if r[4] == 0)
    }

    pub fn read(&mut self, sid: u8, addr: u8, len: u8) -> Option<Vec<u8>> {
        let r = self.txn(sid, INST_READ, &[addr, len], 6 + len as usize)?;
        if r[4] != 0 {
            return None;
        }
        Some(r[5..5 + len as usize].to_vec())
    }

    pub fn write(&mut self, sid: u8, addr: u8, data: &[u8]) {
        let mut params = vec![addr];
        params.extend_from_slice(data);
        let _ = self.txn(sid, INST_WRITE, &params, 6);
    }

    /// Broadcast `sync_write`: one packet, every listed servo. No reply is returned for
    /// the broadcast, so a write that reached nobody is indistinguishable from one that
    /// reached everybody — reads are what prove the bus is alive.
    pub fn sync_write(&mut self, addr: u8, data_len: u8, entries: &[(u8, Vec<u8>)]) {
        let mut params = vec![addr, data_len];
        for (sid, data) in entries {
            params.push(*sid);
            params.extend_from_slice(data);
        }
        let mut body = vec![BROADCAST_ID, (params.len() + 2) as u8, INST_SYNC_WRITE];
        body.extend_from_slice(&params);
        let cks = Self::checksum(&body);
        let mut pkt = vec![0xFF, 0xFF];
        pkt.extend_from_slice(&body);
        pkt.push(cks);
        let _ = self.port.clear(ClearBuffer::Input);
        let _ = self.port.write_all(&pkt);
    }

    pub fn read_u16(&mut self, sid: u8, addr: u8) -> Option<u16> {
        self.read(sid, addr, 2)
            .map(|d| u16::from_be_bytes([d[0], d[1]]))
    }

    pub fn read_u8(&mut self, sid: u8, addr: u8) -> Option<u8> {
        self.read(sid, addr, 1).map(|d| d[0])
    }

    pub fn torque_enable(&mut self, sid: u8, on: bool) {
        self.write(sid, REG_TORQUE_ENABLE, &[u8::from(on)]);
    }

    pub fn goal_position(&mut self, sid: u8, position: u16) {
        let p = position.clamp(POSITION_MIN, POSITION_MAX);
        self.write(sid, REG_GOAL_POSITION, &p.to_be_bytes());
    }

    /// Version word at 0x03: the model/firmware identity. `0x0923` on an SCS225.
    pub fn version(&mut self, sid: u8) -> Option<u16> {
        self.read_u16(sid, REG_VERSION)
    }
}

// ── the robot side of the bus ────────────────────────────────────────────────
//
// Provisional until the robot exists: thirteen servos fill the first thirteen joint
// slots and the last two report zero and ignore commands. `slots` is where the real
// mapping goes once the joints are known.

/// Radians per position count, from the SCS225's 300° over 0..=1024.
pub const RAD_PER_COUNT: f64 = 300.0 * std::f64::consts::PI / 180.0 / 1024.0;

pub fn raw_to_rad(raw: u16) -> f64 {
    (raw as f64 - POSITION_CENTER as f64) * RAD_PER_COUNT
}

pub fn rad_to_raw(rad: f64) -> u16 {
    let counts = rad / RAD_PER_COUNT + POSITION_CENTER as f64;
    counts.round().clamp(POSITION_MIN as f64, POSITION_MAX as f64) as u16
}

pub struct ScsIo {
    bus: ScsBus,
    ids: Vec<u8>,
    slots: Vec<usize>,
    /// Which half of the servos has its speed read this tick. See [`RobotIo::read`].
    speed_phase: bool,
    /// Speeds held between refreshes, indexed as joint slots.
    last_velocities: [f64; NUM_JOINTS],
    /// The orientation source, when one is fitted. The SCS bus carries no IMU, so this is
    /// a separate part on a separate bus; `None` leaves [`Sensors::imu`] at its default.
    imu: Option<crate::mpu6050::Mpu6050>,
}

impl ScsIo {
    /// `slots[i]` is the joint index servo `ids[i]` drives.
    pub fn open(path: &str, baud: u32, ids: Vec<u8>, slots: Vec<usize>) -> Result<Self> {
        let bus = ScsBus::open(path, baud, Duration::from_millis(60))?;
        Ok(Self {
            bus,
            ids,
            slots,
            speed_phase: false,
            last_velocities: [0.0; NUM_JOINTS],
            imu: None,
        })
    }

    /// Attach an already-opened IMU. Not part of `open` because a robot without one must
    /// still be able to run the bus — a missing sensor is a degraded robot, not a dead one.
    pub fn attach_imu(&mut self, imu: crate::mpu6050::Mpu6050) {
        self.imu = Some(imu);
    }

    pub fn ids(&self) -> &[u8] {
        &self.ids
    }

    pub fn present_positions(&mut self) -> Result<Vec<f64>> {
        let mut out = Vec::with_capacity(self.ids.len());
        for &sid in &self.ids {
            let pos = self
                .bus
                .read_u16(sid, REG_PRESENT_POSITION)
                .ok_or_else(|| IoError::Bus(format!("servo {sid} did not answer")))?;
            out.push(raw_to_rad(pos));
        }
        Ok(out)
    }

    /// The SCS analogue of the Dynamixel register check: every servo must answer.
    ///
    /// There is no EEPROM register to assert yet — the SCS225's `return_delay_time` and
    /// gains are written elsewhere — so this is a liveness check, and the count it returns
    /// is always zero.
    pub fn check_registers(&mut self) -> Result<usize> {
        let missing: Vec<u8> = self
            .ids
            .iter()
            .copied()
            .filter(|&sid| !self.bus.ping(sid))
            .collect();
        if missing.is_empty() {
            Ok(0)
        } else {
            Err(IoError::Bus(format!("no answer from servos {missing:?}")))
        }
    }

    /// Ramp every mapped joint from where it is now to `target`, linearly. Used by
    /// `robotd init`, never by the control loop.
    pub fn interpolate_to(
        &mut self,
        target: &[f64; NUM_JOINTS],
        duration: Duration,
        step: Duration,
    ) -> Result<()> {
        let start = self.present_positions()?;
        let steps = (duration.as_secs_f64() / step.as_secs_f64()).ceil().max(1.0) as u32;
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let entries: Vec<(u8, Vec<u8>)> = self
                .ids
                .iter()
                .enumerate()
                .filter_map(|(k, &sid)| {
                    let slot = self.slots.get(k).copied()?;
                    if slot >= NUM_JOINTS {
                        return None;
                    }
                    let from = start.get(k).copied().unwrap_or(0.0);
                    let value = from + (target[slot] - from) * t;
                    Some((sid, rad_to_raw(value).to_be_bytes().to_vec()))
                })
                .collect();
            self.bus.sync_write(REG_GOAL_POSITION, 2, &entries);
            std::thread::sleep(step);
        }
        Ok(())
    }
}

impl RobotIo for ScsIo {
    /// Positions for every servo every tick; speeds for half of them, alternating.
    ///
    /// **Not one combined read.** Measured on SCS225 hardware: a single READ of four bytes
    /// or more fails every time, and five and above never answer at all, while one to three
    /// bytes answer reliably. The datasheet's contiguous 0x38..0x3D block is therefore not
    /// readable in one transaction.
    ///
    /// **And not a speed read per servo per tick either.** A transaction costs about 0.7 ms
    /// through the USB adapter, so thirteen positions plus thirteen speeds is ~19 ms before
    /// the policy runs: the loop sat at 30 Hz and the update gate called it unhealthy.
    /// Reading half the speeds per tick brings the tick back inside the 20 ms budget at full
    /// resolution; a joint's velocity is then at most 40 ms old, the same order as the
    /// servo's own measurement window. SCS sync-read (`0x82`) would solve this properly and
    /// this firmware does not implement it — verified, it answers nothing.
    fn read(&mut self) -> Result<Sensors> {
        let mut sensors = Sensors::default();
        let phase = self.speed_phase;
        for (i, &sid) in self.ids.iter().enumerate() {
            let position = self
                .bus
                .read_u16(sid, REG_PRESENT_POSITION)
                .ok_or_else(|| IoError::Bus(format!("servo {sid} did not answer")))?;
            let slot = self.slots.get(i).copied().unwrap_or(usize::MAX);
            if slot < NUM_JOINTS {
                sensors.positions[slot] = raw_to_rad(position);
                if (i % 2 == 0) == phase {
                    let speed = self.bus.read_u16(sid, REG_PRESENT_SPEED).ok_or_else(|| {
                        IoError::Bus(format!("servo {sid} speed did not answer"))
                    })?;
                    self.last_velocities[slot] = (speed as i16) as f64 * RAD_PER_COUNT;
                }
                sensors.velocities[slot] = self.last_velocities[slot];
            }
        }
        self.speed_phase = !phase;
        // A failed IMU sample keeps the last one rather than failing the tick: the servos
        // answered, and freezing the pose because a second sensor hiccuped is worse than
        // feeding the loop a sample one tick old.
        if let Some(imu) = self.imu.as_mut() {
            sensors.imu = imu.sample().unwrap_or_else(|_| imu.last());
        }
        Ok(sensors)
    }

    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        let entries: Vec<(u8, Vec<u8>)> = self
            .ids
            .iter()
            .enumerate()
            .filter_map(|(i, &sid)| {
                let slot = self.slots.get(i).copied()?;
                if slot >= NUM_JOINTS {
                    return None;
                }
                Some((sid, rad_to_raw(targets.positions[slot]).to_be_bytes().to_vec()))
            })
            .collect();
        self.bus.sync_write(REG_GOAL_POSITION, 2, &entries);
        Ok(())
    }

    fn set_gain(&mut self, kp: u16) -> Result<()> {
        let value = kp.min(254) as u8;
        for &sid in &self.ids {
            self.bus.write(sid, REG_POSITION_P_GAIN, &[value]);
            self.bus.write(sid, REG_POSITION_D_GAIN, &[0]);
        }
        Ok(())
    }

    fn set_torque(&mut self, on: bool) -> Result<()> {
        let mut failed = Vec::new();
        for &sid in &self.ids {
            let before = self.bus.ping(sid);
            self.bus.torque_enable(sid, on);
            if !before {
                failed.push(sid);
            }
        }
        if failed.is_empty() {
            Ok(())
        } else {
            Err(IoError::Bus(format!("no answer from servos {failed:?}")))
        }
    }

    fn slow_sensors(&mut self) -> Result<SlowSensors> {
        let mut volts = Vec::with_capacity(self.ids.len());
        let mut temps_c = [0.0; NUM_JOINTS];
        for (i, &sid) in self.ids.iter().enumerate() {
            let block = self
                .bus
                .read(sid, REG_PRESENT_VOLTAGE, 2)
                .ok_or_else(|| IoError::Bus(format!("servo {sid} did not answer")))?;
            volts.push(block[0] as f64 * 0.1);
            let slot = self.slots.get(i).copied().unwrap_or(usize::MAX);
            if slot < NUM_JOINTS {
                temps_c[slot] = block[1] as f64;
            }
        }
        if volts.is_empty() {
            return Err(IoError::Bus("no voltage readings".into()));
        }
        Ok(SlowSensors {
            volts: volts.iter().sum::<f64>() / volts.len() as f64,
            temps_c,
        })
    }

    /// True once the IMU has produced a sample. With no IMU fitted the trait default (true)
    /// stands, because there is no filter to converge and refusing would keep fall detection
    /// permanently disabled on a robot that simply has no orientation sensor.
    fn imu_ready(&self) -> bool {
        match &self.imu {
            Some(imu) => imu.ready(),
            None => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_conversion_round_trips() {
        for raw in [0u16, 1, 255, 511, 700, 1024] {
            let back = rad_to_raw(raw_to_rad(raw));
            assert!((back as i32 - raw as i32).abs() <= 1, "raw {raw} -> {back}");
        }
    }

    #[test]
    fn centre_is_zero_radians() {
        assert_eq!(raw_to_rad(POSITION_CENTER), 0.0);
    }
}
