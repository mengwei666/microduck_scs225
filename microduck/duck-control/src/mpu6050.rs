//! The MPU6050 IMU, over I2C.
//!
//! Unlike the shipped `imu_to_dxl` board — which fuses on-chip and rides the Dynamixel bus —
//! this is a bare six-axis part: raw accelerometer and gyroscope, no fusion. Projected
//! gravity is the normalised accelerometer (specific force, negated), and the quaternion is
//! the tilt-only rotation taking that gravity to world down. **Yaw is not observable** from
//! an accelerometer and a gyro alone, so the quaternion carries no heading; anything that
//! needs one has to get it elsewhere.
//!
//! The accelerometer is read at ±2 g and the gyroscope at ±250 °/s, both factory defaults
//! for the part.

use std::f64::consts::PI;
use std::time::Duration;

use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;

use crate::imu::ImuData;
use crate::io::{IoError, Result};

const REG_SMPLRT_DIV: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_WHO_AM_I: u8 = 0x75;

const WHO_AM_I_MPU6050: u8 = 0x68;

/// ±2 g full scale.
const ACCEL_LSB_PER_G: f64 = 16384.0;
/// ±250 °/s full scale.
const GYRO_LSB_PER_DPS: f64 = 131.0;

/// One IMU on one I2C bus.
pub struct Mpu6050 {
    dev: LinuxI2CDevice,
    ready: bool,
    last: ImuData,
}

impl Mpu6050 {
    /// Open, verify the chip and configure it for 100 Hz sampling with a 44 Hz DLPF.
    pub fn open(path: &str, addr: u16) -> Result<Self> {
        let mut dev = LinuxI2CDevice::new(path, addr)
            .map_err(|e| IoError::Bus(format!("open imu {path}@0x{addr:02x}: {e}")))?;
        let who = dev
            .smbus_read_byte_data(REG_WHO_AM_I)
            .map_err(|e| IoError::Bus(format!("imu WHO_AM_I: {e}")))?;
        if who != WHO_AM_I_MPU6050 {
            return Err(IoError::Bus(format!(
                "imu at 0x{addr:02x} reports WHO_AM_I 0x{who:02x}, expected 0x{WHO_AM_I_MPU6050:02x}"
            )));
        }
        for (reg, value) in [
            (REG_PWR_MGMT_1, 0x00),   // wake, internal 8 MHz clock
            (REG_SMPLRT_DIV, 0x09),   // 1 kHz / (1 + 9) = 100 Hz
            (REG_CONFIG, 0x03),       // DLPF 44 Hz
            (REG_GYRO_CONFIG, 0x00),  // ±250 °/s
            (REG_ACCEL_CONFIG, 0x00), // ±2 g
        ] {
            dev.smbus_write_byte_data(reg, value)
                .map_err(|e| IoError::Bus(format!("imu write 0x{reg:02x}: {e}")))?;
        }
        std::thread::sleep(Duration::from_millis(50));
        Ok(Self {
            dev,
            ready: false,
            last: ImuData::default(),
        })
    }

    /// One sample: 14 bytes from `ACCEL_XOUT_H` — accel, temperature, gyro.
    pub fn sample(&mut self) -> Result<ImuData> {
        // `smbus_read_i2c_block_data` is the plain register-then-read transfer (two messages,
        // no SMBus length byte), which is what the MPU6050's burst read expects.
        let d = self
            .dev
            .smbus_read_i2c_block_data(REG_ACCEL_XOUT_H, 14)
            .map_err(|e| IoError::Bus(format!("imu burst read: {e}")))?;
        if d.len() != 14 {
            return Err(IoError::ShortRead {
                what: "imu burst",
                expected: 14,
                got: d.len(),
            });
        }
        let raw = |o: usize| i16::from_be_bytes([d[o], d[o + 1]]) as f64;

        let accel = [
            raw(0) / ACCEL_LSB_PER_G,
            raw(2) / ACCEL_LSB_PER_G,
            raw(4) / ACCEL_LSB_PER_G,
        ];
        let gyro = [
            raw(8) / GYRO_LSB_PER_DPS * PI / 180.0,
            raw(10) / GYRO_LSB_PER_DPS * PI / 180.0,
            raw(12) / GYRO_LSB_PER_DPS * PI / 180.0,
        ];

        // Specific force is `a - g`, so at rest the accelerometer points *up*; projected
        // gravity is its negation, which puts upright at `[0, 0, -1]` like the shipped IMU.
        let gravity = normalise([-accel[0], -accel[1], -accel[2]]);
        let quat = tilt_quaternion(gravity);

        self.ready = true;
        self.last = ImuData {
            gyro,
            gravity,
            quat,
        };
        Ok(self.last)
    }

    pub fn ready(&self) -> bool {
        self.ready
    }

    pub fn last(&self) -> ImuData {
        self.last
    }
}

/// Unit vector, falling back to "upright" rather than dividing by ~zero.
fn normalise(v: [f64; 3]) -> [f64; 3] {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if mag > 1e-6 {
        [v[0] / mag, v[1] / mag, v[2] / mag]
    } else {
        [0.0, 0.0, -1.0]
    }
}

/// The rotation that carries `gravity` onto world down, scalar-first.
///
/// The shortest such rotation has no yaw component, which is exactly what an accelerometer
/// can say; any heading the robot had is lost. A robot standing exactly on its head is the
/// degenerate case, and gets a 180° flip about X rather than a NaN.
fn tilt_quaternion(gravity: [f64; 3]) -> [f64; 4] {
    let down = [0.0, 0.0, -1.0];
    let dot = gravity[0] * down[0] + gravity[1] * down[1] + gravity[2] * down[2];
    let axis = [
        gravity[1] * down[2] - gravity[2] * down[1],
        gravity[2] * down[0] - gravity[0] * down[2],
        gravity[0] * down[1] - gravity[1] * down[0],
    ];
    let axis_len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if axis_len < 1e-9 {
        if dot >= 0.0 {
            [1.0, 0.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0, 0.0]
        }
    } else {
        let angle = axis_len.atan2(dot);
        let half = angle / 2.0;
        let s = half.sin() / axis_len;
        [half.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Upright must be the identity — the control loop treats `[0, 0, -1]` as level, and a
    /// non-identity here would tilt every observation.
    #[test]
    fn upright_gravity_is_identity_rotation() {
        let q = tilt_quaternion([0.0, 0.0, -1.0]);
        assert!((q[0] - 1.0).abs() < 1e-9);
        assert!(q[1].abs() < 1e-9 && q[2].abs() < 1e-9 && q[3].abs() < 1e-9);
    }

    /// A 90° tip about Y is a real case: the robot on its side. The rotation that takes the
    /// measured gravity back to world down must be 90° about Y.
    #[test]
    fn tip_about_y_is_ninety_degrees() {
        let g = [1.0, 0.0, 0.0];
        let q = tilt_quaternion(g);
        let expected_half = std::f64::consts::FRAC_PI_4;
        assert!((q[0] - expected_half.cos()).abs() < 1e-9, "{q:?}");
        assert!((q[2] - expected_half.sin()).abs() < 1e-9, "{q:?}");
    }

    /// Standing on the head is the degenerate antiparallel case: no axis is defined, and the
    /// answer must be a real 180° rotation rather than NaN.
    #[test]
    fn upside_down_is_a_clean_flip() {
        let q = tilt_quaternion([0.0, 0.0, 1.0]);
        assert!(q.iter().all(|v| v.is_finite()), "{q:?}");
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        assert!((norm - 1.0).abs() < 1e-9);
    }

    /// A static accelerometer reading of +1 g must arrive as upright.
    #[test]
    fn accel_negation_maps_up_to_down() {
        let g = normalise([-0.0, -0.0, -1.0]);
        assert_eq!(g, [0.0, 0.0, -1.0]);
    }
}
