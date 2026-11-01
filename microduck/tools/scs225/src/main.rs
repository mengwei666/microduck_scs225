//! Feetech SCS225 bench tool: scan, read, move and exercise the bus.
//!
//! The SCS series stores 16-bit registers high-byte-first (big-endian), unlike
//! the SMS series. Position range is 0..=1023 with 511 at centre.

use std::io::{Read, Write};
use std::process::exit;
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, SerialPort};

const REG_VERSION: u8 = 0x03;
const REG_TORQUE_ENABLE: u8 = 0x28;
const REG_GOAL_POSITION: u8 = 0x2A;
const REG_GOAL_SPEED: u8 = 0x2E;
const REG_PRESENT_POSITION: u8 = 0x38;
const REG_PRESENT_SPEED: u8 = 0x3A;
const REG_PRESENT_LOAD: u8 = 0x3C;
const REG_PRESENT_VOLTAGE: u8 = 0x3E;
const REG_PRESENT_TEMPERATURE: u8 = 0x3F;

const POS_MIN: i32 = 0;
const POS_MAX: i32 = 1024;
const POS_CENTER: i32 = 511;

struct ScsBus {
    port: Box<dyn SerialPort>,
    timeout: Duration,
    retries: u32,
}

#[derive(Debug)]
enum BusError {
    Open(String),
    Io(String),
}

impl std::fmt::Display for BusError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BusError::Open(m) => write!(f, "open: {m}"),
            BusError::Io(m) => write!(f, "io: {m}"),
        }
    }
}

impl ScsBus {
    fn open(path: &str, baud: u32, timeout: Duration) -> Result<Self, BusError> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(20))
            .open()
            .map_err(|e| BusError::Open(e.to_string()))?;
        Ok(Self {
            port,
            timeout,
            retries: 2,
        })
    }

    fn checksum(payload: &[u8]) -> u8 {
        !payload.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
    }

    fn read_exact_within(&mut self, len: usize, deadline: Instant) -> Option<Vec<u8>> {
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
            if let Some(r) = self.read_exact_within(resp_len, Instant::now() + self.timeout) {
                if r.len() == resp_len && r[0] == 0xFF && r[1] == 0xFF && r[2] == sid {
                    return Some(r);
                }
            }
        }
        None
    }

    fn ping(&mut self, sid: u8) -> bool {
        matches!(self.txn(sid, 0x01, &[], 6), Some(r) if r[4] == 0)
    }

    fn read(&mut self, sid: u8, addr: u8, len: u8) -> Option<Vec<u8>> {
        let r = self.txn(sid, 0x02, &[addr, len], 6 + len as usize)?;
        if r[4] != 0 {
            return None;
        }
        Some(r[5..5 + len as usize].to_vec())
    }

    fn write(&mut self, sid: u8, addr: u8, data: &[u8]) {
        let mut params = vec![addr];
        params.extend_from_slice(data);
        let _ = self.txn(sid, 0x03, &params, 6);
    }

    fn sync_write(&mut self, addr: u8, data_len: u8, entries: &[(u8, Vec<u8>)]) {
        let mut params = vec![addr, data_len];
        for (sid, data) in entries {
            params.push(*sid);
            params.extend_from_slice(data);
        }
        let mut body = vec![0xFE, (params.len() + 2) as u8, 0x83];
        body.extend_from_slice(&params);
        let cks = Self::checksum(&body);
        let mut pkt = vec![0xFF, 0xFF];
        pkt.extend_from_slice(&body);
        pkt.push(cks);
        let _ = self.port.clear(ClearBuffer::Input);
        let _ = self.port.write_all(&pkt);
    }

    fn read_u16(&mut self, sid: u8, addr: u8) -> Option<u16> {
        self.read(sid, addr, 2)
            .map(|d| u16::from_be_bytes([d[0], d[1]]))
    }

    fn read_u8(&mut self, sid: u8, addr: u8) -> Option<u8> {
        self.read(sid, addr, 1).map(|d| d[0])
    }

    fn torque(&mut self, sid: u8, on: bool) {
        self.write(sid, REG_TORQUE_ENABLE, &[if on { 1 } else { 0 }]);
    }

    fn goal(&mut self, sid: u8, position: i32, speed: u16) {
        let p = position.clamp(POS_MIN, POS_MAX) as u16;
        self.write(sid, REG_GOAL_POSITION, &p.to_be_bytes());
        if speed != 0 {
            self.write(sid, REG_GOAL_SPEED, &speed.to_be_bytes());
        }
    }
}

fn encode_u16(v: u16) -> Vec<u8> {
    v.to_be_bytes().to_vec()
}

fn parse_ids(spec: &str) -> Vec<u8> {
    let mut ids = Vec::new();
    for part in spec.split(',') {
        if let Some((lo, hi)) = part.split_once('-') {
            if let (Ok(lo), Ok(hi)) = (lo.parse::<u8>(), hi.parse::<u8>()) {
                for i in lo..=hi {
                    ids.push(i);
                }
            }
        } else if let Ok(v) = part.parse::<u8>() {
            ids.push(v);
        }
    }
    ids.retain(|&i| i <= 253);
    ids
}

struct Args {
    port: String,
    baud: u32,
    ids: Vec<u8>,
    cmd: String,
    position: Option<i32>,
    speed: u16,
    amplitude: i32,
    period: f64,
    cycles: u32,
}

fn parse_args() -> Result<Args, String> {
    let mut port = String::new();
    let mut baud = 1_000_000u32;
    let mut ids_spec = String::from("1-13");
    let mut cmd = String::new();
    let mut position = None;
    let mut speed = 0u16;
    let mut amplitude = 120i32;
    let mut period = 1.5f64;
    let mut cycles = 3u32;

    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut take = |name: &str| -> Result<String, String> {
            it.next().ok_or_else(|| format!("{name} needs a value"))
        };
        match arg.as_str() {
            "--port" => port = take("--port")?,
            "--baud" => baud = take("--baud")?.parse().map_err(|e| format!("baud: {e}"))?,
            "--ids" => ids_spec = take("--ids")?,
            "--position" => {
                position = Some(take("--position")?.parse().map_err(|e| format!("position: {e}"))?)
            }
            "--speed" => speed = take("--speed")?.parse().map_err(|e| format!("speed: {e}"))?,
            "--amplitude" => {
                amplitude = take("--amplitude")?.parse().map_err(|e| format!("amplitude: {e}"))?
            }
            "--period" => period = take("--period")?.parse().map_err(|e| format!("period: {e}"))?,
            "--cycles" => cycles = take("--cycles")?.parse().map_err(|e| format!("cycles: {e}"))?,
            "--help" | "-h" => {
                return Err("usage: scs225 [--port P] [--baud N] [--ids 1-13] \
                     <scan|read|move --position N|demo --amplitude N --period S --cycles N>"
                    .into())
            }
            other if !other.starts_with('-') && cmd.is_empty() => cmd = other.to_string(),
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    if cmd.is_empty() {
        return Err("a subcommand is required (scan, read, move, demo)".into());
    }
    Ok(Args {
        port,
        baud,
        ids: parse_ids(&ids_spec),
        cmd,
        position,
        speed,
        amplitude,
        period,
        cycles,
    })
}

fn autodetect_port() -> Result<String, String> {
    for pattern in ["/dev/ttyUSB", "/dev/ttyACM"] {
        let mut found: Vec<_> = std::fs::read_dir("/dev")
            .map_err(|e| e.to_string())?
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.starts_with(pattern))
            .collect();
        found.sort();
        if let Some(first) = found.into_iter().next() {
            return Ok(format!("/dev/{first}"));
        }
    }
    Err("no serial port found; pass --port".into())
}

fn main() {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            exit(2);
        }
    };
    let port = if args.port.is_empty() {
        match autodetect_port() {
            Ok(p) => p,
            Err(e) => {
                eprintln!("{e}");
                exit(2);
            }
        }
    } else {
        args.port
    };
    if args.ids.is_empty() {
        eprintln!("no valid ids");
        exit(2);
    }
    println!("port={port} baud={} ids={:?}", args.baud, args.ids);

    let mut bus = match ScsBus::open(&port, args.baud, Duration::from_millis(60)) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{e}");
            exit(1);
        }
    };

    match args.cmd.as_str() {
        "scan" => {
            let mut found = Vec::new();
            for &sid in &args.ids {
                if bus.ping(sid) {
                    let ver = bus.read_u16(sid, REG_VERSION);
                    match ver {
                        Some(v) => println!("  id {sid:>3}  OK  version=0x{v:04x}"),
                        None => println!("  id {sid:>3}  OK  version=?"),
                    }
                    found.push(sid);
                }
            }
            if found.is_empty() {
                println!("  nothing answered. check power, jumper=B, and baud rate");
            } else {
                println!("found {}: {found:?}", found.len());
            }
        }
        "read" => {
            println!("{:>4} {:>6} {:>7} {:>5} {:>6}", "id", "pos", "volt", "temp", "load");
            for &sid in &args.ids {
                let Some(pos) = bus.read_u16(sid, REG_PRESENT_POSITION) else {
                    println!("{sid:>4}     --");
                    continue;
                };
                let volt = bus.read_u8(sid, REG_PRESENT_VOLTAGE);
                let temp = bus.read_u8(sid, REG_PRESENT_TEMPERATURE);
                let load = bus.read_u16(sid, REG_PRESENT_LOAD);
                let v = volt.map(|x| format!("{:.1}V", x as f64 / 10.0)).unwrap_or("?".into());
                let t = temp.map(|x| format!("{x}C")).unwrap_or("?".into());
                let l = load.map(|x| x.to_string()).unwrap_or("?".into());
                println!("{sid:>4} {pos:>6} {v:>7} {t:>5} {l:>6}");
            }
        }
        "move" => {
            let position = args.position.unwrap_or_else(|| {
                eprintln!("move needs --position");
                exit(2);
            });
            let entries: Vec<_> = args
                .ids
                .iter()
                .map(|&sid| (sid, encode_u16(position.clamp(POS_MIN, POS_MAX) as u16)))
                .collect();
            bus.sync_write(REG_GOAL_POSITION, 2, &entries);
            if args.speed != 0 {
                let entries: Vec<_> = args
                    .ids
                    .iter()
                    .map(|&sid| (sid, encode_u16(args.speed)))
                    .collect();
                bus.sync_write(REG_GOAL_SPEED, 2, &entries);
            }
            println!("moved {} servos to {position}", args.ids.len());
        }
        "demo" => {
            for &sid in &args.ids {
                bus.torque(sid, true);
            }
            std::thread::sleep(Duration::from_millis(200));
            println!(
                "demo: {} cycles, center {POS_CENTER} +/- {}",
                args.cycles, args.amplitude
            );
            for _ in 0..args.cycles {
                for pos in [
                    POS_CENTER + args.amplitude,
                    POS_CENTER,
                    POS_CENTER - args.amplitude,
                    POS_CENTER,
                ] {
                    let p = pos.clamp(POS_MIN, POS_MAX) as u16;
                    let entries: Vec<_> =
                        args.ids.iter().map(|&sid| (sid, encode_u16(p))).collect();
                    bus.sync_write(REG_GOAL_POSITION, 2, &entries);
                    std::thread::sleep(Duration::from_secs_f64(args.period / 4.0));
                }
            }
            for &sid in &args.ids {
                bus.torque(sid, false);
            }
            println!("torque released");
        }
        other => {
            eprintln!("unknown subcommand: {other}");
            exit(2);
        }
    }
}
