# RK3588 动作测试命令（SCS225 总线 + 13-DoF 策略）

在正点原子 ATK-DLRK3588 上运行 `robotd`，用命令行做动作测试。硬件与关节分配见
[`rk3588-scs225-13dof.md`](rk3588-scs225-13dof.md)，训练见
[`training-custom-actions.md`](training-custom-actions.md)。

---

## 0. 板子上已就绪的东西

| 项 | 路径 / 值 |
|---|---|
| ONNX Runtime 1.28.0（aarch64） | `/usr/local/lib/libonnxruntime.so` |
| 策略文件 | `/root/policies/microduck13_vx.onnx`（obs[1,61] → actions[1,14]） |
| robotd | `/root/robotd`（含 `--bus scs` + 13-DoF 映射） |
| 参数文件 | `/etc/robot/robotd.toml`（`[policy] enabled = true`，各槽指向该策略） |
| 命令行客户端 | `/root/robotcli.py` |
| 舵机自检工具 | `/root/scs225`（scan/read/move/demo，**需 robotd 停止**才能开串口） |
| 舵机总线 | Waveshare Bus Servo Adapter (A)，跳线 **B**，USB → `/dev/ttyACM0`，1 Mbps |
| 供电 | 舵机 **7.4V ≥5A 稳压电源**（调试期别用小电池） |
| IMU | MPU6050，I2C2 `P1_A0`=SCL / `P1_A1`=SDA，地址 0x68 |

---

## 从零开始：操作步骤

### 0. 连接板子（二选一）

```sh
# 串口控制台（笔记本上，泰山派调试口为 1500000 8N1；正点原子同）
picocom -b 1500000 /dev/ttyACM0
# 或 ssh（板子联网时）
ssh root@192.168.1.x
```

看到 `root@ATK-DLRK3588:/#` 即可。**下面的命令都在板子上执行。**

### 1. 上电顺序

1. 板子 12V 上电，等它启动完（约 30 秒）；
2. **舵机 7.4V（≥5A）供电**，转接板跳线在 **B**；
3. 转接板 USB 插到板子 —— 板子上出现 `/dev/ttyACM0`。

验证（此时 robotd 还没起，串口空闲）：

```sh
/root/scs225 --port /dev/ttyACM0 --ids 1-13 scan     # 期望 13 个都 OK
```

### 2. 启动 robotd（带策略）

```sh
cd /root
ORT_DYLIB_PATH=/usr/local/lib/libonnxruntime.so RUST_LOG=info \
  ./robotd --bus scs --port /dev/ttyACM0 --socket /run/robotd.sock \
  > /tmp/robotd.log 2>&1 &
```

### 3. 确认它活着

```sh
/root/robotctl --robot-socket /run/robotd.sock health      # 应看到 bus ok / policy 已加载
/root/robotctl --robot-socket /run/robotd.sock policy list
```

### 4. 发动作命令

```sh
R="/root/robotctl --robot-socket /run/robotd.sock"
C="/root/robotcli.py"

$R robot init        # 使能力矩，2 秒回到 home 姿态
$C enable on         # 打开策略（开始能走）
$C move 0.2 0 0      # 前进（vx=0.2, vy=0, vyaw=0）
$C move -0.2 0 0     # 后退
$C stop              # 速度归零
$C enable off        # 关策略并回 home
$R robot relax --yes # 卸力
```

### 5. 收工

`$C stop` → `$C enable off` → `$R robot relax --yes`，然后关舵机电源。

---


### 台架实测（2026-10-21，泰山派 RK3566 / Ubuntu 20.04）

- 部署成功：`robotd`、`robotctl`、`scs225`、ONNX Runtime 1.28、策略文件都在；
  `robotd` 的 md5 为 `3b4b796e...`（带"缺失舵机跳过"逻辑）。
- **实际舵机 ID 不是 1–13**，而是 microduck 布局：
  - 右腿 `10–14`、左腿 `20–24`（各 5 个）
  - 头部 `30`(neck_pitch) / `31`(head_pitch) / `33`(head_roll)；`32`(head_yaw) 不用
- 本次只接了 10 个（头部 3 个未接）；`robotd` 会打印
  `servo missing; that joint will read zero` 并照常加载策略。
- **低电量会让整块板关机**：电池 6.92V(20%) 驱动时塌压 → 总线连续读失败 →
  `robotd` 的 `battery_empty_shutdown` 触发 `poweroff`。台架务必用
  **7.4V ≥5A 稳压电源**或充满的高倍率电池；若只想台架调试、不想它自动关机，
  可在 `/etc/robot/robotd.toml` 加：
  ```toml
  [safety]
  battery_empty_shutdown = false
  ```
  （仅限供电可靠时使用。）

## 1. 启动 robotd（带策略）

```sh
cd /root
ORT_DYLIB_PATH=/usr/local/lib/libonnxruntime.so RUST_LOG=info \
  ./robotd --bus scs --port /dev/ttyACM0 --socket /run/robotd.sock \
  > /tmp/robotd.log 2>&1 &
```

确认（约 10 秒后）：

```sh
/root/robotctl --robot-socket /run/robotd.sock health
/root/robotctl --robot-socket /run/robotd.sock policy list
```

期望：

```
robot     healthy
  loop      50.2 of 50.0 Hz · ...
  bus       ok
  imu       ready
```

```
mode: walk
 * walk   local   /root/policies/microduck13_vx.onnx
 ...
```

没有策略加载就查日志：`grep -iE 'policy|onnx' /tmp/robotd.log`。

---

## 2. 动作命令

### 关节上电 / 回中位 / 放松（不需要策略）

```sh
R="/root/robotctl --robot-socket /run/robotd.sock"
$R robot init      # 力矩开，约 2 秒把关节插值到 home 姿态
$R robot relax     # 关力矩（机器人会瘫软）
```

`init` 期望输出：`standing up — about two seconds to the home pose`，
日志里随后是 `at the home pose; the policy has the robot`。

### 行走（策略驱动）

```sh
C="/root/robotcli.py"
$C enable on            # 打开策略执行（失败/成功都有 JSON 回复）
$C move 0.2 0 0         # 前进 0.2 m/s（连续指令，通知，无回复）
$C move -0.2 0 0        # 后退 0.2 m/s
$C move 0 0 0.4         # 原地左转 0.4 rad/s（本 13-DoF 策略未训转向，慎用）
$C stop                 # 速度归零（不关策略）
$C enable off           # 关策略并回到 home 姿态
```

`enable on` 期望：`{"result": {"accepted": true, "reason": "enabled — driving"}}`

### 一次性技能与其它

```sh
C="/root/robotcli.py"
$C do kick_left         # 运行 kick_left 槽的策略（本机各槽都指向同一个文件）
$C mode                 # 当前 drive mode
```

> 本机把 `[policy]` 的所有槽都指向同一个 `microduck13_vx.onnx`（台架临时配置）。
> 因此 `do ...` 跑出来的还是前进/后退网络，不是真正的踢腿/翻滚。要真技能需要分别训练。

---

## 3. 一次完整的动作测试（照抄即可）

```sh
# 1) 启动
cd /root && ORT_DYLIB_PATH=/usr/local/lib/libonnxruntime.so RUST_LOG=info \
  ./robotd --bus scs --port /dev/ttyACM0 --socket /run/robotd.sock > /tmp/robotd.log 2>&1 &
sleep 12

# 2) 自检
/root/robotctl --robot-socket /run/robotd.sock health
/root/robotctl --robot-socket /run/robotd.sock policy list

# 3) 上电回中位
/root/robotctl --robot-socket /run/robotd.sock robot init
sleep 3

# 4) 开策略，前进 5 秒
/root/robotcli.py enable on
/root/robotcli.py move 0.2 0 0
sleep 5

# 5) 后退 5 秒，然后停
/root/robotcli.py move -0.2 0 0
sleep 5
/root/robotcli.py stop

# 6) 收工：关策略回中位，再放松
/root/robotcli.py enable off
sleep 3
/root/robotctl --robot-socket /run/robotd.sock robot relax
```

### 实测结果（2026-09-18，台架散件）

| 步骤 | 结果 |
|---|---|
| `robotd` 启动（不驱动） | ✅ `loop 50.0–50.2/50 Hz`，bus ok，imu ready |
| `policy list` | ✅ 7 个槽都指向 `/root/policies/microduck13_vx.onnx` |
| `robot init` | ✅ 力矩开，2 秒回 home |
| `enable on` | ✅ `accepted: true, enabled — driving` |
| `move ±0.2` | ✅ 指令被接受 |
| **驱动后的 Bus** | ❌ `bus read failed` 连续增长，loop 掉到 3.3 Hz |
| `relax --yes`（卸力） | ✅ 总线立刻恢复 `bus ok`、loop 回到 ~50 Hz |

**原因确认：是供电，不是软件。** 换用能扛住 13 舵机连续运动的电池后（高倍率 2S），
再跑同一流程：

| 阶段 | 结果 |
|---|---|
| 空闲 / 卸力 | `loop 50.0–50.2 Hz`，bus ok |
| **驱动（前进/后退，action_scale 恢复 0.9 / gain 200）** | ✅ **bus 全程 ok，无连续读失败** |
| 驱动中的环率 | ⚠️ **27–31 Hz**（低于 45 判据；功能正常，动作照做） |
| `stop` / `enable off` / `relax` | ✅，回到 50 Hz |

### 为什么驱动时只有 ~28 Hz

不是掉线，是**运动中的读更慢**：SCS225 不支持 SYNC READ，位置和速度必须一问一答；
舵机在驱动电机时应答变慢（单事务 ~0.7ms → ~1.3ms），一个 tick 的 19–20 个事务
就从 ~14ms 涨到 ~26ms，超出 20ms 预算。空闲时舵机不忙，所以能跑满 50Hz。

可选的处理，按代价排序：

1. **台架就这样**：27–31Hz 功能正常，动作能执行（当前状态）。
2. **按 30Hz 重训**：在 `microduck_rl` 里把控制频率设成 30Hz，硬件就能轻松达标，
   这是最干净的做法（需要重训一次）。
3. 继续稀疏速度读（现在每 tick 读一半；再稀到 1/3、1/4），代价是速度反馈更旧。

### 其它

- 舵机是散的会让电流更大、动作互相碰撞，**固定到治具**后表现更接近真机。
- 板子 **wlan0 会掉线**（USB WiFi），掉线后重新拉起：
  `ip link set wlan0 up && udhcpc -i wlan0`（只影响传文件，不影响串口调试）。
- 小容量电池（1000mAh）驱动 13 舵机掉电很快（实测 ~1 分钟从 7.5V 掉到 7.2V/39%），
  长时间调试建议用稳压电源或更大的电池。

---

## 4. 状态与日志

```sh
R="/root/robotctl --robot-socket /run/robotd.sock"
$R health                                  # 环率/总线/IMU/电池/温度
$R monitor                                 # 全屏 TUI（姿态、重力、命令）
tail -f /tmp/robotd.log                    # 运行日志
grep -c 'bus read failed' /tmp/robotd.log  # 读失败计数
```

看环率是否达标：`health` 里的 `loop … of 50.0 Hz`。低于 45 Hz 会被判 unhealthy
（更新门限），但那只是判据，不影响手动命令。

---

## 5. 舵机总线自检（robotd 必须先停）

```sh
kill $(pidof robotd); sleep 1
/root/scs225 --port /dev/ttyACM0 --ids 1-13 scan    # 13 个都应在
/root/scs225 --port /dev/ttyACM0 --ids 1-13 read    # 位置/电压/温度
/root/scs225 --port /dev/ttyACM0 --ids 1-13 demo --amplitude 150 --cycles 2
```

---

## 6. 故障排查

| 现象 | 原因 / 处理 |
|---|---|
| **所有舵机无应答** | 舵机供电掉了 / 电池塌压。**断舵机电源 5 秒再上电**；换 7.4V ≥5A 稳压电源；必要时把 USB 转接板也重插 |
| 只有部分舵机无应答 | 该舵机线松/坏，或它把总线拉死；单独接它测 |
| `scan 1-3` 通、`scan 1-13` 不通 | 供电撑不住 13 个；先换电源 |
| 驱动后 `bus read failed` 连续增长 | 同上，供电问题（不是软件）。软件侧已有重试，但 retry 救不了 brownout |
| 环率 < 45 Hz | 检查是否未用最新 robotd（位置全读+速度半读）；确认没有别的进程占串口 |
| `open: Unable to acquire exclusive lock` | robotd 正占着串口，先 `kill $(pidof robotd)` |
| `policy` 加载失败 | `ORT_DYLIB_PATH` 没设，或 `.onnx` 不是 obs[1,61]→actions[1,14]，看 `/tmp/robotd.log` |
| `enable on` 被拒 | `health` 看原因；策略没加载时不会驱动 |
| 舵机发烫/吱吱叫 | 立刻 `robot relax` 或断电。散件被卡住时会堵转 |
| 板子突然关机 | 电池空了触发了 `battery_empty_shutdown`；换稳压电源/充电，或临时设为 false |

## 7. 设计说明（为什么读要拆成一半）

SCS225 单次 READ **最多可靠读 3 字节**（≥4 字节必失败），且**不支持 SYNC READ（0x82）**，
所以每个量都是独立事务。实测：13 个位置 = 8.2 ms，位置+速度交替 = 24 ms，分组全读 =
19.5 ms。50Hz 的预算是 20 ms，所以驱动里做成：

- **每个 tick 读 13 个位置**；
- **速度每 tick 交替读一半**（每个关节 40 ms 刷新一次，全分辨率）。

这样空闲环率恢复到 **50.2 Hz**。改的是
`duck-control/src/scs.rs` 的 `ScsIo::read`，改回"每 tick 读全速度"会让环率掉到 ~30Hz。
