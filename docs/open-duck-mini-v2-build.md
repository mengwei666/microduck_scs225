# 打印与组装清单（Open Duck Mini v2，13×SCS225）

这台机器人以 **Open Duck Mini v2** 为骨架：它是 Apache-2.0 开源、专为 **Feetech
STS3215** 设计的 42cm 双足。**SCS225 与 STS3215 同壳同尺寸**
（45.2×24.7×35mm、25T 输出齿），所以它的结构件可以直接用。

- 上游仓库：https://github.com/apirrone/Open_Duck_Mini （分支 **v2**）
- 本机已克隆：`/tmp/opencode/odm`
- 舵机 ID 分配与本项目 `microduck` 完全一致（左 20–24、头颈 30–33、右 10–14）

---

## 1. 打印设置（上游 print_guide）

| 材料 | 设置 | 用于 |
|---|---|---|
| PLA | 15% 填充 | 其余全部结构件 |
| TPU | 40% 填充 | `foot_bottom_tpu.stl`（脚底防滑垫） |

拓竹机器直接按此出片；结构件建议 0.2mm 层高、3 层墙。

## 2. 打印清单（逐件）

**腿部（×2 组）**
- `foot_top.stl` ×2、`foot_side.stl` ×2
- `foot_bottom_pla.stl` ×2、`foot_bottom_tpu.stl` ×2（TPU）
- `knee_to_ankle_left_sheet.stl` ×4、`knee_to_ankle_right_sheet.stl` ×4
- `leg_spacer.stl` ×4
- `left_roll_to_pitch.stl` ×1、`right_roll_to_pitch.stl` ×1
- `roll_motor_bottom.stl` ×2、`roll_motor_top.stl` ×2

**躯干与外壳**
- `trunk_bottom.stl` ×1、`trunk_top.stl` ×1
- `body_front.stl` ×1、`body_middle_bottom.stl` ×1、`body_middle_top.stl` ×1、`body_back.stl` ×1
- `battery_pack_lid.stl` ×1

**头颈（13 自由度要动这里，见 §3）**
- `neck_left_sheet.stl` ×1、`neck_right_sheet.stl` ×1
- `head_pitch_to_yaw.stl` ×1
- `head_yaw_to_roll.stl` ×1        ← 与 head_yaw 舵机相连
- `head_roll_mount.stl` ×1
- `head.stl` ×1、`head_bot_sheet.stl` ×1
- `left_eye.stl`/`right_eye.stl` ×1 各
- `left_antenna_holder.stl`/`right_antenna_holder.stl` ×1 各
- `left_cache.stl`/`right_cache.stl` ×1 各

**装饰（可不打）**
- `bulb.stl`、`flash_light_module.stl`、`flash_reflector_interface.stl`、
  `speaker_interface.stl`、`speaker_stand.stl`

## 3. 13 自由度（去掉 head_yaw）怎么处理

上游头部是**串成一条链**的，head_yaw 舵机本身是结构件：

```
neck_pitch(30) → head_pitch(31) → [head_pitch_to_yaw] → head_yaw(32) → [head_yaw_to_roll] → head_roll(33) → 头壳
```

去掉 head_yaw 舵机后，`head_pitch_to_yaw` 和 `head_yaw_to_roll` 之间没有连接件。三个方案：

1. **推荐（最省事）**：再买 1 个 SCS225 当第 14 个舵机，头部按原设计装全。我们的
   13-DoF 策略仍是 14 动作（head_yaw 被锁死/跳过），真机上多出的这一个舵机可以
   不使能（自由/阻尼），或以后单独训练头转向。结构最省心。
2. **打印"假舵机"适配块**：我用 OpenSCAD 生成一个替代 head_yaw 舵机的刚性件
   （外形/安装孔按 SCS225，把两个打印件固连，等效把 yaw 锁在 0）。需要先量出
   `head_pitch_to_yaw` 与 `head_yaw_to_roll` 的对接面尺寸和孔位。
3. 直接不做 yaw：把头壳刚性固定在 neck 上，需要额外设计一个转接件。

> 在决定前，先 **打印单个头部件 + 拿一个 SCS225 试装**，确认舵机输出齿的 25T
> 花键和随附舵盘与上游打印件孔位能对上（STS3215 与 SCS225 同规格，但舵盘孔位
> 批次可能有差异）。整个机身没问题了再批量打印。

## 4. 舵机 ID 与接线

上游 ID（与本项目一致）：

```
left_hip_yaw 20, left_hip_roll 21, left_hip_pitch 22, left_knee 23, left_ankle 24
neck_pitch 30, head_pitch 31, head_yaw 32(不用), head_roll 33
right_hip_yaw 10, right_hip_roll 11, right_hip_pitch 12, right_knee 13, right_ankle 14
```

- 13 自由度实际使用：20,21,22,23,24, 30,31,33, 10,11,12,13,14。
- 本项目运行时的映射常量是 `robotd::SCS_JOINT_SLOTS`；如果按上面这套 ID 接，需要把
  `open_scs` 里的 `ids` 从 `1..=13` 改成这套顺序（现为临时 bench 映射）。
- 改 ID/标定：板子上的 `/root/scs225` 工具目前只有 scan/read/move/demo，没有改 ID
  和设限位的子命令；需要的话我加上 `set-id` / `set-limit`。
- 总线仍是 Waveshare Bus Servo Adapter (A)，跳线 B，1 Mbps，7.4V。

## 5. 其它硬件

- **M3 热熔铜螺母**（上游大量使用）、M3 螺丝
- 轴承（头部、腿部，按上游 BOM）
- 2S 电池（7.4V，18650 双节座）——与本文档的舵机供电一致；记得 ≥5A 放电
- 主控：**RK3588 暂不上机**。整机先用舵机转接板 + 桌面供电，从 RK3588 串口/USB
  连线跑 `robotd --bus scs` 做台架调试；装机时再决定主控和电池仓布局。

## 6. 与本项目软件的对应

| 项 | 状态 |
|---|---|
| SCS225 驱动（`duck-control/src/scs.rs`） | 完成，板子上已验证 |
| MPU6050 驱动（I2C2） | 完成 |
| 13-DoF 训练任务 `Mjlab-Velocity-Flat-MicroDuck13` | 完成（vx 前进/后退，短训中） |
| 仿真模型比例 | ⚠️ 现在用的是 **microduck 比例**，与 Open Duck Mini 实物不一致；装机前要把 MJCF 换成 Open Duck Mini 的连杆尺寸/质量，再重训 |
| 主控安装 | 暂不做（本文件） |

## 7. 下一步

1. 先试装一个 SCS225 到打印件（验证舵盘/孔位）。
2. 决定 head_yaw 方案（§3 的三选一）。
3. 我把训练模型移植成 Open Duck Mini 几何（从上游 MJCF/URDF 提取连杆与质量），
   重新训练前进/后退。
4. 装机后量实际质量/质心，微调 MJCF。
