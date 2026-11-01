# Microduck RL 自定义动作训练指南

面向这台 13 自由度机器人（RK3588 + 13×SCS225 + MPU6050），说明如何在
`microduck_rl` 里**新训一个动作**、从环境搭建到导出的完整流程。硬件接口与关节分配见
[`rk3588-scs225-13dof.md`](rk3588-scs225-13dof.md)。

> 已经有 13-DoF 前进/后退任务：`Mjlab-Velocity-Flat-MicroDuck13`。
> 它也是"如何加一个新动作"的模板。

---

## 0. 一分钟速查

```bash
cd microduck_rl

# 冒烟（必做，约 1 分钟）：64 环境 × 5 迭代
uv run train <TASK_ID> --env.scene.num-envs 64 --agent.max_iterations 5 --agent.logger tensorboard

# 短训（本机实测 ~2.2 s/迭代 @1024 环境）：1024 × 1000 ≈ 40 分钟
uv run train <TASK_ID> --env.scene.num-envs 1024 --agent.max_iterations 1000 --agent.logger tensorboard

# 正式：4096 × 4000 ≈ 3.5–4 小时
uv run train <TASK_ID> --env.scene.num-envs 4096 --agent.max_iterations 4000 --agent.logger tensorboard

# 看效果 / 录视频
uv run play <TASK_ID> --checkpoint-file logs/rsl_rl/<experiment>/<run>/model_1000.pt

# 续训（注意预算公式：显示的 max_iterations = 已训迭代 + 本次给的）
uv run train <TASK_ID> --env.scene.num-envs 1024 \
  --agent.load-checkpoint model_1000.pt --agent.resume True --agent.max_iterations 1000

# 日志与曲线
tail -f /tmp/train_xxx.log
uv run --with tensorboard tensorboard --logdir logs/rsl_rl --port 6006
```

---

## 1. 代码结构：改哪个文件

| 你想改的东西 | 文件 |
|---|---|
| 机器人模型（连杆、质量、关节、执行器） | `src/mjlab_microduck/robot/microduck/*.xml` |
| 机器人常量（HOME 姿态、执行器参数、robot cfg） | `src/mjlab_microduck/robot/microduck_constants.py` |
| 每个动作的**环境**（命令、观测、奖励、课程） | `src/mjlab_microduck/tasks/microduck_<动作>_env_cfg.py` |
| 自定义奖励/事件/观测函数 | `src/mjlab_microduck/tasks/mdp.py` |
| 任务注册（`TASK_ID`） | `src/mjlab_microduck/tasks/__init__.py` |
| 镜像损失表（左右对称） | `src/mjlab_microduck/tasks/symmetry.py` |

13-DoF 相关：
- 模型：`robot_walk_13dof.xml`（去掉 head_yaw 执行器，并用
  `<equality><joint joint1="head_yaw" polycoef="0 1 0 0 0"/></equality>` 锁死）
- robot cfg：`MICRODUCK_WALK_13DOF_ROBOT_CFG`
- 任务：`Mjlab-Velocity-Flat-MicroDuck13`（`microduck_velocity13_env_cfg.py`）

---

## 2. 新加一个动作：最小步骤

以"只训前进/后退"为模板：

1. **复制模板**
   `cp .../tasks/microduck_velocity13_env_cfg.py .../tasks/microduck_<名字>_env_cfg.py`

2. **改工厂函数**（覆盖命令/奖励/课程，尽量不动共享的 velocity 工厂）：

   ```python
   def make_microduck_<名字>_env_cfg(play=False, rough=False):
       cfg = make_microduck_velocity_env_cfg(play=play, rough=rough)
       cfg.scene.entities = {"robot": MICRODUCK_WALK_13DOF_ROBOT_CFG}
       twist = cfg.commands["twist"]
       twist.ranges.lin_vel_x = (-0.3, 0.5)   # 前进正、后退负
       twist.ranges.lin_vel_y = (0.0, 0.0)
       twist.ranges.ang_vel_z = (0.0, 0.0)
       twist.rel_standing_envs = 0.0
       twist.rel_turn_in_place_envs = 0.0
       return cfg
   ```

3. **注册** `src/mjlab_microduck/tasks/__init__.py`：

   ```python
   from .microduck_<名字>_env_cfg import make_microduck_<名字>_env_cfg

   register_mjlab_task(
       task_id="Mjlab-<名字>-MicroDuck13",
       env_cfg=make_microduck_<名字>_env_cfg(),
       play_env_cfg=make_microduck_<名字>_env_cfg(play=True),
       rl_cfg=MicroduckRlCfg,
       runner_cls=MicroduckOnPolicyRunner,
   )
   ```

4. **冒烟**：`uv run train Mjlab-<名字>-MicroDuck13 --env.scene.num-envs 64 --agent.max_iterations 5 --agent.logger tensorboard`
   确认三点：无报错退出、`nan_state: 0`、有 `Mean reward` 和各项指标。

5. **正式训练** → `play` 看 → 导出/发布。

> 不要直接改共享的 `make_microduck_velocity_env_cfg` 来加新动作：量产 14-DoF 任务
> 也用它。覆盖（override）比修改安全。

---

## 3. 常见动作怎么改

| 动作 | 改法 |
|---|---|
| **前进/后退** | `twist.ranges.lin_vel_x=(负, 正)`，其余速度=0（模板已实现） |
| 原地转向 | `twist.ranges.ang_vel_z=(-1.0, 1.0)`，线速度=0 |
| 侧移 | `twist.ranges.lin_vel_y=(-0.3, 0.3)` |
| 站立保持 | 全部速度范围 `(0,0)`，`rel_standing_envs=1.0`，可把 `head_pose` 也置 0 |
| 抬头/低头 | `cfg.commands["head_pose"].ranges` 的第 1/2 槽给范围（head_yaw 槽保持 0） |
| 坐-站 | 用 `Mjlab-SitStand-*` 任务的命令（posture flag） |
| 踢球 | 用 `Mjlab-BallKick-*` 任务 |
| 前滚翻 | 用 `Mjlab-Roulade-*` 任务 |

**奖励函数去哪儿改**：`cfg.rewards["<name>"]` 查权重；实现放在 `mdp.py`。
两个铁律（见 `microduck_rl/AGENTS.md`）：
- 每个 `Episode_Reward/<惩罚项>` 必须 ≤ 0（为正说明奖励漏洞，策略会刷它）；
- 别在"坏状态"上给正奖励（会趴着刷分），用势函数塑形。

---

## 4. 参数与预算（本机 RTX 2070）

| 参数 | 含义 | 建议 |
|---|---|---|
| `--env.scene.num-envs` | 并行环境数 | 64 冒烟 / 1024 短训 / 4096 正式 |
| `--agent.max_iterations` | 本次迭代预算 | 简单技巧 ~1000；步态 4000–6000 |
| `--agent.logger` | 日志后端 | `tensorboard`（零配置）或 `wandb`（需登录） |
| `--agent.load-checkpoint` + `--agent.resume True` | 续训 | 环境数必须与原来一致 |

实测：1024 环境约 **2.2 s/迭代**（RTX 2070），即 1000 迭代 ≈ 40 分钟。
OOM 就降到 2048/1024。

**续训预算公式**：`命令行 max_iterations = 想达到的总数 − 已加载的迭代数`。

---

## 5. 训练完 → 真机

1. **导出 ONNX**：训练正常结束会自动导出到
   `logs/rsl_rl/<experiment>/<run>/<run>.onnx`；手动导出用
   `uv run scripts/export.py`。**观测归一化参数已烘焙进图内**，这是唯一可部署格式。
2. **策略维度**：BAM 执行器按关节名匹配，所以策略仍是 **61D 观测 / 14 动作**
   （13 个真实关节 + 被锁死的 `head_yaw`），与 `robotd` 的契约一致，无需改维度。
   真机上 `head_yaw` 动作被 `SCS_JOINT_SLOTS` 跳过。
3. **部署**：把 ONNX 传给 `robotd`（`robotctl policy load` / `publish` 到 HF）。
4. **调试顺序建议**：先 `uv run play` 在桌面看步态 → 机器人吊起来跑一次 →
   落地。第一次上电务必有人扶/急停。

---

## 6. 常见坑

| 现象 | 原因/处理 |
|---|---|
| `No API key configured`（wandb） | 加 `--agent.logger tensorboard`，或先 `wandb login` |
| CUDA OOM | 降 `num-envs` |
| `XML Error: ... unrecognized attribute` | MuJoCo 3.10 的 equality 用 `joint1`（不是 `joint`） |
| 改完 MJCF 没生效 | 有 `__pycache__`/缓存时删掉重跑；确认 `spec_fn` 指向新 XML |
| 惩罚项变正 | 奖励漏洞，策略在刷分；改奖励或加门控 |
| 训练数据正常但视频很怪 | 看视频和检查接触几何/轴，别只看指标 |
| 续训后总迭代数不对 | 见 §4 的预算公式 |

---

## 7. 这台机器当前进度

- 13-DoF 模型与 `Mjlab-Velocity-Flat-MicroDuck13` 已加；先训前进/后退。
- 数据/导出：`logs/rsl_rl/velocity13/...`（若 `experiment_name` 未单独设置，沿用 velocity）。
- 下一步：短训验收 → 桌面/吊装验证 → 装机下地；再训练转向、抬头等动作。
