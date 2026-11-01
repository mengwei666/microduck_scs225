# Microduck RL — 单人命令行运行指南

> 本文档面向"一个人、一台机器、纯命令行"跑完 **训练 → 监控 → 看效果 → 导出** 全流程。
> 所有命令在本仓库(`microduck_rl`)实际验证过。
> 本机环境参考:RTX 2070 8GB / 30GB 内存 / Ubuntu Linux(Wayland)。

---

## 0. 一分钟速查

```bash
# 1) 冒烟测试(正式训练前必做,约 1 分钟)
uv run train Mjlab-Velocity-Flat-MicroDuck --env.scene.num-envs 64 --agent.max_iterations 5

# 2) 正式训练(主任务:4096 环境 × 4000 迭代,RTX 2070 约 3.5~4 小时)
uv run train Mjlab-Velocity-Flat-MicroDuck \
  --env.scene.num-envs 4096 --agent.max_iterations 4000 \
  --agent.logger tensorboard

# 3) 中断后续训(从最新 checkpoint 继续,预算换算见第 4 节!)
uv run train Mjlab-Velocity-Flat-MicroDuck \
  --env.scene.num-envs 4096 --agent.max_iterations 2000 \
  --agent.load-checkpoint model_2000.pt --agent.resume True \
  --agent.logger tensorboard

# 4) 3D 看效果
uv run play Mjlab-Velocity-Flat-MicroDuck \
  --checkpoint-file logs/rsl_rl/velocity/<run目录>/model_2000.pt

# 5) 实时看日志
tail -f /tmp/你的训练日志.log      # 或 tmux attach
```

---

## 1. 环境准备

### 1.1 硬件要求

- **必须**:NVIDIA 显卡(≥8GB 显存),训练跑在 MuJoCo Warp(CUDA)上
- 内存 ≥16GB(4096 环境约 3~5GB 显存 + 若干 GB 内存)
- 磁盘:项目 + 虚拟环境约 8~10GB

### 1.2 首次安装

```bash
# 安装 uv(包管理器)
curl -LsSf https://astral.sh/uv/install.sh | sh
# 或: apt install uv / 官网下载

# 进入仓库并同步依赖(首次约 2GB,CUDA 相关;耐心等)
cd microduck_rl
uv sync

# ARM 机器(DGX Spark / Jetson)首次同步容易超时:
export UV_HTTP_TIMEOUT=600
uv sync
```

### 1.3 GPU 健康自检(克隆仓库后先跑一次)

```bash
# 显卡状态
nvidia-smi

# CUDA 是否能真正分配显存(比 nvidia-smi 更严格)
cd microduck_rl
uv run python -c "import torch; x=torch.zeros(2, device='cuda'); torch.cuda.synchronize(); print('GPU OK:', x.device)"
```

**⚠️ 如果 nvidia-smi 正常但上面 torch 报 `CUDA error: CUDA-capable device(s) is/are busy or unavailable`**
→ 内核里查得到 `NVRM: Failed to create a DMA mapping` 的话,说明 **NVIDIA 驱动处于损坏状态**。
解决办法只有系统级:`sudo reboot`(或 `sudo nvidia-smi -r` 热重置)。重启后重跑自检。
(本机实测:驱动坏了之后,重启才恢复;这是 2026-09-05 遇到过的真实故障。)

---

## 2. 训练全流程(五步)

### 第 1 步:看有哪些任务

```bash
uv run list-envs
```

现有任务族(每个都有 Flat/Rough 地形与 `-Backlash-` 变体):

| 任务 ID | 内容 |
|---|---|
| `Mjlab-Velocity-{Flat,Rough}-MicroDuck` | **主任务**:按 twist(速度)+ head_pose 指令行走 |
| `Mjlab-VelStand-*` | 行走 + 跌倒恢复 |
| `Mjlab-StandUp-*` / `Mjlab-SitStand-*` | 站起 / 坐-站 |
| `Mjlab-GroundPick-*` / `Mjlab-BallKick-*` | 嘴尖触地 / 踢球 |
| `Mjlab-Roulade-*` | 前滚翻 |
| `Mjlab-*-Rollers` / `Swizzle` / `RollerCrouch` / `RollerSlope` / `RollerStandUp` / `Spin` | 滚轮滑冰系列 |
| `Mjlab-Velocity-Flat-Backlash-MicroDuck` 等 | 每个主任务的齿轮间隙(backlash)变体 |

### 第 2 步:冒烟测试(铁律,每次正式训练前必跑)

```bash
# 64 环境 × 5 迭代,1~2 分钟,能暴露 ~95% 的配置错误
uv run train Mjlab-Velocity-Flat-MicroDuck --env.scene.num-envs 64 --agent.max_iterations 5
```

**确认三点**才可继续:
1. 没有报错退出(exit 0)
2. 日志里 `nan_state: 0.0000`(无 NaN)
3. 出现 `Mean reward`、各项 `Episode_Reward` 等指标

### 第 3 步:正式训练

```bash
# 推荐配置(仓库官方预算):4096 环境 × 4000 迭代
cd microduck_rl
uv run train Mjlab-Velocity-Flat-MicroDuck \
  --env.scene.num-envs 4096 \
  --agent.max_iterations 4000 \
  --agent.logger tensorboard \
  > /tmp/train_velocity.log 2>&1 &
```

> **强烈建议用 tmux 跑长训练**(关终端/掉线都不影响,见 4.5)。

### 第 4 步:监控(见第 5 节)

### 第 5 步:看效果 / 导出(见第 6、7 节)

---

## 3. 关键参数详解

### 3.1 训练参数(`uv run train <任务> [参数]`)

| 参数 | 值域 | 说明 |
|---|---|---|
| `--env.scene.num-envs` | 64~8192 | 并行环境数。**64 = 冒烟,4096 = 标准训练**。越大每迭代越慢但样本越多 |
| `--agent.max_iterations` | 整数 | 迭代上限。预算:简单技巧 ~1000,步态类 4000~6000(AGENTS.md 官方区间) |
| `--agent.logger` | `tensorboard` / `wandb` | 日志后端。**没登录 wandb 用 `tensorboard`(零配置,本地 tfevents 文件)**;`wandb` 需 `wandb login` |
| `--agent.resume True` | flag | 从 checkpoint 续训(见第 4 节,注意预算会累加) |
| `--agent.load-checkpoint` | 文件名 | 要加载的 checkpoint 名,如 `model_2000.pt`(配合 resume) |
| `--agent.load-run` | 目录名 | 精确指定从哪个 run 目录加载(默认自动取最新) |
| `--agent.run-name` | 字符串 | 给本次 run 起名(日志目录后缀) |
| `--hf-jobs` | flag | 改到 Hugging Face 云 GPU 上训练(需先 `hf auth login`) |
| `--gpu-ids` | `0` / `all` / `None` | 多卡训练 `all`;`None` = 纯 CPU 模式(极慢,仅调试) |

### 3.2 迭代预算怎么算(重要)

- 每迭代 = 每个环境跑 `24` 步(`NUM_STEPS_PER_ENV`)。
- 4096 环境 × 24 步 × 4000 迭代 ≈ **3.93 亿环境步**。

---

## 4. 中断与续训(重点!)

### 4.1 为什么中断也不怕:checkpoint 自动保存

- 训练每 **250 迭代**自动存一份 `model_XXXX.pt`(可改 `--agent.save-interval`)
- 训练正常结束时还会存最终版 + 自动导出 ONNX
- **所以**:任何时刻被 kill / 崩溃 / 断电,最多损失最近 250 迭代,其余 100% 可续

### 4.2 checkpoint 在哪

```bash
logs/rsl_rl/<experiment_name>/<run目录>/
├── model_0.pt / model_250.pt / model_2000.pt ...   # 每 250 迭代一个
├── <run>.onnx                                      # 训练结束自动导出的部署文件
├── events.out.tfevents.*                            # tensorboard 曲线数据
└── params/ agent.yaml env.yaml                      # 本次配置快照
```

> 桌面查看最近存档:`ls -t logs/rsl_rl/velocity/*/model_*.pt | head`

### 4.3 续训命令(模板)

```bash
uv run train Mjlab-Velocity-Flat-MicroDuck \
  --env.scene.num-envs 4096 \                     # 必须和原来一致
  --agent.max_iterations <新增迭代数> \           # ← 见下方公式!
  --agent.load-checkpoint model_2000.pt \         # 要续的 checkpoint(自动选最新可省略)
  --agent.load-run 2026-09-05_17-40-38_velocity \ # 可选:锁定 run 目录
  --agent.resume True \
  --agent.logger tensorboard
```

### 4.4 ⚠️ 续训迭代预算公式(最容易踩的坑)

```
显示的总迭代 = 已加载的迭代 + 你命令行给的 max_iterations
```

| 实际情况 | 正确的 `--agent.max_iterations` | 日志里看到 |
|---|---|---|
| checkpoint 2000,想总共训到 4000 | **2000** | `2013/4000` |
| checkpoint 2000,想总共训到 6000 | **4000** | `2013/6000` |
| checkpoint 3500,想总共训到 4000 | **500** | `3505/4000` |

> **反面教材**:用户传了 `--agent.max_iterations 4000` 续 2000 的档,
> 结果日志显示 `2013/6000`——**多训了 2000 迭代**,就是没按公式换算。

### 4.5 用 tmux 跑长训练(强烈推荐,避免"关终端就停")

```bash
tmux new -s train                    # 新建会话
uv run train ...                     # 在里面跑训练(前台即可)
# 让它后台挂着:按 Ctrl+B,再按 D  detach
tmux attach -t train                 # 随时回来查看(实时滚动)
# 训练过程想中断:
#   会话内 Ctrl+C,或 tmux kill-session -t train
```

好处:关终端、断 SSH、切换工作都不影响训练(比 nohup 更可管理)。

### 4.6 正确停训 vs 强杀

- **想优雅停**:等训练结束(自动存最终档 + ONNX);或 `tmux` 里 Ctrl+C(rsl_rl 容错退出,已存的 checkpoint 都在)。
- **强杀**(`kill -9` / `killall train`):损失 ≤250 迭代,之后 resume 即可,不伤大局。
- **不要边跑边改训练进程的预算/环境数**——resume 时环境数必须与原来一致。

---

## 5. 监控训练

### 5.1 看实时日志

```bash
tail -f /tmp/train_velocity.log                 # 完整滚动
watch -n 10 "tail -30 /tmp/train_velocity.log | grep -E 'Learning iteration|Mean reward|Mean episode length|fell_over|Iteration time|ETA'"
tail -f /tmp/train_velocity.log | grep -E "Learning iteration|Iteration time|Mean reward|ETA"
```

### 5.2 看曲线(tensorboard)

```bash
# 方法 A:Bash 里直接起(推荐):
uv run --with tensorboard tensorboard --logdir logs/rsl_rl/velocity --port 6006
# 浏览器打开 http://localhost:6006

# 方法 B:用离线工具看 tfevents 文件
# logs/rsl_rl/velocity/<run目录>/events.out.tfevents.*
```

### 5.3 看显卡

```bash
watch -n 1 nvidia-smi     # 利用率/显存/功率实时刷新
nvidia-smi dmon           # 紧凑滚动表
```

### 5.4 指标怎么判读(AGENTS.md 规则)

| 现象 | 判读 |
|---|---|
| `nan_state: 0.0000` | ✅ 无 NaN,健康 |
| 所有 `Episode_Reward/<惩罚项>` ≤ 0 | ✅ 无奖励漏洞(任何惩罚项变正 = 出问题) |
| `fell_over` 从 100+ → 个位数/≈0 | ✅ 摔得越来越少 |
| `Mean episode length` 接近上限(1200) | ✅ episode 基本跑满 |
| `Episode_Reward/track_linear_velocity` 持续增长 | ✅ **主任务**(追速度)真在学 |
| `Curriculum/*` 权重按计划变化 | ✅ 课程机制推进正常 |
| `Mean reward` 忽高忽低 | 正常,看趋势别盯单点(课程权重变化会让绝对值跳) |

---

## 6. 查看训练效果(3D)

### 6.1 打开 MuJoCo 窗口看指定 checkpoint

```bash
uv run play Mjlab-Velocity-Flat-MicroDuck \
  --checkpoint-file logs/rsl_rl/velocity/<run目录>/model_2000.pt
```

- 会打开实时 3D 窗口(50Hz),**一直运行直到你关窗口 / Ctrl+C**
- 鼠标转视角、滚轮缩放、`Space` 暂停
- 常用附加参数:

```bash
--num-envs 24          # 限制并行环境数(画面更清晰、显存小)
--no-terminations      # 关闭终止(只看动作不摔停)
```

### 6.2 录制视频(不需要重训!)

```bash
uv run play Mjlab-Velocity-Flat-MicroDuck \
  --checkpoint-file logs/rsl_rl/velocity/<run目录>/model_4000.pt \
  --video True
# 视频保存到:logs/rsl_rl/velocity/<run目录>/videos/play/*.mp4
# 想录多长:--video-length 500(帧);多个同时:--num-envs 24
```

> 相比训练时加 `--video True`(每步渲染,训练明显变慢),**训练后用 play 录视频零影响**。

---

## 7. 导出与部署

### 7.1 训练结束,ONNX 已自动导出

正常结束(或 resume 到 max_iterations 后结束)时,日志目录会生成:

```
logs/rsl_rl/velocity/<run目录>/<run目录>.onnx
```

这个文件已经把观测归一化参数**烘焙进图内**,是唯一可部署的格式。

### 7.2 手动重新导出(可选)

```bash
# 从 wandb run 导出(需 wandb run path)
uv run scripts/export.py Mjlab-Velocity-Flat-MicroDuck --wandb-run-path <entity/project/run_id>
```

### 7.3 在纯 CPU 的 MuJoCo 里演练部署

```bash
uv run scripts/infer_policy.py --walking logs/rsl_rl/velocity/<run目录>/<run目录>.onnx
# 键盘控制:方向走路、G 触地、Y 坐站、R 滚翻…
```

### 7.4 发布到 Hugging Face Hub(真机安装用)

```bash
# 从已导出的 ONNX 直传(不需要 wandb):
uv run publish --onnx logs/rsl_rl/velocity/<run目录>/<run目录>.onnx \
  --repo <你的用户名>/microduck-my-walk --kind perpetual --slot walk
# 先干跑看会传什么:
uv run publish --onnx <...>.onnx --repo <...> --kind episodic --duration-s 4.0 --dry-run
```

> 依赖:已登录 HF(`hf auth login`)。真机上 `sudo robotctl policy add/load` 安装。
> 本文档不展开,详见 README "Publishing a policy"。

---

## 8. 实战故障排查

### 8.1 训练进程没了?先别慌

```bash
pgrep -af "python" | grep train     # 还在吗
tail -5 /tmp/训练日志.log            # 最后输出到哪了
ls -t logs/rsl_rl/velocity/*/model_*.pt | head -1   # 最新 checkpoint
```
→ 按第 4 节 resume 续上(损失 ≤250 迭代)。

### 8.2 "显卡没跑满 / 太慢"

- 实测瓶颈是 **CPU 单线程编排**(mjlab 架构),GPU 利用率 70% 属正常,**不是配置问题**。
- `nproc` 多核无济于事:框架内没有多核 CPU 开关;想更快只有:多卡(`--gpu-ids all`)、换更快单核 CPU、或云端 `--hf-jobs`。
- 每个任务的合理时长(4096 envs,RTX 2070 级别):简单技巧 ~1h,步态 4000 迭代 ~3.5~4h,6000 迭代 ~5~6h。

### 8.3 屏幕周期性黑一下

- 训练把 GPU 推到 70%+ 时,Wayland 合成器(gnome-shell)周期性掉帧 → "黑一下亮"循环。
- **不影响训练**;训练结束自动恢复。想根治:登录界面切 **"Ubuntu on Xorg"** 会话。
- 自检是不是训练引起:`kill -STOP <训练pid>` 观察 3 分钟,不黑 = 训练引起;`kill -CONT <训练pid>` 恢复(零损失)。

### 8.4 wandb 报错 `No API key configured`

- 没登录 wandb 时训练会卡在启动。解决:`--agent.logger tensorboard`(见 3.1),或 `wandb login`。

### 8.5 CUDA 各种报错速查

| 报错 | 大多原因 | 处理 |
|---|---|---|
| `cudaErrorDevicesUnavailable` + 内核 `NVRM ... DMA mapping` | 驱动损坏 | `sudo reboot` |
| `index out of range` 于 `select_gpus` | aarch64 CPU 版 torch | 看 AGENTS.md 的 torch 源配置 |
| 显存不足(OOM) | 环境数太大 | 降到 2048 或 1024 |
| `--gpu_ids None` 也报 CUDA | 环境变量残留 `CUDA_VISIBLE_DEVICES` | `unset CUDA_VISIBLE_DEVICES` |

### 8.6 其他日常

```bash
df -h                                   # 磁盘满没(日志/缓存会占空间)
du -sh /tmp ~/.cache                    # 大头排查
rm -rf logs/rsl_rl/velocity/<旧run目录>   # 清理不想要的 run(先确认已不再需要!)
```

---

## 9. 官方预算与推荐配置速查

来源:仓库 `AGENTS.md`。

| 用途 | 配置 |
|---|---|
| 冒烟测试(必跑) | 64 envs × 5 iter |
| 简单技巧(Roulade / Kick / Spin 等) | 4096 envs × ~1000 iter |
| **步态类(Velocity / StandUp / Stand 等)** | **4096 envs × 4000~6000 iter** |
| 续训预算 | 见 4.4 公式 |
| IPC/CI 测试 | `uv run --with pytest pytest tests/`(CPU 即可) |

---

## 10. 教程动线(第一次完整走一遍)

```bash
cd microduck_rl && uv sync                       # 1. 装依赖
uv run list-envs                                 # 2. 选任务
uv run train ... --env.scene.num-envs 64 --agent.max_iterations 5    # 3. 冒烟
tmux new -s train                                # 4. 开 tmux
uv run train ... 4096/4000 --agent.logger tensorboard    # 5. 正式训练
# 6. 监控:tail -f / tensorboard / nvidia-smi
# 7. 中断了就 resume(公式!4.4)
# 8. 完成后:play 看效果 / play --video 录视频
# 9. publish 发布(可选)
```

Happy training!