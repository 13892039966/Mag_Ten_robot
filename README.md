# 四绳 SOFA＋libigl 仿真

本目录是 MagRobot 下独立的四绳导航试验。采用 SOFA25.06、CPython3.12 和独立 Conda 环境；使用这里的入口启动。

## 控制代码从哪里看

| 文件 | 职责 |
| --- | --- |
| `tendon_sim/closed_loop.py` | `DLSConfig` 控制参数、`BoundedDLS.command()` 有界 DLS、`slew_targets()` 每物理步速度限幅 |
| `tendon_sim/cc_predictor.py` | `CCFourTendon` 常曲率预测、末端位置与雅可比 |
| `tendon_sim/live_scene.py` | `Trial` 组织20ms反馈、动作下发、安全检查和停止条件 |
| `tendon_sim/references.py` | 路线目标、前视与出口判断 |
| `tendon_sim/body_geometry.py` | 当前完整镜身 `BodyConfig` 参数与113节点布局 |
| `tendon_sim/physics.py` | 梁、四绳、导向、碰撞球和 SOFA 求解器 |
| `tendon_sim/libigl_wall.py` | 管壁查询与原生适配库加载 |
| `run.py` | 命令行、场景创建、物理推进和记录核验 |

调用关系：`run.py → physics.py + live_scene.py → closed_loop.py → cc_predictor.py`。
管壁调用：`libigl_wall.py → native/libigl_adapter/`。

注意：`cc_predictor.py` / `model.py` 保留原小尺寸模型的默认值与历史说明；当前完整镜身入口显式传入 `BodyConfig`（主动200mm、被动1800mm）。调整本试验尺寸先看 `body_geometry.py`，不要只修改预测器默认值。

## 启动

从本目录执行，输出目录必须不存在：

```bash
./run.sh --dt .005 --cpu 4 --seconds 1 --output outputs/smoke_5ms
```

`run.sh` 自动切换到本目录并激活 `setup/activate.sh` 指定的环境。需要交互开发时：

```bash
source setup/activate.sh
python run.py --dt .005 --cpu 4 --seconds 200 --output outputs/dt5_200s
```

物理步长默认5ms；控制周期始终20ms，5/10/20ms分别对应4/2/1次物理更新。10/20ms是独立试验参数，不能当作等价显示降帧。

## 目录分工

- `tendon_sim/`：控制、模型、物理、安全和记录模块。
- `assets/`：完整几何、碰撞网格与距离场，随源码一起保留。
- `native/`：原生接口与随包 libigl 源码、许可；`build/` 为本机生成文件。
- `setup/`：固定依赖锁、安装、激活、编译与 doctor。
- `tests/`：便携逻辑测试。
- `reference/`：Windows参考数据，不能当作Linux运行成绩。
- `.install/`：本机环境路径、显式锁与doctor报告，不纳入版本控制。
- `outputs/`：今后运行的新输出，不纳入版本控制。

完整迁移流程和物理限制见 [README_ZH.md](README_ZH.md)。原始文件校验由 `BUNDLE_SHA256.json` 与 `verify_bundle.py` 保留。

## 本机验证与搬迁

本机已完成 Ubuntu22.04 / i5-14400 安装、原生编译、doctor四线程、5项测试及短窗参考对照。三种步长各200s均未到达出口；10ms至终止于285.54s绳行程饱和停滞。重复BaseMass警告、质量装配与实物标定问题仍保留。

原验收数据保留在 `/home/maxlab1/Documents/Codex/2026-10-29/files-mentioned-by-the-user-tendon/outputs/`。源码从该聊天的 `work/` 搬到本目录；旧源码路径是指向这里的兼容链接，供旧记录中的资产路径访问。Conda环境仍在 `/home/maxlab1/.local/share/tendon-libigl-sofa2506`。

搬迁阶段保留原算法与资产，并在新路径重新编译原生库。后续控制调参修改见下节；搬迁验证日志保存在 `.install/relocation-*.log`。

## 自动生成真实管壁 GIF

`./run.sh ...` 和 `python run.py ...` 现在默认在仿真结束、记录核验并释放 SOFA 后，依次执行路线分析和真实管壁 GIF 渲染，无需再次手动生成。只读取保存的姿态，不重跑物理。输出同目录下 `wholebody_with_colon.gif`、首末帧 PNG、`independent_verification.json`、`colon_render_verification.json` 和 `postprocess.json`。

默认每1秒模拟取一帧，每帧播放100ms（10倍速）。可用 `--gif-sample-seconds .2 --gif-frame-ms 100` 改为2倍速，或用 `--no-gif` 跳过自动分析与动画。动画耗时单独记录在渲染报告中，不计入物理 animate 耗时。

饱和停滞或安全停止的动画仍表示原停止原因，不会被改成成功；核验前的异常不自动渲染。后处理失败保留原始数据，写入 `postprocess.json`，返回非零状态。

已有记录可直接补生成：

```bash
source setup/activate.sh
python analyze.py outputs/dt20_terminal
python render.py outputs/dt20_terminal --sample-seconds 1 --frame-ms 100
```

原始 `BUNDLE_SHA256.json` 保留作迁移基线。当前改动的原包文件为 `run.py`、`tendon_sim/live_scene.py` 和 `tendon_sim/references.py`；`verify_bundle.py` 会报告原包差异。物理、安全、原生碰撞、资产和依赖锁未改，详见 `tuning/physical_baseline_verification.json`。

## 控制策略与历史调参

当前全程使用同一套有界DLS控制与中心线前瞻目标。可配置项只有lookahead、position_gain_per_s、velocity_damping、damping_mm_s、smoothing_mm_s、feed_speed_scale、cable_speed_scale。尚未实现逐点到达后再切换目标。

2026-10-30已删除所有接近出口才启用的控制参数、分支、配置、未验证原型及重复源码副本。真实出口5mm到达判定、物理限制和安全检查保留。旧版本源码可从Git历史查看。

此前12组入口初态试验全部未通过。仅改前瞻距离15/25/40mm即可到出口附近，最近真实出口距离分别12.3519/12.3527/12.3483mm；末段补偿的最好保存观测11.9733mm也未通过，且旧试验超时退出的末帧核验失败。这些历史结果不能代表当前删除后的新运行验证。

无末段特殊控制的25mm前瞻候选可按以下命令运行，输出目录必须不存在：

```bash
./run.sh --dt .02 --cpu 4 --until-terminal \
  --controller-config tuning/configs/lookahead25.json \
  --output outputs/reproduce_lookahead25
```

详细删除范围与历史记录说明见[tuning/CONTROL_CHANGES.md](tuning/CONTROL_CHANGES.md)。
