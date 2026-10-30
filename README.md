# 四绳 SOFA＋libigl 仿真

本目录是 MagRobot 下独立的四绳导航试验。采用 SOFA25.06、CPython3.12 和独立 Conda 环境；使用这里的入口启动。

## 控制代码从哪里看

| 文件 | 职责 |
| --- | --- |
| `tendon_sim/closed_loop.py` | `DLSConfig` 控制参数、`BoundedDLS.command()` 有界 DLS、`slew_targets()` 每物理步速度限幅 |
| `tendon_sim/cc_predictor.py` | `CCFourTendon` 常曲率预测、末端位置与雅可比 |
| `tendon_sim/live_scene.py` | `Trial` 组织20ms反馈、动作下发、安全检查和停止条件 |
| `tendon_sim/references.py` | 外部轨迹读取、逐点到达与完成判断 |
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

## 外部轨迹逐点跟踪

默认读取离线采样的 `assets/trajectories/centerline_10mm.csv`（135点，世界坐标、单位米，沿原中心线弧长每10mm采样，保留起终点，最后一段约8.875mm），也可用 `--trajectory /绝对路径/trajectory.csv` 指定外部轨迹。CSV必须有 `x_m,y_m,z_m` 列；原文件的其他列可保留。至少两个有限三维点，严格按文件顺序执行，不插值、不重采样、不生成新点。

从第一个点开始固定追踪。每20ms根据最新状态重新计算雅可比和DLS动作；物理更新后，末端中心到当前点的真实三维距离不超过5mm即确认到达，最多切换一个点。未到达不换点，不按时间或最近投影跳点。密集点可能连续几个周期依次完成。停滞检查当前点误差是否改善，并在切换时重置；原物理和安全限制继续生效。

```bash
./run.sh --dt .02 --cpu 4 --until-terminal \
  --trajectory assets/trajectories/centerline_10mm.csv \
  --output outputs/waypoints_20ms
```

不再支持lookahead或任何末段特殊参数。全程DLS参数和速度比例仍可通过controller-config配置。此前的移动前瞻JSON配置已删除；历史试验记录和旧版本保留在Git历史与tuning记录中，不能作为当前逐点控制的运行成绩。

输出保存原始trajectory.csv及其SHA、点数、单位和实际到达半径判据。trace记录内部点编号、当前目标、误差和到达事件，供核验顺序；回放只显示目标和误差、实际到达阈值，不显示点编号。

所有外部点完成后标记trajectory_completed；只有同时处于真实结肠出口5mm内，才标记station_reached。自定义轨迹结束不等于通过整段结肠。默认物理步长仍5ms，控制周期固定20ms。

详细改动与历史说明见[tuning/CONTROL_CHANGES.md](tuning/CONTROL_CHANGES.md)。

原始1340点轨迹保留。采样只在离线工具中执行，控制器读取文件后不再次采样。复现采样时指定不存在的输出路径：

```bash
python setup/sample_trajectory.py assets/original/centerline_world_m.csv /新的输出路径/centerline_10mm.csv --spacing-mm 10
```

采样报告保存源文件和输出SHA、点数及末段长度。10mm是原始折线的弧长间隔，弯道中相邻采样点的三维直线距离可能略小于10mm。

## 实时有头长测

```bash
./run.sh --gui --dt .02 --cpu 4 --until-terminal --output outputs/gui_dt20_waypoints_terminal
```

使用现有SOFA25.06的Sofa.SofaGL渲染和独立环境内的PySide6窗口；本环境没有官方Qt/ImGui GUI插件，未混装其他版本。窗口显示真实原始管壁、实际节点姿态生成的镜身表面和当前目标。显示更新不替代或跳过物理步进。支持按住左键旋转、按住右键平移、滚轮缩放、结肠/全身视角、暂停及停止保存。关闭窗口按人为中断保存，不算通过。运行中保存gui_latest.png，首末帧为gui_first.png/gui_final.png。

GUI可视开销独立于animate计时，总现实耗时仍包含窗口开销。默认结束后执行分析和GIF生成。10mm/5mm是当前默认外部轨迹和到达条件，不是出口特殊控制。

界面布局：顶部按钮栏、中间自适应三维视口、底部固定26像素状态栏。管壁由SOFA绘制；镜身表面每次显示刷新直接从当前物理节点姿态重建并绘制，避免Ogl缓存或延迟映射导致画面停在旧状态。显示顶点不参与物理求解。

## 分项计时

```bash
python setup/profile_run.py --gui --dt .02 --cpu 4 --seconds 150 --no-gif --output outputs/profile_20ms
```

profile.json每约2秒更新，包含物理推进、控制回调、DLS、安全、录制、GUI处理/绘制/抓图以及SOFA内部阶段采样。层级计时包含子项，不应直接相加；disjoint_animate给出不重叠分类。计时从调用正常入口开始，不含分析器预先导入模块的时间。脚本使用同一入口和初态，不从不完整检查点恢复。诊断最小化可加--profile-minimized。

已修复窗口最小化、隐藏或Qt报告未暴露时仍持续请求重绘和抓图的路径；继续处理恢复/停止事件，物理、安全和每步录制照常执行。当前正在运行的旧进程不会自动加载此补丁，下次启动生效。更完整的物理/显示分进程仍是后续优化候选，尚未实现。

## 无头计算后回放（当前默认5mm到达半径）

```bash
./run.sh --dt .02 --cpu 4 --arrival-radius-mm 5 --until-terminal --wall-seconds 600 --output outputs/headless_dt20_r5
```

不传--gui即完全无头推进，先完成每步记录核验，再由独立进程分析并生成真实管壁GIF。--arrival-radius-mm默认5；复现先前3mm试验可显式传3。绳行程±30mm、张力2N、速度及安全阈值均不变。默认GIF每模拟1秒取一帧、每帧100ms（10倍速）；慢放已有记录可单独调用render.py调整播放速度，不重跑物理。

stalled_at_cable_limit是当前控制器的操作性停止条件（固定点未到达、误差持续缺少改善且绳命令到限），不是物理不可达证明。新记录在summary.detail保存目标、误差、到达半径和停滞窗口说明。历史记录和GIF仍保留原到达半径，不追改为5mm。

## 位置增益6对照（未通过）

`tuning/configs/gain6.json`仅覆盖position_gain_per_s=6，默认仍为4。保持20ms物理步长、10mm轨迹、5mm到达半径、10秒停滞窗口，其余参数不变。两组均完成78/135点，跟踪第79点时触发原停滞条件；增益6未改善路线完成度。完整数值与记录核验摘要见tuning/gain6_comparison.json。

```bash
./run.sh --dt .02 --cpu 4 --arrival-radius-mm 5 --controller-config tuning/configs/gain6.json --until-terminal --wall-seconds 600 --output outputs/reproduce_gain6
```
