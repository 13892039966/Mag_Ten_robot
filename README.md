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

## 2026-10-29 20ms调参：未通过

12组从相同入口初态开始的候选试验均未达到真实出口5mm内。前视15/25/40mm可越过原约0.78m停滞区，但多数在出口附近偏离并触发安全停止。

最接近出口的已保存观测为 `outputs/tune_terminal_z12_la25/`：前视25mm、最后200mm内平滑增加世界Z方向-12mm控制目标补偿。最近出口11.973mm、最终保存帧16.163mm，386.26s后触发1200s现实时间上限。**不是通过，也不是推荐默认参数**。真实出口和5mm验收判据未移动。

这一旧试验还有录制缺口：超时在回调内关闭记录后，SOFA额外推进了一步，原进程最终状态核验失败；原timing和recording_verification缺失，不补造。因此只把SHA/时间网格已验证的386.26s保存前缀用于观察和回放。超时退出现已移到物理步之间，专门299帧短回归通过末帧/内存一致性检查，旧试验不追认修复。

结论、对照表和审计在 `outputs/tuning_20261029/`；回放在 `outputs/tune_terminal_z12_la25/wholebody_with_colon.gif`。GIF为原686080面真实管壁、10倍速；显示的goal error是补偿控制目标误差，不能替代真实出口距离。

调参需显式传入JSON，默认步长仍5ms，默认控制不采用上述失败配置。例如，仅供复现该未通过候选，使用修复后的退出路径：

```bash
./run.sh --dt .02 --cpu 4 --until-terminal --wall-seconds 1200 \
  --controller-config tuning/configs/terminal_z12_la25.json \
  --output outputs/reproduce_closest_20ms
```

`--wall-seconds` 是现实时间上限。候选筛选可用 `--no-gif` 跳过后处理，普通运行仍默认自动生成GIF。运行参数、源码哈希保存在run_contract和录制metadata。当前10项测试、默认1秒参考对照均通过。

代码：`run.py` 解析参数和管理超时，`live_scene.py` 注入DLS配置、速度比例、可选末段预紧/增益/控制敏感度，`references.py` 可补偿控制目标；`closed_loop.py` 的DLS求解算法保持原样。配置文件有准备但未运行的候选，完整实测清单以 `tuning/experiment_table.csv` 为准。

`tuning/audit_limits.py RUN` 核对每个保存步的命令限值、原始初态和未偏移的出口；它不替代原进程的最后内存状态核验。`tuning/summarize.py` 汇总候选，`run_batch.py` 带本次固定9小时墙钟截止时间，已过期，不能直接用于新一轮任务。

检查点重建首步偏差17.8mm且出现虚假接触突变，方法已作废；数据移至 `outputs/rejected_checkpoint_probes/`，不能用于选优或整段验收。未启用的方向补偿原型归档在 `tuning/unvalidated_prototypes/heading_revision/`，没有被验证或计入成绩。当前代码快照在 `tuning/final_source/`；旧最佳候选实际录制模块快照在 `outputs/tuning_20261029/best_observed/recorded_runtime_sources/`。

时间：9小时窗口为上海时间2026-10-29 21:14至2026-10-30 06:14；实际候选计算集中在10月29日21:15至23:35，累计仿真耗时约2小时16分。之后存在未执行试验的间隔，续接时已超过窗口，未再启动调参搜索；仅完成核验、回放和超时退出缺陷的短回归。本记录不宣称持续执行了9小时。
