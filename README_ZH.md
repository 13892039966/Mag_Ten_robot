# Ubuntu22.04四绳SOFA＋libigl迁移包

这是源码与资产迁移包，**尚未在目标Linux机安装、编译或运行**。当前机器的Windows试验不能冒充Ubuntu验收。目标：Ubuntu22.04 x86_64／i5-14400；采用独立Conda环境、SOFA25.06、CPython3.12，不修改系统Python或强化学习环境。

## 先理解步长

`--dt .005/.01/.02`是实际SOFA物理步长5/10/20ms，不是显示跳帧。控制周期始终20ms，所以每次控制之间分别4/2/1次物理更新。步长会改变接触、张力和形态，反馈动作也可能分化。Windows200s试验中10/20ms相对5ms的中途最大轨迹差约16.94/17.28mm，不能称等价。默认保留5ms；10ms是可选提速候选，20ms须单独验收。

## 完整流程

将整个目录复制到Ubuntu本地文件系统，例如`~/robotics/tendon_sofa`，不要只复制run.py，也不要混用Windows的DLL。需要预先安装Miniforge/Conda。

压缩包顶层目录名为`tendon_sofa_ubuntu2204_20261029`；解压后可整体重命名为`tendon_sofa`，或者在下面命令中使用解压后的实际路径。

```bash
cd ~/robotics/tendon_sofa
python3 verify_bundle.py
bash setup/install.sh
source setup/activate.sh
bash setup/build.sh
python setup/doctor.py
PYTHONDONTWRITEBYTECODE=1 python -m unittest discover -s tests -v
python run.py --dt .005 --cpu 4 --zero --seconds .1 --output outputs/zero_5ms
python run.py --dt .005 --cpu 4 --seconds 1 --output outputs/smoke_5ms
python compare_reference.py outputs/smoke_5ms
```

每一步成功再继续。doctor只证明模块、插件、原生库和实际线程池可加载，不证明物理正确。若安装失败，保留日志，不去掉SOFA版本约束，不混另一版本库。安装脚本拒绝覆盖现有环境；默认安装位置`~/.local/share/tendon-libigl-sofa2506`，可传另一个不存在的路径。环境安装成功后保存本机实际显式锁；包内跨平台solver记录只是依赖解析证据，不是Linux执行证据。

`compare_reference.py`核对包内Windows入口1秒／201帧参考的全身位置和50个控制时刻的绳力。它是无接触短窗门槛，不是Linux过弯接触验收。便携版Python在现有Windows DLL桥接下已实际推进1秒，并逐帧复现原新库参考（位置／速度／命令最大绝对差0）；此桥接不随包提供DLL，也不代表Linux运行过。

已用linux-64目标、glibc2.35／Linux5.15虚拟条件成功解析183个依赖；`setup/linux64-explicit.txt`固定全部下载URL与校验值，安装直接使用该锁，不重新追随latest。完整记录见`setup/linux64-resolved-records.json`；这仍不表示Linux实际装载／编译通过。

确认短测通过后运行：

```bash
python run.py --dt .005 --cpu 4 --seconds 200 --output outputs/dt5_200s
python run.py --dt .01  --cpu 4 --seconds 200 --output outputs/dt10_200s
python run.py --dt .02  --cpu 4 --seconds 200 --output outputs/dt20_200s
# 不设模拟时长上限，仍按原安全／饱和停滞／出口条件停止：
python run.py --dt .01 --cpu 4 --until-terminal --output outputs/dt10_terminal
# 完成后核验路线并生成真实管壁回放：
python analyze.py outputs/dt10_200s
python render.py outputs/dt10_200s --sample-seconds 1 --frame-ms 100
```

输出目录必须新建。以上是200秒**模拟时间**，不是200秒现实时间。长跑建议在目标机保持唤醒、用稳定供电，首次对照串行运行。中止用Ctrl+C，保存已完成记录；物理失败和人为中断不能叫成功。

## 代码调用关系

`run.py → tendon_sim/physics.py`构建梁、四绳、入口外导向和碰撞球；`tendon_sim/libigl_wall.py → native/libigl_adapter`替换管壁查询；`tendon_sim/live_scene.py → closed_loop.py → cc_predictor.py`执行20ms有界DLS。每个物理步有绳速限幅、力学求解、安全检查和完整姿态录制。主要学习入口是这四个文件；环境安装、编译、重复参数试验可自动处理。

主动200mm、被动1800mm、113节点、401个碰撞球；四绳±30mm／1mm/s／2N、进给5mm/s。只固定入口外导向，腔内柔性被动段自由受力。绳力约束按2N×dt换算为冲量，读取再除dt。GenericConstraintSolver容差1e-9，最大500次。CPU4真实线程池在init前后设置并读回，启用原来的三项并行设置；不会因为有16逻辑线程就默认全部占用。

## 碰撞与安全

本包是当前新库版本：4mm粗距离场仅保守排除远点，近壁调用libigl2.6.0的AABB／pseudonormal精确三角面查询；171776面网格用于碰撞，原686080面用于安全与回放。每个球单最近接触与旧多接触模型不同，不能因几何查询精确就宣称力学等价。源资产单位米，内径40mm／外径46mm，两端开口。

已有重复BaseMass警告保留，质量装配及实物标定尚未解决；未完成精确全网格自交认证。安全为离散采样，不是连续无碰撞证明。本包不宣称实物精度或任意路径可通过。

## 输出与验收

`run_contract.json`记录dt／实际线程池／平台／原生库SHA；`physical_recording`保存每步全113节点位置姿态、速度和命令，分块SHA；`trace.jsonl`每20ms保存末端、目标、张力和接触指标；`summary.json`记录真实停止原因；`recording_verification.json`核对块、帧数及末帧。`timing.json`区分加载和含控制／安全／记录的animate耗时。出口成功是镜头到出口5mm内，不是整根2m镜身全部穿出。

跨系统首测先比较同初态、相同参数／输入和短窗，不以“能运行”当作轨迹一致；保存环境显式锁、doctor报告、run_contract和原始输出后再评估新CPU提速。reference目录为Windows200s实验记录，**不是Linux成绩**。

## 来源与许可

SOFA25.06及原生插件来自[官方发布](https://github.com/sofa-framework/sofa/releases/tag/v25.06.00)；Conda由[SOFA维护渠道](https://prefix.dev/channels/sofa-framework/packages/sofa-devel)提供linux-64包，其元数据列出glibc最低2.17，不依赖Ubuntu系统Python版本。安装固定25.06，Python3.12与所选SofaPython3构建对齐。

libigl固定v2.6.0提交`40e7900ccbd767f1f360e0eb10f0f1a6432e0993`，上游[libigl](https://github.com/libigl/libigl/tree/v2.6.0)头文件和MPL2／GPL许可随包保留；未重新自写BVH。SofaDistanceGrid的官方源码许可声明保留在源文件中，薄适配层负责SOFA接触接口、SHA及资产加载。跨平台修改局限于路径／动态库加载、原生构建和可观测输出，未单边修复原物理模型。
