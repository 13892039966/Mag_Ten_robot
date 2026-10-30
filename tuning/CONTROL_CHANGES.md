# 控制改动与实测配置

本目录保留失败候选用于复现；没有通过整段结肠的配置，默认控制未被替换。

## 不使用末段补偿的结果

所有下列试验均为20ms物理步长、20ms控制周期、4线程、相同入口初态。

| 配置 | 相对默认的修改 | 最近真实出口距离 | 停止原因 |
| --- | --- | --- | --- |
| lookahead15.json | lookahead: 8→15mm | 12.3519mm | outside_lumen |
| lookahead25.json | lookahead: 8→25mm | 12.3527mm | outside_lumen |
| lookahead40.json | lookahead: 8→40mm | 12.3483mm | outside_lumen |

三者都推进到出口附近，不能把单调路线游标到达末端视为真实通过。仅按最近出口距离，40mm这一组略小；差异只有约0.004mm，不能据此断定优劣。25mm可作为无末段补偿的后续对照配置，但不是验证通过的推荐配置。

包含末段补偿的最近观测为terminal_z12_la25.json：lookahead=25mm、terminal_zone_m=200mm、terminal_goal_offset_m=[0,0,-12mm]。最近11.9733mm，比单改25mm约改善0.3793mm；未达到真实出口5mm门槛。该旧试验超时退出时末帧一致性核验失败，只能评价已保存前缀，不能宣称完整运行核验通过。

## 新增代码接口

- run.py：controller-config JSON解析、参数范围校验、超时退出修复、自动真实管壁回放。
- tendon_sim/live_scene.py：配置注入、全程速度比例、可选末段增益/速度/预紧/控制雅可比模型。
- tendon_sim/references.py：可选末段世界坐标目标偏置。
- tendon_sim/closed_loop.py：原DLS求解算法保持不变。

terminal_*是本轮新增的可选实验接口；不指定它们不会启用末段补偿。它们没有改变真实出口、成功判据、机器人实际尺寸或安全限制。现有证据没有证明这些补偿能解决通过问题，后续以不含terminal_*的全程参数作为对照。

## 证据与版本

- baseline_source/：调参前控制源码快照，可与当前入口逐文件比较。
- configs/：包括实测和仅准备的配置；是否实测以experiment_table.csv为准。
- experiment_table.csv：12组有效入口初态候选和原始基线，全部未通过。
- unvalidated_prototypes/：未验证原型，不代表当前入口支持或运行过。
- probe_invalidated.json：检查点恢复误差导致相关探测试验无效，不能参与参数排名。
- outputs/、本机环境、编译产物、运行日志不纳入Git；依赖锁、资产、许可和原生源码保留。

复现无末段补偿的25mm候选（输出目录必须不存在）：

```bash
./run.sh --dt .02 --cpu 4 --until-terminal \
  --controller-config tuning/configs/lookahead25.json \
  --output outputs/reproduce_lookahead25
```

该命令不代表保证通过。运行结束仍由原安全、饱和停滞和真实出口条件判断。
