# 准静态 GGV 与圈速分析

本目录保存可以直接运行的准静态分析入口。底层 GGV、圈速和结果整理函数位于 `scripts/ggv/` 与 `scripts/lap_time/`。

## 运行入口

| 入口 | 用途 |
|---|---|
| `simulation/plotCurrentGGV3D.m` | 生成当前参数的三维 GGV |
| `simulation/runQuasiStaticLapTime.m` | 使用已有 GGV 计算单工况圈速 |
| `simulation/runLapTimeParameterSweep.m` | 执行单参数或双参数圈时扫描 |
| `plotting/plotQuasiStaticTrackParameterMap.m` | 按赛道位置显示选定参数 |
| `plotting/plotQuasiStaticParameterTrace.m` | 绘制两个参数的关系曲线 |
| `plotting/plotQuasiStaticAnalysisResults.m` | 汇总 QuasiStatic GGV 能力扫描结果 |

完整配置、运行命令和结果目录见上级 [仿真入口使用说明](../README.md)，算法边界见 [GGV 与准静态圈速参考](GGV_LapTime_Reference.md)。

## 输出与验证

- GGV 输出：`results/quasi_static/ggv/`；
- 单工况圈速：`results/quasi_static/lap_time/<event>/`；
- 参数扫描：`results/quasi_static/parameter_sweep/<event>/`；
- 回归与交叉验证：[`tests/quasi_static/`](../../tests/quasi_static/README.md)。

准静态结果不包含驾驶员跟踪误差、执行器动态、轮胎温度和瞬态悬架效应。
