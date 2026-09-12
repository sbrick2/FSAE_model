# 时域闭环赛项仿真

本目录保存 7DOF/10DOF 时域闭环仿真和结果分析入口。

## 仿真入口

运行 `simulation/runLapSimulation.m`，在脚本顶部选择赛项、车辆自由度、驾驶员、轮胎模型、求解器档位和输出名称。结果默认写入：

```text
results/time_domain_closed_loop/<event>/
```

输入配置、结果结构和验收要求见 [圈速仿真与结果分析规格](Lap_Simulation_Spec.md)。

## 绘图入口

`plotting/` 包含统一结果绘图和专项分析函数，入口配置、字段选择、四轮通道、单位与导出方式见 [绘图使用指南](plotting/README.md)。

## 相关文档

- [仿真入口总说明](../README.md)
- [结果信号参考](../Result_Signal_Reference.md)
- [自适应 Autocross 驾驶员](../../models/driver/Adaptive_Autocross_Driver.md)
- [TorqueVectoring 扭矩矢量](../../models/controller/Torque_Vectoring.md)
- [UnifiedControl 统一控制分配](../../models/controller/Unified_Control_Allocation.md)
