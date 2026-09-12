# Vehicle10DOF 10DOF 动态轮荷 Plant 实施计划

> 状态：功能实现完成；实车校准项开放

## 1. 工作包与完成定义

| Roadmap | 实施内容 | 交付物 | 当前结论 |
|---|---|---|---|
| VerticalRollPitchDynamics | 增加 heave/roll/pitch 状态、角点几何和动态轮荷 | `Vehicle10DOF.slx` | 完成 |
| SuspensionInterfaces | 集中弹簧、阻尼器、防倾杆参数并支持查表替换 | `Vehicle10DOFSuspension`、`vehicle10DOFSuspensionSFunction.m` | 完成 |
| VehicleModelComparison | 统一输入和固定积分网格进行 7DOF/10DOF 回归 | `runVehicle10DOFVerification.m`、对比图和报告 | 完成 |
| Vehicle10DOFCalibration | 快速转向、制动入弯代理工况及闭环冒烟 | Vehicle10DOF 顶层和验证摘要 | 数值基线完成，实车校准待完成 |

## 2. 参数实施顺序

1. 运行 `installE41VehicleParameters`，将用户参数表作为悬架几何和刚度权威值。
2. 运行 `installVehicle10DOFSuspensionData`，安装 TTX25 C12/R12 曲线、暂估惯量和初始状态。
3. 编译 `Vehicle10DOF`，再编译 `VehiclePlant10DOF` 与 `FSAE_Vehicle10DOF_ClosedLoop`。
4. 运行持久化 Gherkin 静平衡测试。
5. 运行 `runVehicle10DOFVerification` 生成数值摘要和图表。

`setupProject` 只负责项目骨架和 Bus 架构；现已改为在物理参数组存在时不覆盖审计后的数据，避免重新配置项目把有限值恢复成 ProjectFoundation `NaN`。

## 3. 关键设计决定

- 保留原 7DOF 模型和 TorqueVectoring 顶层，新增隔离的 Vehicle10DOF 文件，降低对已有脏工作区的覆盖风险。
- 直接采用表内 `76.74/60.64 kN/m` 轮端线刚度；不使用与表内侧倾刚度矛盾的二次派生值。
- 非线性阻尼采用压缩/回弹分支查表；查表结构可直接替换其他旋钮组合。
- 无数值 K&C 数据时外倾为零，不用图形外观估计增益。
- 物理比较在相同 1 ms 固定步长上执行，避免不同连续状态数导致可变步长 Memory 输出出现伪差异。

## 4. 未完成校准项

- 从 CAD、摆锤试验或系统辨识得到 `Ixx/Iyy/Izz`；
- 确认前后阻尼器实际 LSC/HSC/LSR/HSR 设置；
- 导入 K&C 外倾—行程/侧倾数值曲线；
- 澄清参数表防倾杆单列值与总侧倾刚度差值的 8 倍关系；
- 以实车快速转向、制动入弯和路面输入日志完成相关性校准。

