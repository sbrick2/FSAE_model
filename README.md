# FSAE 整车仿真平台

面向四轮独立驱动 FSAE 赛车的 MATLAB / Simulink 整车仿真平台，强调模块可替换、接口可验证和参数来源可追踪。目标环境为 MATLAB / Simulink R2026a。

## 1. 项目简介

项目覆盖以下能力：

- 7DOF 平面车辆动力学与四轮轮速模型；
- 10DOF 动态轮荷、升沉、侧倾和俯仰模型；
- TireSimple、TireMF62 和 TireTTCMap 轮胎模型；
- TorqueVectoring 与 UnifiedControl 控制分配；
- 准静态 GGV、圈速规划和参数扫描；
- 时域闭环赛项仿真、结果绘图和验证回归；
- MATLAB 图形化仿真与结果分析界面。

## 2. 当前状态

- 工程骨架、数据字典、Bus 接口和初始化流程已建立；
- 7DOF VehiclePlant、开环组件和路径跟踪闭环已有验证基线；
- 10DOF 动态轮荷和 UnifiedControl 功能基线已接入，但惯量、阻尼、K&C 外倾曲线和实车相关性仍待校准；
- Adaptive Autocross、准静态 GGV、圈速分析和结果可视化入口已提供；
- 阶段性验证结果保存在 [验证历史](tests/Verification_History.md) 及各测试目录中；
- 车辆质量、几何、惯量和部分控制参数仍可能是占位值，不得直接解释为实车性能结论。

## 3. 快速开始

在 MATLAB 中从项目根目录运行：

```matlab
project = openProject(fullfile(pwd, "FSAE_Simulation.prj"));
initProject();
```

启动图形化仿真中心：

```matlab
app = launchFSAESimulationApp();
```

GUI 包含“车辆参数”“准静态”和“时域闭环”三个主要模块。原有脚本入口仍可独立运行。

## 4. 主要入口

### GUI 仿真中心

```matlab
app = launchFSAESimulationApp();
```

### 时域闭环仿真

入口：`simulation/time_domain_closed_loop/simulation/runLapSimulation.m`

支持 7DOF / 10DOF、`adaptive_autocross` / `reference_speed`、多种轮胎模型和求解器配置。结果默认写入 `results/time_domain_closed_loop/<event>/`。

### 准静态 GGV 与圈速

- GGV：`simulation/quasi_static/simulation/plotCurrentGGV3D.m`
- 准静态圈速：`simulation/quasi_static/simulation/runQuasiStaticLapTime.m`
- 参数扫描：`simulation/quasi_static/simulation/runLapTimeParameterSweep.m`

### 结果绘图与分析

入口：`simulation/time_domain_closed_loop/plotting/plotLapSimulationResults.m`

该入口读取已保存的 MAT 结果，不启动 Simulink。更多绘图入口见 [仿真入口说明](simulation/README.md) 和 [圈速结果绘图指南](simulation/time_domain_closed_loop/plotting/README.md)。

### 验证与回归

- 总览：[验证与回归目录](tests/README.md)
- 阶段历史：[阶段验证历史](tests/Verification_History.md)
- 10DOF：[Vehicle10DOF 测试](tests/vehicle_10dof/README.md)
- UnifiedControl：[UnifiedControl 测试](tests/unified_control/README.md)
- TorqueVectoring：[TorqueVectoring 测试](tests/torque_vectoring/README.md)
- QuasiStatic：[QuasiStatic 回归测试](tests/quasi_static/README.md)

## 5. 文档导航

### 模型与接口

- [整车模型使用说明](models/README.md)
- [系统架构](models/Architecture.md)
- [坐标系、单位与符号](models/CoordinateSystem.md)
- [信号接口合同](models/SignalInterfaces.md)
- [Adaptive Autocross 驾驶员](models/driver/Adaptive_Autocross_Driver.md)
- [TorqueVectoring 设计与验证](models/controller/Torque_Vectoring.md)
- [UnifiedControl 设计与验证](models/controller/Unified_Control_Allocation.md)

### 数据与参数

- [数据与参数入口](data/README.md)
- [参数来源与不确定性管理](data/ParameterSources.md)
- [赛道数据来源与提取说明](data/TrackData/README.md)
- [第三方材料与受限数据声明](THIRD_PARTY_NOTICES.md)

### 仿真与结果

- [仿真入口使用说明](simulation/README.md)
- [准静态 GGV、圈速与参数扫描](simulation/quasi_static/README.md)
- [时域闭环赛项仿真](simulation/time_domain_closed_loop/README.md)
- [标准结果信号参考](simulation/Result_Signal_Reference.md)

## 6. 重要限制

- 43075 目标胎缺少直接纵向和联合滑移 TTC 数据，相关模型使用明确标记的 43100 R20 代理数据；
- 真实车辆质量、几何、惯量、阻尼和部分控制参数尚未全部确认或标定；
- 10DOF 模型当前适合接口、算法和相对趋势验证，不等同于已标定实车模型；
- 阶段验证中的合成工况、代理赛道和历史结果不能直接作为实车成绩或设计定案；
- 修改 `VehicleData.sldd`、Bus 或模型后，应重新运行受影响的组件、Plant 和闭环回归。

## 7. 许可证与第三方材料

除另有明确说明外，项目原创代码、Simulink 模型和原创文档采用 [Apache License 2.0](LICENSE) 授权。

TTC 轮胎数据、赛事手册图片、车队文件、用户资料和厂商资料不因本项目采用 Apache-2.0 而获得重新授权，具体边界见 [第三方材料声明](THIRD_PARTY_NOTICES.md)。
