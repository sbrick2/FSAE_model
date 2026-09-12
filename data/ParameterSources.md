# FSAE 参数来源与不确定性管理

> 状态：E41 悬架转向参数表 + Vehicle10DOF TTX25 阻尼 + UnifiedControl 控制分配参数同步版
> 更新日期：2026-08-02
> 参数容器：`data/VehicleData.sldd`

## 1. 目标

所有车辆、轮胎、气动、动力系统、控制和仿真参数必须：

- 有唯一名称、单位和物理含义；
- 可追踪到实测、CAD、厂商、TTC、CFD、辨识、估计或设计决定；
- 区分真实测量、暂定范围和纯占位值；
- 在参数扫描和结果报告中保留版本与来源；
- 不依赖 Base Workspace 或硬编码块参数。

## 2. 参数记录结构

未知或需追踪的物理参数使用以下字段：

```matlab
Parameter.Value
Parameter.LowerBound
Parameter.UpperBound
Parameter.Unit
Parameter.Source
Parameter.Confidence
Parameter.LastUpdated
Parameter.IsMeasured
Parameter.IsPlaceholder
Parameter.Notes
```

字段规则：

| 字段 | 规则 |
|---|---|
| `Value` | 当前模型使用值；未知且 ProjectFoundation 不参与运算时使用 `NaN` |
| `LowerBound` / `UpperBound` | 无可靠范围时使用 `NaN`，不得凭经验伪造 |
| `Unit` | SI 单位字符串；无量纲使用 `1` |
| `Source` | 采用受控来源类别及具体文件/试验说明 |
| `Confidence` | `High`、`Medium`、`Low`、`Unknown` |
| `LastUpdated` | ISO 日期 `YYYY-MM-DD` |
| `IsMeasured` | 只有实测或直接 CAD 提取才为 `true` |
| `IsPlaceholder` | 暂定或未知值为 `true` |
| `Notes` | 工况、换算、符号、版本和限制 |

## 3. 来源类别

| 类别 | 说明 | 典型证据 |
|---|---|---|
| `Measured` | 实车或台架实测 | 原始数据、设备、日期和校准记录 |
| `CAD` | CAD 质量属性或几何 | CAD 版本、配置和导出文件 |
| `Manufacturer` | 厂商数据 | 数据表、版本、温度/电压工况 |
| `TTC` | FSAE TTC 轮胎数据 | Run 编号、胎压、轮辋、载荷和速度 |
| `CFD` | CFD 或风洞 | 网格、边界条件、姿态和收敛记录 |
| `Identified` | 参数辨识 | 数据集、算法、训练/验证划分和误差 |
| `Estimated` | 工程估计 | 依据、范围和敏感性说明 |
| `Design` | 数值或控制设计决定 | 设计理由和验证结果 |
| `StandardConstant` | 标准物理常数 | 标准来源 |
| `Placeholder` | 仅保证接口或模型可编译 | 必须标记，不用于性能结论 |

## 4. Data Dictionary 分组

```text
Vehicle.*
Tire.*
Aero.*
Powertrain.*
Battery.*
Brake.*
Control.*
Simulation.*
Variant.*
<Bus Objects>
Default<BusName>
```

ProjectFoundation 只创建结构、接口所需参数和少量安全的仿真配置。真实参数在用户数据导入和审计后填入。

## 5. 当前参数清单

### 5.1 车辆

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `Vehicle.Mass` | kg | `320` | E41 参数表；含驾驶员 | UserProvidedDocument |
| `Vehicle.Wheelbase` | m | `1.54` | E41 参数表 `1540 mm` 换算 | UserProvidedDocument |
| `Vehicle.TrackFront` | m | `1.20` | E41 参数表 `1200 mm` 换算 | UserProvidedDocument |
| `Vehicle.TrackRear` | m | `1.16` | E41 参数表 `1160 mm` 换算 | UserProvidedDocument |
| `Vehicle.CGHeight` | m | `0.300` | E41 参数表目标质心高度 | UserProvidedDocument / DesignTarget |
| `Vehicle.CGToFrontAxle` | m | `0.847` | E41 参数表轴荷比前:后 `45:55` 和轴距 `1.54 m` 推导 | DerivedFromE41ParameterTable |
| `Vehicle.InertiaYaw` | kg*m^2 | `120` | 320 kg FSAE 赛车典型初始估计，待 CAD/摆锤试验替换 | TypicalValue |
| `Vehicle.InertiaPitch` | kg*m^2 | `80` | 320 kg FSAE 赛车典型初始估计，待 CAD/试验替换 | TypicalValue |
| `Vehicle.InertiaRoll` | kg*m^2 | `50` | 320 kg FSAE 赛车典型初始估计，待 CAD/试验替换 | TypicalValue |
| `Vehicle.StaticLoadDistributionFront` | 1 | `0.45` | E41 参数表静态前轴载荷占比 | UserProvidedDocument |
| `Vehicle.StaticLoadDistributionRear` | 1 | `0.55` | E41 参数表静态后轴载荷占比 | UserProvidedDocument |

### 5.2 轮胎与车轮

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `Tire.UnloadedRadius` | m | `0.2` | 用户提供的未加载半径 | UserProvided |
| `Tire.EffectiveRadius` | m | `0.2` | 用户提供的有效滚动半径 | UserProvided |
| `Tire.WheelInertia` | kg*m^2 | `0.215490` | 用户提供的轮端等效总转动惯量，包括电机、轮胎、轮辋和减速器 | UserProvided |
| `Tire.VerticalStiffnessPressureGrid` | Pa | `[68947.6, 82737.1, 96526.6]` | 图片中 `10/12/14 psi` 换算 | UserProvidedImage |
| `Tire.VerticalStiffnessLoadGrid` | N | `[889.644, 1334.47, 1779.29]` | 图片中 `200/300/400 lbf` 换算 | UserProvidedImage |
| `Tire.VerticalStiffnessMap` | N/m | `3x3` | 图片中的静态垂向刚度表；行对应气压，列对应轮荷 | UserProvidedImage |
| `Tire.RelaxationLengthLongitudinal` | m | `NaN` | 暂无证据 | Placeholder |
| `Tire.RelaxationLengthLateral` | m | `NaN` | 暂无证据 | Placeholder |
| `Tire.LowSpeedEpsilon` | m/s | `0.1` | Design，数值保护初始占位 | Placeholder |
| `Tire.LongitudinalStiffness` | N | `34808.5` | 43100 R20、7 inch 代理纵向 TTC 拟合曲面在参考工况的局部斜率 | TTCProxyDerived |
| `Tire.LateralStiffness` | N/rad | `30009.7` | 43075 R20、7 inch 横向 TTC 拟合曲面在参考工况的 `-dFy/dalpha` | TTCTargetDerived |
| `Tire.CamberStiffness` | N/rad | `2757.18` | 43075 R20、7 inch 横向 TTC 拟合曲面在参考工况的 `dFy/dgamma` | TTCTargetDerived |
| `Tire.LinearStiffnessReferenceLoad` | N | `780.783` | 所选 TireModel profile 的参考轮荷 | TTCDerived |
| `Tire.LinearStiffnessReferencePressure` | Pa | `82510` | 所选 TireModel profile 的固定参考胎压 | TTCDerived |
| `Tire.LinearStiffnessLoadGrid` | N | `[706.079, 780.783, 862.985]` | 当前前轮静态、TireModel参考和后轮静态单轮轮荷 | Derived |
| `Tire.LongitudinalStiffnessLoadValues` | N | `[31364.4, 34808.5, 38626.1]` | 沿轮荷网格的纵向局部刚度 | TTCProxyDerived |
| `Tire.LateralStiffnessLoadValues` | N/rad | `[27527.8, 30009.7, 32645.4]` | 沿轮荷网格的侧偏局部刚度 | TTCTargetDerived |
| `Tire.CamberStiffnessLoadValues` | N/rad | `[2523.11, 2757.18, 3007.47]` | 沿轮荷网格的外倾局部刚度 | TTCTargetDerived |
| `Tire.ForceEpsilon` | N | `1.0` | Design，力正则化初始占位 | Placeholder |

`Tire.LowSpeedEpsilon` 只用于未来低速算法初始测试，实施时必须做敏感性和连续性检查，不得视为物理测量值。

垂向刚度表的原始条件为 Hoosier 43075 LC0 R20、8 inch 轮辋和
`preload=0`。它不用于替代纵向、侧向或外倾刚度；当前 TireModel 轮胎 profile
使用 7 inch 轮辋，因此使用该垂向表时必须保留轮辋条件差异。

线性刚度由 `deriveTTCLinearStiffness` 对所选 TireModel TTC 拟合曲面做中心差分：
`Ckappa=dFx/dkappa`、`Calpha=-dFy/dalpha`、
`Cgamma=dFy/dgamma`。标量值对应 `Fz=780.783 N`、`P=82.510 kPa`
和零滑移/零外倾参考点，用于 `TireSimple`。43075 没有可传递驱动转矩的
drive/brake 数据，因此纵向刚度仍是 43100 R20 同配方异尺寸代理，不得
描述为目标胎纵向实测。

纵向、横向松弛长度描述轮胎从“运动学滑移已经改变”到“轮胎力建立完成”
所需滚过的距离，而不是刚度。常用一阶近似为
`tau_kappa = L_kappa / max(|Vx|, Veps)`、
`tau_alpha = L_alpha / max(|Vx|, Veps)`：松弛长度越大，同一车速下力响应越慢。
当前 TTC 流水线拟合的是准稳态力曲面，不能仅凭稳态斜率唯一确定松弛长度；
需要保留时间轴的滑移阶跃、扫频或专门瞬态试验并校正台架延迟后辨识。因此
这两个值继续保持 `NaN`，也尚未参与当前代数型 `TireSimple/MF62/TTCMap`
力计算。

#### TireModel 高保真轮胎参数

| 数据/参数 | 当前选择 | 来源 | 标记 |
|---|---|---|---|
| 目标轮胎 | Hoosier 43075 16x7.5-10 R20 | 用户指定 + TTC Round 9 | Confirmed |
| 轮辋宽度 | 7 inch | 用户指定 + TTC run 元数据 | Confirmed |
| 横向与外倾 | 43075 R20 cornering runs | TTC 实测、训练/验证分离 | Identified |
| 纵向与联合滑移 | Hoosier 43100 18.0x6.0-10 R20，7 inch rim | 同 R20 配方异尺寸 TTC 代理 | Identified Proxy |
| `TireSelectedProfile` | `Round9_43075_R20_Rim7` | TireModel 拟合流水线 | Selected |
| `TireSelection.ModelMode` | `1` (`TireMF62`) | 用户选择与 TireModel 集成决定 | Design |
| `TireMap*` / `TireMFParameters` | 数值运行数组 | 从所选 profile 确定性生成 | Derived |

目标胎横向验证集 `Fy` 为 RMSE 164.386 N、R² 0.9884。代理 drive/combined 验证集为 `Fx` RMSE 169.784 N、R² 0.9836，`Fy` RMSE 123.397 N、R² 0.9465。后两项只证明代理数据拟合质量，不代表 43075 纵向精度。

43075 的偏置适配器不能传递驱动转矩，因此当前 TTC 数据没有目标尺寸 drive/brake 测量。`TireDatabase`、所选 profile、Variant 选择和代码生成运行数组均保存在 `VehicleData.sldd`，模型不依赖 Base Workspace。

### 5.3 气动

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `Aero.CdA` | m^2 | `1.4` | 理论值 2.0 × 70% 实际折减 | Estimated |
| `Aero.ClAFront` | m^2 | `2.1504` | 总 ClA(0 deg) 4.48 × 前轴 48% | Estimated |
| `Aero.ClARear` | m^2 | `2.3296` | 总 ClA(0 deg) 4.48 × 后轴 52% | Estimated |
| `Aero.DownforceBalanceFront` | 1 | `0.48` | 用户提供前后气动载荷比 48:52 | UserProvided |
| `Aero.DownforceBalanceRear` | 1 | `0.52` | 用户提供前后气动载荷比 48:52 | UserProvided |
| `Aero.YawAngleGrid` | deg | `[0, 5, 10]` | 用户提供偏航数据点 | UserProvided |
| `Aero.ClATotalYaw` | m^2 | `[4.48, 3.85, 3.71]` | 理论值 `[6.4, 5.5, 5.3]` × 70% | Estimated |
| `Aero.YawDownforceScale` | 1 | `[1, 0.859375, 0.828125]` | 相对 0 deg 总 ClA 的比例 | Estimated |
| `Aero.YawCorrection` | 1 | `0` | 用户假设不同来流偏航角下 `CdA` 不变 | UserAssumption |
| `Aero.SpeedEpsilon` | m/s | `0.1` | Design，方向正则化初始占位 | Placeholder |

`AeroModel` 和 QuasiStatic GGV 求解器均使用 `abs(yaw)` 的分段线性插值；偏航角超出 `0--10 deg` 时保持端点值。当前偏航数据只约束下压力，阻力采用用户指定的 `CdA` 不随偏航角变化假设。

### 5.4 动力系统与电池

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `Powertrain.GearRatio` | 1 | `12.7536` | 用户提供的电机到车轮减速比 | UserProvided |
| `Powertrain.GearEfficiency` | 1 | `0.97` | 单级减速器典型效率假设，待传动台架数据替换 | TypicalValue |
| `Powertrain.MotorTorqueLimit` | N*m | `21.0` | A2370DD 数据手册最大扭矩；三温度地图保留在 `MotorMap` | ManufacturerDatasheet |
| `Powertrain.MotorSpeedLimit` | rad/s | `2094.3951` | A2370DD 数据手册机械最高转速 20,000 rpm 换算 | ManufacturerDatasheet |
| `Powertrain.MotorResponseTime` | s | `0.020` | 电机/逆变器等效响应时间典型假设，待台架或控制器数据替换 | TypicalValue |
| `Powertrain.InverterEfficiency` | 1 | `0.95` | 逆变器平均效率典型假设，不使用电机效率表替代 | TypicalValue |
| `Battery.NominalVoltage` | V | `507.6` | `ESF.xlsx` 电池箱表；141 × 3.6 V 交叉核对 | BatteryMaps |
| `Battery.MaximumVoltage` | V | `592.2` | `ESF.xlsx` 电池箱表；141 × 4.2 V 交叉核对 | BatteryMaps |
| `Battery.MinimumVoltage` | V | `352.5` | 141 × 2.5 V 单体截止电压 | Derived |
| `Battery.Capacity` | J | `31978800` | `ESF.xlsx` 的最高电压算术能量，592.2 V × 15 Ah | BatteryMaps |
| `Battery.NominalEnergy` | J | `27410400` | 507.6 V × 15 Ah | Derived |
| `Battery.SeriesCells` / `ParallelCells` | 1 | `141 / 3` | 六段 `[23 24 23 24 23 24]s`、全部 3p | BatteryMaps |
| `Battery.TotalCells` | 1 | `423` | `ESF.xlsx` 总数，与六段结构交叉核对 | BatteryMaps |
| `Battery.PackCapacityAh` | A*hr | `15` | 三并联 × 5 Ah 单体容量 | Derived |
| `Battery.PowerLimitDrive` | W | `80000` | 用户提供的电池箱输出功率限制 80 kW | UserProvided |
| `Battery.PowerLimitRegen` | W | `10000` | 用户指定的再生制动功率上限 10 kW | UserProvided |
| `Battery.PackDischargeCurrentLimit` | A | `150` | 3p 电芯能力与 ESF 连接器 150 A 额定值取小 | Derived |
| `Battery.PackChargeCurrentLimit` | A | `45` | 规格书 15 A/单体 × 3p | ManufacturerDatasheet |
| `Battery.MainFuseRatedCurrent` | A | `60` | `ESF.xlsx` FWP 700 V 主熔断器额定值；不作为瞬时限流值 | BatteryMaps |
| `Battery.PackResistance` | Ohm | `0.188` | 规格书 4 mOhm/单体、141s3p 缩放 | ConservativeProxy |
| `Battery.MaxCellTemperature` | degC | `80` | 电芯规格书连续放电截止温度 | ManufacturerDatasheet |
| `Battery.InitialSOC` | 1 | `1.0` | 用户提供的初始 SOC 100% | UserProvided |
| `Battery.SOCLowerLimit` | 1 | `0.05` | Design，保护边界初始占位 | Placeholder |
| `Battery.SOCUpperLimit` | 1 | `0.95` | Design，保护边界初始占位 | Placeholder |
| `Battery.PowerEpsilon` | W | `1.0` | Design，比例正则化初始占位 | Placeholder |
| `Battery.OCVSOCBreakpoints` | 1 | `[0, 0.5, 1]` | BatteryMaps 未提供 OCV-SOC 曲线 | Placeholder |
| `Battery.CellOCVValues` | V | `[2.5, 3.6, 4.2]` | 最低/额定/最高电压三点近似 | Placeholder |

已导入 `MotorMap` 数据字典项：A2370DD 在 80/100/120 degC 下的轴端扭矩、电磁转矩和总损耗二维地图，转速轴为 0--20,000 rpm，电流轴为 0--105 Arms。地图极值分别为 22.306、21.835、21.364 N*m；标量动力系统模型仍采用数据手册明确的 21 N*m 上限。减速比使用用户提供值；齿轮效率、逆变器效率和响应时间为低置信度典型值，后续应替换为实测或厂商数据。

`installBatteryMapData` 从 `data/BatteryMaps/ESF.xlsx` 读取电芯、分段、连接器和熔断器数据，并与 INR21700-RS50 规格书交叉核对。`Battery.PackResistance=0.188 Ohm` 是把规格书 1 kHz ACIR 上限缩放到 141s3p 后得到的保守代理，不等价于直流脉冲内阻。BatteryMaps 未给出 OCV-SOC、温升热阻/热容、熔断器时间-电流曲线、老化或极化数据；因此当前三点 OCV 表和 SOC 保护边界均不得作为最终标定。

### 5.5 制动、转向和悬架

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `Brake.MaxPressure` | Pa | `5.0e6` | 用户提供；定义为前油路最大压力 | UserProvided |
| `Brake.MaxPressureRear` | Pa | `4.526801e6` | 固定前后油压比推导 | Derived |
| `Brake.PressureRatioRearToFront` | 1 | `0.9053603` | `3.4439/3.8039`，前后油压固定比例 | Derived |
| `Brake.ReferencePressureFront` | Pa | `3.8039e6` | 用户提供的前油路标定压力 | UserProvided |
| `Brake.ReferencePressureRear` | Pa | `3.4439e6` | 用户提供的后油路标定压力 | UserProvided |
| `Brake.ReferenceForceFront` | N | `1269.7120` | 用户提供的前轮单边制动力，不是力矩 | UserProvided |
| `Brake.ReferenceForceRear` | N | `593.3127` | 用户提供的后轮单边制动力，不是力矩 | UserProvided |
| `Brake.ForcePerPressureFront` | N/Pa | `3.3379216e-4` | 标定点推导的前轮单边制动力增益 | Derived |
| `Brake.ForcePerPressureRear` | N/Pa | `1.7227931e-4` | 标定点推导的后轮单边制动力增益 | Derived |
| `Brake.TorquePerPressureFront` | N*m/Pa | `6.6758432e-5` | 使用有效滚动半径 `0.2 m` 从制动力增益换算 | Derived |
| `Brake.TorquePerPressureRear` | N*m/Pa | `3.4455861e-5` | 使用有效滚动半径 `0.2 m` 从制动力增益换算 | Derived |
| `Brake.MaxTorquePerWheel` | N*m | `[333.7922, 333.7922, 155.9748, 155.9748]` | 最大前压 `5 MPa`、固定后/前压力比、标定增益及 `0.2 m` 半径推导 | Derived |
| `Brake.ForceDistributionFront` | 1 | `0.6815326` | 优化后的前轴制动力分配系数 | Derived |
| `Brake.ForceDistributionRear` | 1 | `0.3184674` | 优化后的后轴制动力分配系数 | Derived |
| `Brake.SynchronousAdhesionCoefficient` | 1 | `1.1885` | 用户提供的同步附着系数 | UserProvided |
| `Vehicle.SteeringRatio` | 1 | `3.5`，范围 `2.5--4.5` | E41 参数表给出可变角位移比；当前标量模型使用区间中点 | UserProvidedDocument / ScalarApproximation |
| `Vehicle.AckermannRate` | 1 | `NaN` | E41 参数表对应数值为空，继续保持未定 | Placeholder |
| `Vehicle.RollCenterHeightFront/Rear` | m | `0.040 / 0.060` | E41 参数表 | UserProvidedDocument |
| `Vehicle.VirtualSwingArmLengthFront/Rear` | m | `1.500 / 1.800` | E41 参数表 | UserProvidedDocument |
| `Vehicle.KingpinInclinationFront/Rear` | deg | `5 / 2` | E41 参数表 | UserProvidedDocument |
| `Vehicle.CasterAngleFront/Rear` | deg | `7 / 0` | E41 参数表 | UserProvidedDocument |
| `Vehicle.ScrubRadiusFront/Rear` | m | `0.035 / 0.045` | E41 参数表 | UserProvidedDocument |
| `Vehicle.MechanicalTrailFront/Rear` | m | `0.040 / 0` | E41 参数表 | UserProvidedDocument |
| `Vehicle.RideFrequencyFront/Rear` | Hz | `3.6 / 3.2` | E41 参数表未写单位，按偏频解释为 Hz | UserProvidedDocument / UnitInterpreted |
| `Vehicle.SuspensionLeverageRatioFront/Rear` | 1 | `0.923 / 0.882` | E41 参数表；定义为车轮位移/弹簧位移 | UserProvidedDocument |
| `Vehicle.SpringRateFront/Rear` | N/m | `32690 / 23590` | E41 参数表 `32.69/23.59 N/mm` 换算 | UserProvidedDocument |
| `Vehicle.WheelRateFront/Rear` | N/m | `76740 / 60640` | E41 参数表直接悬架线刚度；能复算表内弹簧侧倾刚度 | UserProvidedDocument / AuthoritativeForVehicle10DOF |
| `Vehicle.SuspensionLinearStiffnessFront/Rear` | N/m | `76740 / 60640` | E41 参数表悬架线刚度，作为可追踪别名保留 | UserProvidedDocument |
| `Vehicle.SpringRollStiffnessFront/Rear` | N*m/deg | `964.34 / 712.07` | E41 参数表弹簧侧倾刚度 | UserProvidedDocument |
| `Vehicle.SpringRollGradientFront/Rear` | deg/g | `0.45 / 0.62` | E41 参数表弹簧侧倾梯度 | UserProvidedDocument |
| `Vehicle.AntiRollBarBladeStiffness` | N*m/deg | `4415.01` | E41 参数表防倾杆耳片刚度 | UserProvidedDocument |
| `Vehicle.AntiRollBarStiffnessFront/Rear` | N*m/deg | `1.955 / 1.423` | E41 参数表 | UserProvidedDocument |
| `Vehicle.RollStiffnessFront/Rear` | N*m/deg | `979.98 / 723.45` | E41 参数表前后总侧倾刚度 | UserProvidedDocument |
| `Vehicle.RollGradientFront/Rear` | deg/g | `0.45 / 0.61` | E41 参数表前后总侧倾梯度 | UserProvidedDocument |
| `Vehicle.RollStiffnessDistributionFront` | 1 | `0.5752981` | `979.98 / (979.98 + 723.45)` | DerivedFromE41ParameterTable |

E41 参数表同时给出了弹簧刚度、杠杆比和悬架线刚度。按表头
`杠杆比 = 车轮位移 / 弹簧位移`，由 `SpringRate / 杠杆比^2` 会得到
`38.3717/30.3243 kN/m`，与表中直接线刚度 `76.74/60.64 kN/m` 不一致，
且并非小数舍入可以解释。直接线刚度代入 `k*t^2/2` 后能准确复算表内
`964.34/712.07 N*m/deg` 弹簧侧倾刚度，因此按“表中数值优先”原则，Vehicle10DOF
运行参数 `WheelRateFront/Rear` 使用 `76740/60640 N/m`；冲突派生值不入模。
前后轴总侧倾刚度 `979.98/723.45 N*m/deg` 的派生前轴分配为 `0.5752981`；
若用于以 `rad` 表示的侧倾角动态方程，需要先进行单位换算。

#### Vehicle10DOF 阻尼器查表

| 参数 | 当前值 | 来源 | 标记 |
|---|---|---|---|
| 阀系 | `C12/R12` | TTX25 MkII FSAE/QM 资料说明的交付阀系 | ManufacturerDocument |
| 当前查表曲线 | `10-4.3-10-4.3` | `_TTX25 MkII Dyno N vs mmps.pdf` 第 2 页数字化 | ManufacturerDyno / Digitized |
| 速度断点 | `[0,15,25,50,...,250] mm/s` | 同上 | Digitized |
| 压缩力 | `[0,16.3,20.4,38.8,...,324.5] N` | 同上负速度支路的力幅值解释 | Digitized / InferredBranch |
| 回弹力 | `[0,20.4,24.5,53.1,...,436.7] N` | 同上正速度支路的力幅值解释 | Digitized / InferredBranch |
| 数字化不确定度 | `8 N` | 图像读取误差预算 | Estimated |
| 实车旋钮设置确认 | `false` | 参数表和阻尼资料均未给出实车设置 | OpenQuestion |

测功机曲线在零速附近包含滞环/气压偏置；Vehicle10DOF 将零速度点强制为 `0 N`，避免
把该偏置重复计入静态轮荷。正/负速度分支对应回弹/压缩是基于常用测功机图
方向作出的解释，后续应以 Öhlins 原始数值数据或台架导出再次确认。

`VehiclePlant` 已消费 `FrictionBrakeTorqueRequest`：先取四轮非负请求总量，
再用共同液压比例按 `Brake.MaxTorquePerWheel` 分配，以保证整个工作区间内
后/前油压比固定为 `0.9053603`；液压比例限制在 `0--1`，执行器未使能时
输出为零，最终制动力矩始终与各轮轮速方向相反，并从驱动轮端转矩中扣除。

### 5.6 开发期采样时间

以下为 ProjectFoundation 开发合同，不是最终 VCU 周期：

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `Simulation.TsMotor` | s | `0.001` | Design placeholder | Placeholder |
| `Simulation.TsSensor` | s | `0.005` | Design placeholder | Placeholder |
| `Simulation.TsController` | s | `0.005` | Design placeholder | Placeholder |
| `Simulation.TsDriver` | s | `0.010` | Design placeholder | Placeholder |
| `Simulation.TsEnergyManagement` | s | `0.050` | Design placeholder | Placeholder |
| `Simulation.StopTimeSmoke` | s | `1.0` | ProjectFoundation 冒烟测试设计值 | Design |
| `Simulation.Gravity` | m/s^2 | `9.80665` | StandardConstant | Fixed |

### 5.7 TorqueVectoring 基础扭矩矢量控制

TorqueVectoring 使用现有车辆、制动、动力系统与 TireModel TTC/MF6.2 参数，不新增“实测”车辆参数。
控制设计记录保存在 `TorqueVectoringControlDesign`，模型使用同源的 `TorqueVectoring*` 标量别名：

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `TorqueVectoringControlSampleTime` | s | `0.005` | 共享 `Simulation.TsController` | Design / Tentative |
| `TorqueVectoringControlDesign.YawKp.Value` | N*m/(rad/s) | `360` | 合成闭环标定 | Design / Tentative |
| `TorqueVectoringControlDesign.YawKi.Value` | N*m/rad | `90` | 合成闭环标定 | Design / Tentative |
| `TorqueVectoringControlDesign.YawAntiWindupGain.Value` | 1/s | `4` | 防积分饱和设计 | Design / Tentative |
| `TorqueVectoringControlDesign.YawMomentLimit.Value` | N*m | `350` | TorqueVectoring 控制器限幅 | Design / Tentative |
| `TorqueVectoringControlDesign.UndersteerGradient.Value` | s^2/m | `0` | 中性转向初始基线 | Placeholder |
| `TorqueVectoringControlDesign.MinimumControlSpeed.Value` | m/s | `1` | 低速保护阈值 | Design / Tentative |
| `TorqueVectoringControlDesign.FrictionEstimate.Value` | 1 | `1.0` | 当前场景道路附着估计，不等同于 TTC 峰值 | Placeholder |
| `TorqueVectoringControlDesign.LateralAccelerationLimit.Value` | m/s^2 | `8` | 参考横摆稳定边界 | Design / Tentative |
| `TorqueVectoringControlDesign.YawRateLimit.Value` | rad/s | `2.5` | 参考横摆绝对限幅 | Design / Tentative |

所选 TTC/MF 数据在参考轮荷 `780.783 N` 附近的抽样峰值约为
`mu_y=2.4045`，代理纵向约为 `mu_x=2.2425`。现行场景使用
`RoadGripScale=1` 保留参考轮胎能力，并以 `RoadMuLimit=Inf` 默认关闭额外的
绝对摩擦圆限幅；控制器内部的 `1.0` 摩擦估计仅是保守控制设计参考，不再裁剪
轮胎模型输出。质量、轴距、轮距、轮胎半径、减速比、效率、单电机限制、
80 kW 驱动功率、10 kW 再生功率和四轮摩擦制动能力均从既有数据字典记录读取。

### 5.8 UnifiedControl 统一控制分配

UnifiedControl 不新增实测物理参数。`installUnifiedControlData` 从既有 `Vehicle`、`Tire`、
`Powertrain`、`Battery`、`Brake` 和 `Simulation` 记录复制运行别名，并把新控制
标定集中记录在 `UnifiedControlDesign`。下列低可信度值仅用于功能基线：

| 参数 | 单位 | 当前值 | 来源 | 标记 |
|---|---|---:|---|---|
| `UnifiedControlSampleTime` | s | `0.005` | 共享 `Simulation.TsController` | Shared |
| `UnifiedControlMotorTorqueRateLimit` | N*m/s | `1050` | `21 N*m / 0.02 s`，由当前电机限制和响应时间推导 | Derived / Tentative |
| `UnifiedControlFrictionEstimate` | 1 | `1.0` | TorqueVectoring 场景道路附着估计 | Placeholder |
| `UnifiedControlTCSlipOn/Off` | 1 | `0.12 / 0.08` | TC 滞环设计 | Design / Tentative |
| `UnifiedControlTCSlipFullCut` | 1 | `0.25` | TC 完全削减阈值 | Design / Tentative |
| `UnifiedControlMinimumNormalLoad` | N | `50` | 轮胎容量与数值保护 | Design |
| `UnifiedControlEnergyDriveFraction` | 1 | `0.90` | 能量管理驱动功率保留 | Placeholder |
| `UnifiedControlForcePriorityWeight` | 1 | `1.0` | 投影分配器设计 | Design |
| `UnifiedControlYawPriorityWeight` | 1 | `4.0` | 饱和时优先保留横摆权威 | Design |
| `UnifiedControlRegularizationWeight` | 1 | `1e-4` | 平衡分配数值正则 | Design |
| `UnifiedControlAllocatorIterations` | 1 | `24` | 固定执行次数 | Design |

物理限制继续使用共享记录：单电机 `21 N*m`、驱动功率 `80 kW`、再生功率
`10 kW`、SOC 再生上限 `0.95`，以及现有转速、减速比、效率和四轮摩擦制动
能力。安装或刷新参数时运行：

```matlab
addpath(fullfile(project.RootFolder, "scripts", "unified_control"));
installUnifiedControlData();
```

TC 阈值、附着估计、电机转矩变化率和能量保留比例需要实车/台架证据后重新标定；
当前验证结论只覆盖约束执行与数值鲁棒性，不代表实车性能。

## 6. 参数导入流程

1. 保留原始数据文件，不覆盖；
2. 记录文件版本、时间、单位和坐标系；
3. 通过独立导入函数完成单位和符号转换；
4. 在审计报告中标记异常、缺失和外推区域；
5. 将审核后的值写入 `VehicleData.sldd`；
6. 更新本文件和 [`CHANGELOG.md`](../CHANGELOG.md)；
7. 运行受影响组件与整车回归测试。

## 7. 参数扫描记录

每次批量仿真至少保存：

- 参数名、值、单位和来源版本；
- Model Variant；
- 场景和赛道版本；
- 求解器、容差和步长；
- MATLAB/Simulink 版本；
- 模型和脚本版本；
- 关键指标、约束激活和失败原因。

当前目录尚未初始化 Git，ProjectFoundation 报告必须明确写为 `Git commit: unavailable`，不得虚构提交号。

## 8. 变更控制

- 共享参数名、层级或单位变更属于接口变更；
- 已冻结参数不得直接改名，必须先做影响分析；
- 新增参数必须指定来源和默认行为；
- `NaN` 占位值一旦参与计算必须在模型更新或仿真前显式报错；
- 任何经验范围必须标记为 `Estimated` 或 `Placeholder` 并进入敏感性分析。

## 9. ProjectFoundation 验收

- [x] `VehicleData.sldd` 包含本文件规定的顶层结构；
- [x] 未知物理参数未被写成真实数值；
- [x] 所有占位参数带 `IsPlaceholder = true`；
- [x] 标准常量和开发期设计值有明确类别；
- [x] Bus 默认结构与物理参数分离；
- [x] 模型不依赖 Base Workspace；
- [x] 初始化脚本能报告仍为占位的关键参数。
