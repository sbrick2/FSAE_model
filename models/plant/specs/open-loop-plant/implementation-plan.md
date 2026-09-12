# SlipKinematics–BatteryPowerBaseline 实施计划

> 执行状态：步骤 1–7 已完成；OpenLoopPlantIntegration 集成回归通过。

## 1. 构建顺序

1. 扩展 `VehicleData.sldd` 参数记录和电池初始状态；
2. 构建 `WheelSlipKinematics`，冻结 SlipKinematics→SimpleTire 接口；
3. 构建 `TireSimple` 和 `LoadTransferModel`；
4. 构建 `AeroModel`；
5. 构建 `BatteryPowerLimiter`、`BatteryModel` 和 `PowertrainModel`；
6. 分别运行结构检查和解析仿真；
7. 在 OpenLoopPlantIntegration 接入 `VehiclePlant`，按既有顶层 Bus 合同完成映射、诊断输出收口和开环回归。

## 2. 主要参数

| 组 | 参数 | 单位 | 当前来源 |
|---|---|---|---|
| Tire | `LongitudinalStiffness` | N | `NaN`，待 TTC/标定 |
| Tire | `LateralStiffness` | N/rad | `NaN`，待 TTC/标定 |
| Tire | `CamberStiffness` | N/rad | `NaN`，待 TTC/标定 |
| Tire | `ForceEpsilon` | N | 设计正则化占位 |
| Aero | `CdA`, `ClAFront`, `ClARear` | m² | `NaN`，待 CFD/试验 |
| Aero | `SpeedEpsilon` | m/s | 设计正则化占位 |
| Powertrain | `InverterEfficiency` | 1 | `NaN`，待地图/台架 |
| Battery | `SOCLowerLimit`, `SOCUpperLimit` | 1 | 设计边界，待确认 |
| Battery | `PowerEpsilon` | W | 设计正则化占位 |

## 3. 同步门禁

- 每个 `.slx` 创建前执行 `model_read`；结构只通过 `model_edit` 修改；
- 每个模型完成后执行 `model_read` 和 `model_check`；
- 合成参数不得写回生产记录；
- 测试包装模型若保留，必须位于 `tests/` 并带 `TEST_ONLY_NOT_FOR_PRODUCTION` 标注。
