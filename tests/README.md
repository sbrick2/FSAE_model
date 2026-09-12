# 验证与回归目录

所有验证入口、测试资产和验证专用辅助函数统一放在 `tests/`，按里程碑或功能
划分子目录。生产算法、模型构建脚本和用户分析入口继续保存在原功能目录。

ProjectFoundation–QuasiStatic 的阶段验收结论集中保存在 [阶段验证历史](Verification_History.md)。TorqueVectoring–UnifiedControl
的复现入口和生成结果说明见对应里程碑目录。

| 子目录 | 范围 |
|---|---|
| `battery/` | 电池包行为验证 |
| `integration/` | 跨模型集成与 Variant 选择 |
| `path_tracking/` | 路径和场景回归 |
| [`quasi_static/`](quasi_static/README.md) | GGV、圈速和时域交叉验证 |
| [`torque_vectoring/`](torque_vectoring/README.md) | 基础扭矩矢量与成对场景验证 |
| [`vehicle_10dof/`](vehicle_10dof/README.md) | 10DOF 动态轮荷验证 |
| [`unified_control/`](unified_control/README.md) | 统一控制分配验证 |

每个里程碑的生成结果保存在本里程碑的 `results/` 中。根目录 `results/` 仅保存
对整车开发有复用价值的圈速、GGV、参数扫描和用户分析结果。
