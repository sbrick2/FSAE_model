# TorqueVectoring 测试

- `YawController.feature`：参考横摆、低速/附着边界、反馈极性、限幅和禁用复位。
- `TorqueVectoringAllocator.feature`：TV0 等分、TV1 差动、单轮限制、功率限制和摩擦制动。
- `TorqueVectoringVehicleControllerBrakeBlend.feature`：验证 SOC 上限门控；高 SOC 禁用再生，
  低 SOC 保留再生。与 `TorqueVectoringAllocator.feature` 的摩擦接管场景组合覆盖完整链路。
- `TorqueVectoringAllocatorTestWrapper.slx`：仅用于 Gherkin 标量端口测试，模型描述已标记
  `TEST_ONLY_NOT_FOR_PRODUCTION`。
- `runTorqueVectoringVerification.m`：TV0/TV1 成对闭环验证入口；其余 `.m` 文件是该入口的
  场景、输入、汇总和对比辅助函数。

测试必须使用完整编译模式运行；草稿 harness 会把逻辑量和向量端口按 double 标量
处理，不能作为该模型的有效结果。

验证汇总默认写入 `tests/torque_vectoring/results/TorqueVectoringVerificationSummary.mat`。
