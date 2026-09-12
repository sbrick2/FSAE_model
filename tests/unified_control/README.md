# UnifiedControl 测试

- `TorqueAllocator.feature`：统一分配器完整编译测试，覆盖标称分配、横摆符号、
  单轮 TC、驱动功率与能量预算、多约束组合、再生/摩擦融合、测量降级和安全禁用。
- `TorqueAllocatorTestWrapper.slx`：仅用于 Gherkin 的标量端口包装，模型说明标记为
  `TEST_ONLY_NOT_FOR_PRODUCTION`；生产模型不引用它。
- `runUnifiedControlFunctionVerification.m`：估计器、投影器、执行器融合和状态恢复的
  纯函数回归；额外覆盖 TC 滞回、转矩变化率和轮荷守恒。
- `runUnifiedControlVerification.m`：UnifiedControl 顶层 10DOF MIL 冒烟与组合场景汇总。

`TorqueAllocator.feature` 含 boolean、uint 和向量端口，草稿 harness 不能代表实际
数据类型；最终验收必须使用 `draft_mode=false`。生产模型不得引用测试资产。

验证汇总默认写入 `tests/unified_control/results/`。
