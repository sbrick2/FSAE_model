# QuasiStatic 回归测试

验证入口和专用辅助函数均位于本目录：

- `runQuasiStaticVerification.m`：合成参数、GGV、圈速和扫描回归；
- `runQuasiStaticTimeDomainCrossValidation.m`：PathTracking 时域结果与 QuasiStatic GGV 交叉验证；
- `AdaptiveAutocrossQualificationTest.m`：自适应驾驶员的闭合赛道原始 GGV 圈时比、
  开放赛道资格分流、GGV 上限使用和全程硬门槛；
- `AdaptiveRacingLineTest.m`：宽度约束最小曲率赛车线的求解、边界、缩短效果、
  高曲率软惩罚、物理中心线保持和开放赛道非周期端点；
- `AdaptiveAutocrossDriverStepTest.m`：双路径转向影响、高曲率回退、直线退出前馈、
  开放赛道终点预瞄不回绕和终端参考速度；
- `createQuasiStaticVerificationParameters.m`、`crossValidateQuasiStaticWithTimeDomain.m`：验证专用辅助函数。

`QuasiStaticRegressionTest.m` 覆盖：

- 前后轴相对质心的横摆力臂；
- 开放赛道初末速度与距离合同；
- 左/右弯分别使用对应 GGV 横向边界；
- 时域信号的纵向和横向 GGV 越界判定；
- 参数扫描的软件、Git、工况、单位和来源追踪。

`QuasiStaticRobustnessTest.m` 额外覆盖：

- 超持续车速时 `ax=0` 不可行但自然滑行/制动仍可行的 GGV；
- 纵向搜索初值不足时的自适应边界扩展；
- 全零速度历程的无效判定；
- `0.1 m/s` 等非整数扫参步长；
- 八字绕环左右第二计时圈平均值。

运行：

```matlab
results = runtests(["tests/quasi_static/LapReferenceSpeedTest.m", ...
    "tests/quasi_static/AdaptiveAutocrossQualificationTest.m", ...
    "tests/quasi_static/AdaptiveRacingLineTest.m"]);
assertSuccess(results);
```

验证汇总默认写入 `tests/quasi_static/results/`。
