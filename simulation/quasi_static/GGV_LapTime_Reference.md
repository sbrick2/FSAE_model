# QuasiStatic GGV、准稳态圈速与参数扫描

## 实现范围

QuasiStatic 在不改变 PathTracking 时域闭环模型接口的前提下，新增一条离线准稳态求解链：

```text
VehicleData-style parameters
        ↓
createQuasiStaticConfiguration
        ↓
solveSteadyState
        ↓
generateGGV(speed-dependent surface)
        ↓
calculateSpeedProfile
        ↓
calculateLapTime / parameter sweep
```

核心文件：

- `scripts/ggv/createQuasiStaticConfiguration.m`：将结构化参数转换为数值契约，并拒绝 `NaN` 占位参数；
- `scripts/ggv/solveSteadyState.m`：准静态轮荷、气动、车身力/横摆力矩平衡、轮胎包络和动力系统约束；
- `scripts/ggv/generateGGV.m`：按速度和横向加速度分数自适应搜索纵向加速/制动边界；当 `ax=0` 因最高转速或阻力不可行时，仍保留自然滑行与制动可行区；
- `scripts/lap_time/calculateSpeedProfile.m`：曲率限速、前向加速和后向制动传播；
- `scripts/lap_time/calculateLapTime.m`：按弧长积分圈时；
- `scripts/lap_time/calculateEventMetric.m`：将完整速度历程转换为比赛项目指标；八字绕环取右侧第二圈与左侧第二圈的平均时间；
- `scripts/ggv/createQuasiStaticParameterSweep.m`、`runQuasiStaticParameterSweep.m`：可追踪扫描组合、软件/Git/工况元数据和失败原因；
- `scripts/reporting/plotQuasiStaticResults.m`：GGV 边界、纵向能力面和扫描汇总图；
- `tests/quasi_static/crossValidateQuasiStaticWithTimeDomain.m`：与 PathTracking 归一化时域结果交叉比较；
- `tests/quasi_static/runQuasiStaticVerification.m`：合成参数回归入口；
- `tests/quasi_static/runQuasiStaticTimeDomainCrossValidation.m`：运行真实 PathTracking 仿真并完成 PathTracking→QuasiStatic 慢速集成比较。

## 轮胎和 TTC 数据

默认 `TireModel="MF62"`，使用 TireModel 已选轮胎 profile 的 MF6.2-equivalent steady-state 实现。目标胎 `43075` 的 TTC Map 只有零纵向滑移的横向/外倾数据；因此它适合做横向参考真值，不能单独支撑 GGV 的驱动边界。驱动和联合滑移仍显式使用 TireModel 中标记为代理的 `43100 R20` 配方。

`TireModel="TTCMap"` 保留统一接口，但若请求纵向 GGV，求解器会提示 Map 不包含可用的纵向能力，而不会静默生成虚假的驱动性能。

## 约束和未知量处理

稳态求解包含：

- SAE 车身坐标下的总纵向力、横向力和横摆力矩平衡；
- 前后轴准静态纵向载荷转移；
- 按前后侧倾刚度分配的横向载荷转移；
- 阻力、前后轴下压力和相对风速；
- 四轮 MF/TTC 轮胎力包络；
- 电机转矩—转速、驱动功率、再生功率和摩擦制动边界。

车辆字典中的质量、质心、气动、动力系统等当前仍是占位值。QuasiStatic 不会替换这些占位值；调用 `createQuasiStaticConfiguration` 时必须传入有限数值。`createQuasiStaticVerificationParameters` 中的数值仅用于算法守恒和接口验证，`SyntheticOnly=true`，不代表实车。

## 运行验证

在 MATLAB 中执行：

```matlab
addpath(genpath(pwd));
report = runQuasiStaticVerification();
integrationReport = runQuasiStaticTimeDomainCrossValidation();
testResults = runtests(["tests/quasi_static/QuasiStaticRegressionTest.m", ...
    "tests/quasi_static/QuasiStaticRobustnessTest.m"]);
assertSuccess(testResults);
```

验证入口默认将汇总保存到 `tests/quasi_static/results/`；该目录中的生成结果不进入版本控制。

验证覆盖：

1. LP 稳态点的力/力矩平衡、轮荷守恒和前后轴横摆力臂；
2. 小规模速度相关 GGV；
3. 闭合/开放赛道前后向速度规划、首末速度和圈时积分；
4. 左/右弯分别使用对应的正/负横向边界；
5. QuasiStatic GGV 与合成时域信号的纵向、横向边界比较及越界拒绝；
6. 两组气动阻力参数的扫描、追踪元数据和失败原因保留；
7. PathTracking 加速场景实际仿真输出与 QuasiStatic GGV 的交叉验证。

持久回归位于 `tests/quasi_static/QuasiStaticRegressionTest.m` 和
`tests/quasi_static/QuasiStaticRobustnessTest.m`，覆盖基础接口、零速无效判定、非整数速度步长、
八字绕环计时圈和超持续车速制动包络。

正式性能分析前，应使用用户确认的车辆、气动、电机、制动和电池数据，并将 TTC 代理通道替换或单独标注。
