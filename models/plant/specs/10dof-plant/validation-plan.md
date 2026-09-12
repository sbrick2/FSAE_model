# Vehicle10DOF 10DOF 动态轮荷 Plant 验证计划与结果

> 最近执行：2026-08-02  
> 自动入口：`tests/vehicle_10dof/runVehicle10DOFVerification.m`

## 1. 验收矩阵

| 验证项 | 方法 | 门限 | 当前结果 |
|---|---|---|---|
| 结构完整性 | 三个 Vehicle10DOF 模型 `model_check` | 无断口/悬空线/Stateflow lint | 通过 |
| 静态平衡 | 零输入 0.2 s | `z,phi,theta` 漂移 `<1e-10` | 通过 |
| 静态载荷 | 四轮载荷求和 | 与 `m*g` 差 `<1e-8 N` | `0 N`，通过 |
| 模态 | 广义特征值 | 三个频率为有限正数 | `4.489/7.032/7.410 Hz` |
| 动态载荷守恒 | 组合 Fx/Fy 阶跃 | 残差 `<1e-6 N` | `4.55e-13 N` |
| 轮荷非负 | 组合、快速转向、制动入弯 | 最小值 `>=0 N` | `470.762/477.235/522.985 N`，通过 |
| 平面回归 | 相同输入、1 ms 固定步长 | 前 14 路最大差 `<1e-8` | `0` |
| 持久化行为测试 | `Vehicle10DOF.feature` | 1 场景全部断言通过 | 1/1 场景、4/4 断言 |
| Vehicle10DOF 闭环冒烟 | Skidpad，1 s | 有限且轮荷非负 | 最小轮荷 `475.468 N`，通过 |

## 2. 工况定义

- 组合符号工况：`Fx=+600 N`、`Fy=+800 N`，检查正侧倾和正纵向加速度产生负俯仰（车头抬起）。
- 快速转向代理：`Fx=0`、`Fy=+1200 N` 阶跃。
- 制动入弯代理：`Fx=-1600 N`、`Fy=+1000 N` 阶跃。
- 7DOF/10DOF 回归：相同输入、相同初始状态、相同 1 ms `ode4` 网格。
- 闭环冒烟：TorqueVectoring Skidpad 驾驶员/控制器接 Vehicle10DOF Plant，运行 1 s。

上述代理工况用于方程符号、载荷守恒和数值稳定性验证，不代表特定实车操纵事件的标定输入。

## 3. 复现命令

```matlab
initProject();
installE41VehicleParameters();
installVehicle10DOFSuspensionData();
report = runVehicle10DOFVerification(RunClosedLoop=true, SaveSummary=true);
```

持久化组件测试使用 `tests/vehicle_10dof/Vehicle10DOF.feature`。输出保存到 `tests/vehicle_10dof/results/Vehicle10DOFVerificationSummary.mat`，并生成平面回归与四轮动态载荷 MATLAB FIG 文件。

## 4. 剩余验证

实车校准关闭前仍需：惯量证据、实际阻尼器设置、K&C 数值曲线、求解器容差/步长扫描、长时 Autocross/Endurance 回归，以及真实快速转向和制动入弯日志对比。
