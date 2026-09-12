# Vehicle10DOF 10DOF 动态轮荷 Plant 系统规格

> 状态：功能基线已实现，实车校准待完成  
> 保真度：L3 解析刚体/查表阻尼基线  
> 适用环境：MATLAB / Simulink R2026a

## 1. 目标与边界

Vehicle10DOF 在既有 7DOF 平面模型的 3 个车身自由度和 4 个车轮转动自由度上，增加簧载质量垂向、侧倾和俯仰 3 个连续自由度。总计 10DOF，不包含四个非簧载质量垂向自由度，也不等同于 14DOF 模型。

交付模型为 `Vehicle10DOF.slx`、`VehiclePlant10DOF.slx` 和 `FSAE_Vehicle10DOF_ClosedLoop.slx`。原 TorqueVectoring/7DOF 文件保持独立，以便回归比较。

## 2. 坐标、状态与角点顺序

- 车身 `+x` 向前、`+y` 向左、`+z` 向上；
- 正侧倾角表示左侧车身抬高；
- 正俯仰角表示车头下沉；
- 四角顺序固定为 `[FL, FR, RL, RR]`；
- 法向载荷 `Fz` 是轮胎所受向上载荷的非负幅值。

Vehicle10DOF 新增状态为 `q=[z, phi, theta]` 和 `qdot=[zdot, phidot, thetadot]`。小角度下，第 `i` 个车身角点位移为：

```text
z_body_i = z + y_i*phi - x_i*theta
delta_i  = z_road_i - z_body_i
```

其中 `x=[a,a,-b,-b]`，`y=[tf/2,-tf/2,tr/2,-tr/2]`。

## 3. 动力学与悬架力

轮端增量悬架力由三部分组成：

```text
DeltaF_i = k_wheel_i*delta_i + F_damper_i/motionRatio_i + F_ARB_i
Fz_i     = max(0, Fz_static_i + DeltaF_i)
```

阻尼器轴速度为轮端相对速度除以表中“车轮位移/弹簧位移”杠杆比。压缩和回弹分别使用一维查表，零速度强制为零力，避免把测功机气压偏置当作静态预载。

新增刚体方程为：

```text
m*zddot       = sum(Fz) - m*g - Df - Dr
Ixx*phiddot   = sum(y_i*Fz_i) + m*Ay*hCG
Iyy*thetaddot = -sum(x_i*Fz_i) + a*Df - b*Dr - m*Ax*hCG
```

前后防倾杆的有效附加侧倾刚度由参数表“总侧倾刚度减弹簧侧倾刚度”取得，并转换为 `N*m/rad`。表中单列防倾杆数值与该差值存在 8 倍关系，保留为待确认源数据解释，不反向修改原表字段。

## 4. 参数证据优先级

1. `E41悬架转向参数表 .docx` 的直接表格数值；
2. `_TTX25 MkII Dyno N vs mmps.pdf` 的 C12/R12 测功机曲线；
3. 从上述数据可复算的派生量；
4. 无直接证据的暂估量。

Vehicle10DOF 使用参数表直接给出的前/后单轮悬架线刚度 `76.74/60.64 kN/m`。虽然弹簧刚度与杠杆比会导出不同结果，但直接线刚度能准确复算表内弹簧侧倾刚度，因此按用户要求优先采用表值。

阻尼器当前基线为 C12/R12 页的 `10-4.3-10-4.3` 曲线。资料没有说明实车旋钮位置，所以 `DamperSettingConfirmed=false`；该选择只表示一条可追踪、可替换的测量曲线。

## 5. 接口

新增输入为前/后轴下压力、四轮路面高度和四轮路面垂向速度；新增输出为侧倾角、俯仰角、垂向位置、垂向加速度、四轮法向载荷、外倾角、悬架挠度、阻尼器速度、悬架力及三个状态速度。

现有前 14 个 `Vehicle7DOF` 输入/输出语义保持不变。模型引用采用 Normal 仿真模式，因为当前悬架核心是 Level-2 MATLAB S-function，尚未提供 TLC/代码生成实现。

## 6. 明确限制

- `Ixx=50 kg*m^2`、`Iyy=80 kg*m^2` 和既有 `Izz=120 kg*m^2` 均为待 CAD/试验替换的暂估值；
- 参数表中的 K&C 曲线没有数值点，当前外倾角输出为零，接口已保留；
- 轮胎离地时只执行 `Fz>=0` 下限，不包含接触事件、非簧载质量或限位块动力学；
- 路面速度目前由 Plant 对路面高度求导，阶跃路面输入应改用平滑剖面；
- 当前阻尼曲线来自 PDF 数字化，记录的不确定度为约 `8 N`；
- 通过结果证明实现一致性和数值可运行性，不构成实车相关性结论。

## 7. 参考

- MathWorks [Automotive Suspension](https://www.mathworks.com/help/simulink/slref/automotive-suspension.html)：悬架力与车辆纵/横/垂向运动耦合的参考边界。
- MathWorks [Passenger Vehicle Dynamics Models](https://www.mathworks.com/help/vdynblks/ug/passenger-vehicle-dynamics-models.html)：车辆动力学模型分层和自由度范围参考。

