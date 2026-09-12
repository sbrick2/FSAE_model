# BatteryMaps 电池箱模型

`models/components/BatteryPackModel.slx` 是根据 `data/BatteryMaps` 构建的独立电池箱组件。它不替换或自动接入现有 `VehiclePlant` 中的 `BatteryModel.slx`，以避免在未冻结新接口前改变整车回归基线。

## 数据与结构

- 电芯：INR21700-RS50，5 Ah，2.5/3.6/4.2 V。
- 电池箱：141s3p，共 423 个电芯；六段为 `[23 24 23 24 23 24]s3p`。
- 容量：15 Ah；额定能量 27.4104 MJ（7.614 kWh）；最高电压算术能量 31.9788 MJ（8.883 kWh）。
- 电流：放电 150 A，由 ESF 连接器额定值限制；充电 45 A，由 15 A/单体 × 3p 得到。
- 电阻：0.188 Ohm，为规格书 4 mOhm、1 kHz ACIR 上限的 141s3p 缩放值，只作保守一阶代理。
- 主熔断器额定值 60 A 仅记录在数据字典中；没有时间-电流曲线时不把它当作瞬时 60 A 限流器。

参数安装入口为：

```matlab
installBatteryMapData
```

脚本会读取工作簿中的精确单元格、验证总电芯数和电压一致性，并更新 `data/VehicleData.sldd` 的 `Battery` 记录。

## 接口

输入 `RequestedElectricalPower` 使用正值放电、负值再生。输出依次为允许功率、功率比例、电池电流、SOC、驱动/再生裁剪标志、端电压、单体电压、开路电压和电阻损耗。所有根端口均显式为标量。

## 方程与限制

三点占位 OCV 表定义为：

```text
SOC        = [0, 0.5, 1]
Cell OCV   = [2.5, 3.6, 4.2] V
Pack OCV   = 141 * Cell OCV
```

放电功率上限取 80 kW 与 `I_dis * (Voc - I_dis*R)` 的较小值；再生上限取 10 kW 与 `I_chg * (Voc + I_chg*R)` 的较小值。SOC 超过上下保护边界时阻断相应方向功率。

允许功率满足 `P = I * (Voc - I*R)`，模型选取连续于 `R -> 0` 的稳定根：

```text
I     = 2P / (Voc + sqrt(Voc^2 - 4RP))
Vterm = Voc - I*R
Ploss = I^2*R
dSOC/dt = -I / (3600*Q_Ah)
```

## 验证与待补数据

`tests/battery/BatteryPackModel.feature` 覆盖额定放电、过功率裁剪、满 SOC 禁止再生、先放电后再生和零请求。完整编译验证为 5/5 场景、26/26 断言通过，模型结构检查无未连接端口或悬空信号。

当前模型没有热状态、RC 极化、滞回、温度/倍率依赖、老化、单体不一致、均衡、接触器/预充或熔断器 I²t。BatteryMaps 也没有提供可用于辨识这些状态的完整数据。下一步应优先补充实测 OCV-SOC 曲线和不同 SOC/温度下的直流脉冲内阻，再决定是否升级为一阶或二阶等效电路。
