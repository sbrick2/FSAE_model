# 轮胎特性分析

本目录保存不依赖整车仿真结果的轮胎模型绘图入口。

运行 `plotting/plotTireModelCharacteristics.m` 可以分析纯滑移、轮荷、外倾角和联合滑移特性。输入数据来自 `data/TireData/`，生成图默认写入 `results/tire/characteristics/`。

轮胎数据来源和参数可信度见 [`data/ParameterSources.md`](../../data/ParameterSources.md)。TireModel 数据处理入口 `scripts/tire/runTirePipeline.m` 生成的报告写入 `data/TireData/TTC_Tire_Model_Report.md`。

完整运行选项见上级 [仿真入口使用说明](../README.md)。
