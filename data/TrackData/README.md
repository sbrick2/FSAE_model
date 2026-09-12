# Autocross / Endurance 赛道来源与提取说明

## 当前数据链路

- 默认原图：`scenarios/Endurance/assets/2024_fsec_endurance_track.png`
- 官方出处：[2024 FSEC竞赛手册Handbook](https://img.sae-china.org/web/2024/10/2024FSEC%E7%AB%9E%E8%B5%9B%E6%89%8B%E5%86%8CHandbook.pdf)
- 原图 SHA-256：`C769CEBF8EAE4CAA92D0BE26629C7665D393E9EADDB17C0314140CC57E7A0531`
- 专用提取入口：`scripts/track/extractFsecEnduranceTrack.m`
- 通用图像提取器：`scripts/track/extractTrackFromImage.m`
- Endurance 场景入口：`scenarios/Endurance/createPathTrackingEnduranceScenario.m`
- Autocross 场景入口：`scenarios/Autocross/createPathTrackingAutocrossScenario.m`

Endurance 使用手册示意图提取的中心线。Autocross 为保持现有模型和回归工况的
共享路线合同，暂时复用同一几何，但元数据明确标记为仿真代理；它不是官方
Autocross 赛道图。

## 已替换的旧来源

旧实现优先读取本机 `data/TrackData/autocross_track_map.csv`，Endurance 再调用
Autocross 场景复用该 CSV。CSV 被 `.gitignore` 排除，生成它的根目录
`track_map.png` 已不存在且从未进入 Git，因此旧几何不可追溯、也无法从仓库复现。
新实现不再读取该 CSV。

## 提取约定

- 图像 ROI：`[85, 165, 1140, 205]` 像素，格式为 `[x, y, width, height]`
- 起点：`[124, 278]` 像素
- 初始行驶方向：图像向下，随后统一校验为逆时针
- 灰度范围：`[90, 210] / 255`
- 色度容差：`15 / 255`
- 仅三个蓝旗超车区参与蓝色遮挡补全，排除 LAP 标牌和旗帜
- 红色虚线绕桩段由图中像素位置形成固定补充路径点并连续化
- 骨架化后删除所有端点支路，只保留单连通闭环
- 平滑距离：`8 m`
- 提取后对图中左上、左下、右上三个连接区域分别做局部高斯平顺化，并用
  二阶连续的 smootherstep 权重渐变回原中心线；终点前尖角使用同类处理，
  未标记的弯道不做额外全局平滑
- 起终点接缝前后 `16 m` 固定为 `X=0` 的竖直直线，出发方向沿 `-Y`，
  在 `42 m` 内渐变回原中心线
- 圈长：归一化为 `1000 m`
- 车道宽度：`3 m` 仿真假设；示意图没有提供可测量比例尺或连续边界宽度

由于来源是赛事说明示意图而非测绘图，提取结果适合 MIL/SIL 路径跟踪、能耗和
控制策略回归，不应作为真实赛场坐标或绝对圈速结论。
