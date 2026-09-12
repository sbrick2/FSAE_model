# 第三方材料与受限数据声明

根目录 [`LICENSE`](LICENSE) 中的 Apache License 2.0 仅适用于项目有权授权且未
另行注明的原创材料，不对第三方材料授予任何权利。第三方材料始终以其权利人、
合同或来源文件规定的条款为准。

## FSAE Tire Test Consortium 数据

本项目的本地数据和参数可能包含 Formula SAE Tire Test Consortium（FSAE TTC）
与 Calspan Tire Research Facility（TIRF）的试验数据、拟合结果或派生参数。
这些内容受 FSAE TTC Participant Agreement 约束，不因存放在本项目中而获得
重新授权。

- TTC 原始数据不得上传到公开仓库或转交未获授权的个人、车队、学校或组织。
- 不得将 TTC 数据用于商业用途或车队授权范围以外的用途。
- 对外展示或发表前，必须重新核对当前 Participant Agreement，并满足其中关于
  去标识化、呈现形式和致谢的要求。
- `data/` 下的本地数据文件由 `.gitignore` 排除；不得绕过该规则提交受限数据。

## FSEC 赛事手册与赛道图片

`scenarios/Endurance/assets/2024_fsec_endurance_track.png` 来源于 2024 FSEC
竞赛手册。来源和校验信息记录在 `data/TrackData/README.md`。该图片及原始手册
不受 Apache-2.0 重新授权；对外复制、发布或创建公开发行包前，应取得必要许可
或用具有明确授权的自有素材替换。

## 车队、用户及厂商资料

项目参数可能来自车队内部文件、用户提供文档、厂商数据表、测功机曲线以及由这些
资料数字化或推导出的结果。包括但不限于 E41 车辆参数和 Öhlins TTX25 相关资料。
这些内容仅限其原授权范围使用；公开、转交或商业使用前必须取得相应权利人的许可。

## MathWorks 软件与商标

MATLAB、Simulink 及相关产品和商标属于 MathWorks。Apache-2.0 不包含
MathWorks 软件，也不替代运行本项目所需的有效产品许可证。
