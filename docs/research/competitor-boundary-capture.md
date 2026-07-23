# 竞品边界采集调研：追觅与九号

调研日期：2026-07-22

范围：仅核对追觅（Dreame）和九号/Segway Navimow 的官方帮助中心、官方更新说明和官方产品支持页面。重点关注边界采集、闭合、自交反馈、草稿和重新采集；公开资料没有披露内部数据结构，因此不能从 UI 文案推断它们是否保存 raw 轨迹。

## 结论先行

- 追觅 A1 的公开流程是“遥控机器人沿草坪边缘行驶，App 实时显示轮廓，接近起点后点击 `Close outline` 完成闭合”。官方资料没有说明自交检测、交点标记、草稿状态或局部重新采集。
- 追觅 A1 的禁区支持两种方式：遥控沿禁区边缘采集后关闭轮廓，或在 App 2D 视图中手动画/调整。这个“可编辑禁区”不能等同于“作业边界自交时能自动修复”。
- 九号 Navimow 新型号同时存在自动建图和手动遥控建图。i2 LiDAR Pro 的官方说明写明地图实时扩展并最终形成闭环；i2 AWD 的官方说明写明可以切换手动建图。
- 九号 i2 LiDAR Pro V4.1 官方更新说明明确支持 App 中直接重画边界，不需要继续遥控机器人；i1 V4.2 更新说明还提到修复“无法拆分边界”的问题。
- 截至本次核查，没有在两家的公开官方资料中找到“保留草稿 + 在地图上标出自交点 + 允许局部重新采集”这一完整契约。它们确实公开了重画、调整和拆分边界能力，但没有公开自交诊断语义。

## 追觅（Dreame）

### A1：遥控采集闭合轮廓

官方帮助文档 [How to create a map outline for the Dreame A1?](https://support.dreametech.com/hc/en-us/articles/13219226930831-How-to-create-a-map-outline-for-the-Dreame-A1) 给出的步骤是：

1. 在 App 中开始创建地图，A1 自动检查和校准。
2. 遥控 A1 到草坪边缘，并点击 `Set starting point`。
3. 从起点遥控 A1 沿草坪边缘行驶，App 同步显示地图轮廓。
4. 接近起点后点击 `Close outline`，App 自动完成轮廓。

官方文档没有写出“如果轮廓自交怎么办”，也没有写出交点坐标、错误状态或自动修复规则。因此不能据此断言 Dreame 会接受自交，更不能断言它会保留一个可规划的自交多边形。

### A1：禁区可以遥控采集，也可以 App 手画

官方帮助文档 [How do I set up virtual no-go zones for the Dreame A1?](https://support.dreametech.com/hc/en-us/articles/13219273941903-How-do-I-set-up-virtual-no-go-zones-for-the-Dreame-A1) 写明：可以遥控机器人绕禁区行驶，接近起点后点击 `Close outline` 并确认；也可以使用 App 的手动绘制功能。文档同时说明手动画/调整禁区只支持 2D 视图。

这说明 Dreame 的公开 UX 有“重新调整几何”的入口，但没有说明作业边界采集失败时是否同样可局部编辑。

### A3：当前官方页面公开到的粒度

官方 [Dreame A3 AWD Series setup walkthrough](https://support.dreametech.com/hc/en-us/articles/15794098815759-Dreame-A3-AWD-Series-Follow-our-step-by-step-walkthrough-to-set-up-your-Robot-Lawn-Mower) 将流程列为 `Map Your Garden`、`Set No-Go Zone`、`Add or Expand Zones` 等视频章节，但页面正文没有公开自交检测、草稿状态或失败恢复的文字契约。A1 的上述结论不能未经验证直接推广到 A3 的具体实现。

## 九号 / Segway Navimow

### 自动建图与手动建图并存

官方 [i2 LiDAR Pro Auto Mapping](https://navimow-support.zendesk.com/hc/en-us/articles/55977819254937--Navimow-i2-Lidar-Pro-What-is-Auto-Mapping-and-How-does-it-work) 说明：App 中点击 `Create Map` 后，机器人探索并识别外边界，地图实时扩展，最终形成闭环；孤立区域需要手动搬运机器人后手动创建。

官方 [i2 AWD Auto Mapping](https://navimow-support.zendesk.com/hc/en-us/articles/54622951167769--Navimow-i2-AWD-What-is-Auto-Mapping-and-How-does-it-work) 说明：可以用 VisionFence 自动识别边界，并在夜间改用手动建图。官方 [i2 AWD mapping distance](https://navimow-support.zendesk.com/hc/en-us/articles/54622930690457--Navimow-i2-AWD-How-far-should-I-stay-behind-the-Navimow-i2-AWD-during-mapping) 进一步说明，手动建图时用户需在约 6 米内保持 Bluetooth 控制；这与“手机遥控机器人采集边界”的模型一致。

### App 直接重画边界

官方 [Navimow i2 LiDAR Pro V4.1.0 release note](https://navimow-support.zendesk.com/hc/en-us/articles/56270051594777-Navimow-i2-LiDAR-Pro-V4-1-0-Firmware-App-Release-Note) 明确写有 `Redraw Boundary`：使用 real-scene map 作为参考，在 App 中直接编辑草坪边界，不需要手动控制机器人，入口为 `Home > Edit > Adjust by drawing`。

官方 [Navimow i1 V4.2.0 release note](https://navimow-support.zendesk.com/hc/en-us/articles/59560944768153--Navimow-i1-Series-V4-2-0-Firmware-App-Release-Notes-updated-2026-6) 还记录了修复“some users could not split the boundary”。这能证明它们提供边界编辑/拆分能力，但仍不能证明它们会把自交点展示给用户。

## 对当前项目的产品含义

1. 竞品公开资料支持“采集后可编辑/重画”这个方向，但没有证据支持“端点闭合就允许自交边界进入规划”。
2. 当前项目继续保留自交 fail-closed 是合理的；为了达到竞品的可恢复体验，可以增加交点可视化、局部重新采集或 App 手动修正，但有效几何仍必须重新通过拓扑校验。
3. 不应把竞品的“重画边界”实现成 `buffer(0)` 或全局提高简化容差；它们解决的是用户修正入口，不是让无效多边形自动变有效。
