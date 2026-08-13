# 0813 100 Hz 下摄实例

该实例来自 `CYT4BB7_Air` 提交
`b839e292218e94af5e0d2f1c8e51d000a60a319f`，用于在
BeaconImageAnalyzer 中离线运行该节点对应的下摄算法。

## 板端快照

以下核心文件直接取自指定提交，未修改检测参数、分类规则、时序跟踪、
坐标约定或下摄闭合边界模型：

- `project/code/Estimation/Pos_Est/image_down.c` -> `algorithm/Image/image.c`
- `project/code/Estimation/Pos_Est/image_down.h` -> `algorithm/Image/image_down.h`
- `project/code/Image/image_data.h`
- `project/code/Image/image_down_horizon.c`
- `project/code/Image/image_down_horizon.h`

该板端节点没有独立算法 Build ID。桌面实例使用由提交哈希派生的
`0xB839E292`，仅用于上位机实例识别和参数快照匹配。板端的 22 项
`0x0300` 至 `0x0315` 参数由 `image_params.c` 原样桥接到
`image_down_remote_param_execute()`；默认值和范围仍由板端源码决定。
摄像头曝光默认值为该提交驱动中的 `400`，屏幕模式默认为
`IMAGE_DEBUG_SCREEN_MODE_DATA`。

## 上位机适配

`algorithm/host` 只提供板端硬件环境的桌面实现和 BeaconImageAnalyzer 动态
接口。每个 `beacon_image_process()` 先更新下摄地平线，再发布且只处理一个
新的摄像头帧。结果读取自 `image_data[Center]`，交给公共渲染器时只转换 X
轴方向。

适配器导出 B0/B1、CAR0、信标二值图、车灯面积、上下地平线、闭合区域和
处理帧计数；核心算法仍保持板端的 188x120 图像、参数和时序语义。

## 使用

在 BeaconImageAnalyzer 中加载本目录 `下摄`。为显示闭合边界，需要图像帧
同时提供有效 Roll、Pitch 和高度遥测；遥测无效时算法行为与板端一致。
