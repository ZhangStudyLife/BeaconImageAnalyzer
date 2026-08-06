# 张跃哲 50 FPS 等效性能优化实例

本实例对应 `CYT2BL3_Image` 提交 `fcb0e1befe4addf384e84af5142ff75f5a0fc099`，用于 BeaconImageAnalyzer 前后摄 2BL3 固件算法运行。

## 内容边界

- `algorithm/Image`：直接取自该提交的 `project/code/Image`，算法、阈值、运行参数、输出结构和地平线逻辑未修改。
- `algorithm/host`：仅提供桌面编译所需的相机/Flash 桩、计时桩、CM4 DSP 指令等价仿真和上位机结果转换。
- 固件算法构建标识保持为 `0x20260906`。

## 来源校验

| 文件 | 提交中的 Git blob |
| --- | --- |
| `image.c` | `b028786d39369d29f618505004c7106d1345a7e3` |
| `image.h` | `f5cf8f6f8c7adeb8308d6ca9e5e76c5b5e96dc36` |
| `image_data.h` | `9596aff83ffeded45d80881555e0ba569d70a695` |
| `image_horizon.c` | `81a7b76c3790249b8388730132e02f0f7d78d119` |
| `image_horizon.h` | `3db9b6bce27cb2fe9c0ecb1dcf63fe819918b7ba` |
| `image_params.c` | `7a0b7d026f5c42693f55087905828199091f5925` |

`image.h` 在原提交中混用了 LF/CRLF；实例统一为 LF，文本内容与预处理结果不变。

## 使用

在 BeaconImageAnalyzer 中选择“加载实例”，选中本目录即可。上位机会根据 `two_bl3_instance.json` 编译并加载独立诊断库。
