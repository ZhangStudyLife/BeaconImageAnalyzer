# JustFloat Monitor

独立的三摄 JustFloat UDP 接收、CSV 记录、回放和车模规划可视化程序。

## 显示窗口

- 主窗口只显示 Front、Center、Back 三路原始相机画面。
- “Center 坐标窗口”和“解耦坐标窗口”分别打开两个独立顶层窗口，可单独拖到其他显示器、缩放或最大化。
- 两个坐标窗口手动关闭时只隐藏，不会停止 UDP、记录或回放。
- 坐标视图支持滚轮缩放、拖动平移和复位，目标详情通过鼠标悬停查看。
- UDP 实时和 CSV 回放都会同步更新已打开的坐标窗口。

## 记录方式

- 点击“开始记录”后立即缓存收到的 UDP 帧，并显示红点、时长和帧数。
- 再次点击“停止并保存”后才选择 CSV 保存位置。
- 取消保存会直接丢弃本次记录。

## 示例日志

[`examples/justfloat_20260820_041811_yaw0.csv`](examples/justfloat_20260820_041811_yaw0.csv) 是一次实际飞行时记录的 CarPlan3 日志（64 列历史格式），可用于参考和快速体验上位机。运行程序后点击“导入 CSV”并选择该文件，即可查看数据回放；日志仅代表其中一次飞行，不作为标准调参数据。

## 协议

- 70 个小端 `float32`：I0 是 JustFloat 自动时间戳，I1-I69 是用户数据。
- 无尾标数据包共 280 字节。
- 可选 JustFloat/VOFA 尾标 `00 00 80 7F`，带尾标共 284 字节。
- CSV 固定为 I0-I69 共 70 列，不兼容旧格式。
- I69 `selected_target_id` 是当前帧三个融合信标槽位 0/1/2；`-1` 表示没有选定目标，不是跨帧永久 ID。
- 默认 UDP 端口：1347。

## 构建与运行

```powershell
./tools/build.ps1
./tools/run.ps1
```

## 便携版

```powershell
./tools/package.ps1
```

输出：`dist/JustFloatMonitor-portable.zip`。
