# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

红外信标图像离线分析、回放、错误标注和导出工具。处理 `188 x 120` 未压缩 DIB AVI 视频，逐帧运行信标检测算法，支持标注纠错和导出。

## 技术栈

- C++17 / Qt 6 / OpenCV 4.13.0
- MSYS2 MinGW 工具链
- CMake + Ninja 构建系统

## 常用命令

```powershell
# 构建
./tools/build.ps1

# 运行（自动构建）
./tools/run.ps1

# 打包便携版
./tools/package.ps1
./tools/package.ps1 -Force  # 覆盖已有输出

# 命令行验证
./tools/run.ps1 --probe ./temp/test_video.avi
./tools/run.ps1 --export-marked ./temp/test_video.avi ./output_marked.avi
./tools/run.ps1 --export-csv ./temp/test_video.avi ./output_result.csv
```

## 项目结构

```
algorithm/    纯 C 信标检测算法（beacon_image_process()），无外部依赖
core/         视频读写、DIB AVI 兜底解析、算法调度、坐标转换、渲染、导出
annotation/   标注模型与 JSON 序列化
app/          Qt 6 GUI 层（MainWindow、VideoWidget、AnnotationPanel 等）
img/          应用图标资源（logo.png、logo.ico）
tools/        PowerShell 构建/运行/打包脚本
installer/    Inno Setup 安装包脚本
docs/         开发计划文档
```

## 关键架构

**视频后端**：普通视频走 OpenCV `VideoCapture`，未压缩 DIB AVI 自动使用内置兜底解析（MSYS2 OpenCV FFMPEG 后端会崩溃）。导出走 OpenCV `VideoWriter`。

**坐标转换**：集中在 `FrameRenderer::algorithmToImagePoint()`
```cpp
center_x = BEACON_IMAGE_W * 0.5f - x;
center_y = BEACON_IMAGE_H * 0.5f + y;
```

**结果结构体**：`beacon_result_t` 包含三层数据通道
- `circles[]/count` — legacy 通道，兼容旧算法
- `beacons[]/beacon_count` — 场地信标灯（`beacon_circle_t`，圆形）
- `car_lamps[]/car_lamp_count` — 车灯标识（`beacon_rect_t`，矩形）

**车灯标识**：用户车上使用高亮度红外灯带，用 `beacon_rect_t` 描述（cx, cy, width, length, angle）。区分思路：固定高阈值（~200）二值化，信标灯是点状，灯带是条状（长宽比大）。

**自动恢复**：程序关闭时写入 `<video_basename>.bia_project.json`，保存视频路径、当前帧、缩放、视图模式、标注数据。下次启动自动恢复。

**UI 布局**：DJI 风格暗色中控台
- 左侧：竖向快捷栏（打开视频、保存/读取标注、导出等）
- 中央：视频画布（5 倍放大，叠加检测圆/矩形、圆心、编号）
- 右侧：状态/检测结果/标注面板
- 底部：回放控制台（播放、逐帧、进度条、跳转）
- 顶部：菜单栏

## 开发注意事项

- 算法层（`algorithm/`）纯 C 实现，不依赖 Qt/OpenCV
- DIB AVI 解析在 `VideoReader` 中实现，修改时注意兼容性
- 标注坐标按原始 `188 x 120` 像素保存
- 漏检标注使用"期望编号"（如 `GT #0`），不关联算法输出
- `temp/` 目录存放测试视频和临时文件，不进入 Git
