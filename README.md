# BeaconImageAnalyzer

红外信标图像离线分析、回放、错误标注和导出工具。

## 当前版本

已实现 V1 闭环：

- 打开 `188 x 120` 未压缩 DIB AVI，例如 `temp/test_video.avi`
- 按 50 FPS 时间基准播放、暂停、拖动、逐帧查看、跳转帧/时间
- 逐帧运行纯 C 风格 `beacon_image_process()` 算法
- 5 倍放大显示灰度图，并绘制有效圆、圆心和编号
- 支持原始图/二值化图实时切换
- 支持当前帧标注、片段标注、结构化图形纠错、删除标注、保存/读取 `annotation.json`
- 自动保存并恢复上次视频、当前帧、视图模式和标注数据
- 使用 OpenCV 导出原始分辨率 `*_marked.avi`
- 导出 `*_result.csv`

视频后端已接入 OpenCV 4.13.0。普通视频优先走 OpenCV `VideoCapture`，导出走 OpenCV `VideoWriter`。`temp/test_video.avi` 是未压缩 DIB AVI，MSYS2 OpenCV 的 FFMPEG 后端会在该输入上崩溃，因此读取该类文件时自动使用内置 DIB AVI 兜底解析；这不影响 OpenCV 对标准 AVI/MJPG 输出的读写。

## 构建

本机已检测到可用工具链：

- `C:/msys64/mingw64/bin/cmake.exe`
- `C:/msys64/mingw64/bin/ninja.exe`
- MSYS2 MinGW GCC
- Qt 6 Widgets
- OpenCV 4.13.0

执行：

```powershell
./tools/build.ps1
```

## 运行

```powershell
./tools/run.ps1
```

示例视频和 UI 参考图等临时素材放在 `temp/`，该目录不会进入 Git 跟踪。打开 `temp/test_video.avi` 后即可查看示例飞行视频。

GUI 布局按 `plan.md` 的 V1 闭环实现：

- 左侧：`188 x 120` 视频 5 倍放大显示，叠加检测圆、圆心、编号和纠错图形
- 右侧：当前帧信息、检测结果、当前帧相关标注、错误标注面板
- 底部：播放、暂停、上一帧、下一帧、原始/二值视图、进度条、跳转帧、跳转时间
- 顶部：打开视频、保存/读取标注、导出 AVI、导出 CSV、播放控制快捷入口

## GUI 操作

- 拖动进度条会立即暂停播放，释放后停在目标帧。
- `Space`：播放/暂停切换。
- `Left` / `Right`：上一帧 / 下一帧，逐帧操作会暂停播放。
- 标注记录列表中选中一条记录后按 `Delete` 删除。
- 纠错工具支持：选择、画圆、画矩形、画点、自由闭合。
- 画圆时从圆心按下，拖到半径位置后释放；纠错图形坐标按原始 `188 x 120` 像素保存。
- 漏检时使用“期望编号”记录正确目标编号，例如 `GT #0`，不需要关联已有算法输出圆。

## 自动恢复

关闭程序时会在视频同目录写入：

```text
<video_basename>.bia_project.json
```

该文件保存当前视频路径、当前帧、缩放、原始/二值视图、覆盖层开关、文字标注和结构化纠错图形。下次启动会自动恢复上一次项目。

## 命令行验证

探测 AVI：

```powershell
./tools/run.ps1 --probe ./temp/test_video.avi
```

导出标注视频：

```powershell
./tools/run.ps1 --export-marked ./temp/test_video.avi ./temp/test_video_marked.avi
```

导出 CSV：

```powershell
./tools/run.ps1 --export-csv ./temp/test_video.avi ./temp/test_video_result.csv
```

## 已验证

已用 `temp/test_video.avi` 完成端到端验证：

```text
size=188x120
frames=2934
fps=50
backend=Internal DIB AVI fallback
codec=DIB
bit_count=24
```

生成文件：

- `test_video_opencv_marked.avi`
- `test_video_opencv_result.csv`
- `test_video_controls_marked.avi`
- `test_video_controls_result.csv`

导出视频反读验证：

```text
size=188x120
frames=2934
fps=50
backend=FFMPEG
codec=MJPG
bit_count=24
```

## 模块结构

- `algorithm/`：纯 C 风格信标检测算法，不依赖 Qt、OpenCV、文件系统
- `core/`：OpenCV 视频读写、DIB AVI 兜底解析、算法调用、坐标转换、画圆渲染、导出
- `annotation/`：标注模型与 JSON 读写
- `app/`：Qt 6 GUI

坐标转换集中在 `FrameRenderer::algorithmToImagePoint()`：

```cpp
center_x = BEACON_IMAGE_W * 0.5f - x;
center_y = BEACON_IMAGE_H * 0.5f + y;
```
