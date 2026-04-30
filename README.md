# BeaconImageAnalyzer

红外信标图像离线分析、回放、错误标注和导出工具。项目面向 `188 x 120` 信标视频，支持逐帧运行 C 算法、查看检测结果、人工纠错、批量补标和结果导出。

## 功能概览

- 视频读取：支持普通视频通过 OpenCV `VideoCapture` 读取；对未压缩 DIB AVI 提供内置兜底解析，避免部分 OpenCV/FFMPEG 后端崩溃。
- 视频回放：支持播放/暂停、上一帧/下一帧、拖动进度条、跳转帧、跳转时间、原始图/二值图切换。
- 真实帧率播放：GUI 打开视频后优先使用视频自身 FPS；倍速播放按真实时间推进，处理跟不上时自动跳到应该显示的目标帧。
- 检测覆盖层：逐帧调用 `beacon_image_process()`，在画面上叠加有效圆、圆心、编号和人工纠错图形。
- 多算法实例：可以新建实例或导入 C 文件，为不同算法版本建立独立实例，并在最多 4 个分屏中对比显示。
- 分屏操作：左键选择当前实例；多分屏时可用鼠标中键拖动交换分屏位置。
- 像素检查：鼠标悬浮在视频画面内时显示原始像素 `X/Y/Gray`，便于检查阈值和局部亮度。
- 自动暂停：播放中可按“目标跳变”或“数量变化”触发自动暂停，用于快速定位异常帧。
- 快速右键纠错：在当前实例画面上右键，可快速添加误检、排序错误、目标跳变、漏检或其他类型纠错。
- 手动图形纠错：支持选择、画圆、画矩形、画点和自由闭合图形；坐标按原始视频像素保存。
- 标注管理：支持当前帧纠错、片段标注、筛选已保存标注、双击跳转到记录帧、Delete 删除选中标注。
- 自定义错误类型：可新增自定义错误类型和说明，便于扩展标注分类。
- 批量补标：支持手动指定帧范围批量添加，也支持自动识别匹配帧后批量标记。
- 自动恢复：关闭时写入 `<video_basename>.bia_project.json`，下次启动自动恢复上次视频、当前帧、视图模式和标注。
- 结果导出：支持导出带覆盖层的 AVI，以及逐帧算法结果 CSV。
- 命令行模式：支持探测视频信息、导出标注视频和导出 CSV，便于批处理验证。
- 暗色粗野风 UI：主界面采用粗边框、硬阴影和高对比色块，左侧快捷栏、中央视频工作台、右侧标注面板、底部播放控制台分区显示。

## 构建

本机预期工具链：

- `C:/msys64/mingw64/bin/cmake.exe`
- `C:/msys64/mingw64/bin/ninja.exe`
- MSYS2 MinGW GCC
- Qt 6 Widgets
- OpenCV 4.13.0

执行：

```powershell
./tools/build.ps1
```

构建产物会生成到 `build/`，该目录被 `.gitignore` 忽略，可以随时删除后重新构建。

## 运行

```powershell
./tools/run.ps1
```

如果 `build/BeaconImageAnalyzer.exe` 不存在，运行脚本会先尝试构建。

## 生成便携版

生成可直接复制给 Windows 用户使用的便携版 zip：

```powershell
./tools/package.ps1
```

输出文件：

```text
dist/BeaconImageAnalyzer-portable.zip
```

如果输出已存在，脚本默认拒绝覆盖；确认覆盖时执行：

```powershell
./tools/package.ps1 -Force
```

便携版会收集 Qt、OpenCV、MinGW 和相关 DLL，并生成启动批处理文件。

## GUI 操作

- `Space`：播放/暂停切换。
- `Left` / `Right`：上一帧 / 下一帧，逐帧操作会暂停播放。
- 拖动进度条：立即暂停并跳转到目标帧。
- 倍速下拉框：支持 `1/8` 到 `8` 倍速；播放按视频真实 FPS 和倍速推进。
- 视图下拉框：在原始图像和二值化图像之间切换。
- 覆盖层开关：菜单“视图 / 显示检测覆盖”可开关检测圆、编号和纠错图形。
- 鼠标悬浮：显示当前像素坐标和灰度值。
- 画圆：从圆心按下，拖到半径位置后释放。
- 漏检：填写期望编号，并用圆形纠错标出漏检目标位置。
- 标注列表：选中记录后按 `Delete` 删除，双击已保存记录可跳转到对应帧。

## 标注与项目文件

手动保存标注时会写出 JSON 文件，包含视频信息、错误类型、帧范围、错误源、期望编号、描述和纠错图形。

自动恢复文件位于视频同目录：

```text
<video_basename>.bia_project.json
```

该文件保存当前视频路径、当前帧、视图模式、覆盖层开关、窗口状态、算法实例信息、文字标注和结构化纠错图形。

## 命令行

查看帮助：

```powershell
./tools/run.ps1 --help
```

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

## 模块结构

- `algorithm/`：纯 C 风格信标检测算法，不依赖 Qt、OpenCV 和文件系统。
- `core/`：视频读写、DIB AVI 兜底解析、算法调用、坐标转换、画面渲染和导出。
- `annotation/`：标注模型与 JSON 读写。
- `app/`：Qt 6 GUI、播放控制、标注面板和交互逻辑。
- `tools/`：构建、运行和便携版打包脚本。
- `img/`：应用图标资源。

坐标转换集中在 `FrameRenderer::algorithmToImagePoint()`：

```cpp
center_x = BEACON_IMAGE_W * 0.5f - x;
center_y = BEACON_IMAGE_H * 0.5f + y;
```

## 临时文件

- `build/`、`build_*`、`dist/`、`temp/` 都是可再生成或临时目录，默认不进入 Git。
- 示例视频、导出视频、CSV、自动恢复项目文件都建议放在 `temp/` 或视频所在目录。
