# BeaconImageAnalyzer

三摄像头信标灯视频图像离线分析、同步回放、算法导入、检测标记显示和标注导出工具。项目面向 `188 x 120` 信标视频，图像处理使用 `algorithm/image.c` 中的 `image_update()`。

## 功能概览

- 三路视频：支持一次选择 3 个视频，或分别导入摄像头 1/2/3 视频。
- 三路算法：为三个摄像头分别导入独立 C 图像处理代码。
- 同步播放：播放、暂停、上一帧、下一帧、进度条和时间跳转同时作用于三路视频。
- 帧对齐：每路视频可设置“同步帧”，用于声明三路视频中哪一帧对应同一时间点。
- 上方显示：固定显示三个视频窗口，每路显示当前帧信标灯和长条灯板检测信息。
- 基础能力：保留视频读取、逐帧算法运行、检测结果覆盖层、人工纠错标注、标注视频导出和 CSV 导出。

旧版“多个算法处理同一个视频/图像并分屏对比”的功能已移除，当前版本只保留三摄像头特化工作流。

## 算法接口

摄像头处理代码需要导出：

```c
void image_init(void);
void image_update(void);
uint8 *image_get_frame_buffer(void);
```

检测结果通过 `g_image_beacons`、`g_image_beacon_count`、`g_image_car_lamps`、`g_image_car_lamp_count` 输出，相关结构体定义位于 `algorithm/image.h`。

## 默认三路算法加载

软件启动时会自动为三个窗口加载并动态编译以下处理代码：

- 摄像头 0 / 左窗口：`D:\HDUASC-SmartCar-21st-FlyOverMinefield\CYT2BL3_Image\project\code\Image\image.c`
- 摄像头 1 / 中间窗口：`D:\HDUASC-SmartCar-21st-FlyOverMinefield\CYT2BL3_Image\project\code\Image\image_down.c`
- 摄像头 2 / 右窗口：`D:\HDUASC-SmartCar-21st-FlyOverMinefield\CYT2BL3_Image\project\code\Image\image.c`

如果通过 `文件 / 导入摄像头 N 处理代码` 手动导入过某个窗口的 `.c` 文件，软件会记住该窗口的导入路径；下次启动时优先使用记住的路径。如果记住的文件不存在，则回退到上面的默认路径。

只修改 D 盘这两份算法 `.c` 文件时，不需要执行 `tools/build.ps1`。启动软件时会重新动态编译当前文件；如果软件已经打开，则重新导入对应窗口的处理代码，或重启软件后生效。只有修改本项目 `E:\BeaconImageAnalyzer` 内的 C++/Qt 上位机代码时，才需要重新执行 `tools/build.ps1` 构建软件。

## 构建

本机预期工具链：

- MSYS2 MinGW GCC
- CMake
- Ninja
- Qt 6 Widgets
- OpenCV 4.x

执行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ./tools/build.ps1
```

构建产物生成到 `build/`。

## 运行

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ./tools/run.ps1
```

## GUI 操作

- `文件 / 同时导入三路视频`：一次选择 3 个视频，按选择顺序对应摄像头 1、2、3。
- `文件 / 导入摄像头 N 处理代码`：为指定摄像头导入独立图像处理 C 文件。
- 每个摄像头标题栏的 `同步帧`：设置该视频中与其他视频同一时刻对应的帧号。
- `Space`：播放/暂停。
- `Left` / `Right`：上一同步帧/下一同步帧。
- 数字键 `1`、`2`、`3`：切换当前标注摄像头。
- 视图下拉框：原图和二值图切换。

## 命令行

```powershell
./tools/run.ps1 --help
./tools/run.ps1 --probe ./temp/test_video.avi
./tools/run.ps1 --export-marked ./temp/test_video.avi ./temp/test_video_marked.avi
./tools/run.ps1 --export-csv ./temp/test_video.avi ./temp/test_video_result.csv
```

## 模块结构

- `algorithm/`：默认 C 图像处理算法接口。
- `core/`：视频读取、算法动态编译调用、渲染和导出。
- `annotation/`：标注模型与 JSON 读写。
- `app/`：Qt GUI、三摄像头同步、播放控制和标注面板。
- `tools/`：构建、运行和打包脚本。
