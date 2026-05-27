# BeaconImageAnalyzer

三摄像头信标灯视频图像离线分析、同步回放、算法导入、结果融合、雷达显示和标注导出工具。项目面向 `188 x 120` 信标视频，单路图像处理算法仍使用 `beacon_image_process()`，三路融合分析使用 `beacon_fusion_analyze()`。

## 功能概览

- 三路视频：支持一次选择 3 个视频，或分别导入摄像头 1/2/3 视频。
- 三路算法：为三个摄像头分别导入独立 C 图像处理代码。
- 融合算法：第四个入口导入三摄像头结果分析 C 代码。
- 同步播放：播放、暂停、上一帧、下一帧、进度条和时间跳转同时作用于三路视频。
- 帧对齐：每路视频可设置“同步帧”，用于声明三路视频中哪一帧对应同一时间点。
- 上方显示：固定显示三个视频窗口，每路显示当前帧信标灯数量、XY 坐标和面积。
- 下方显示：输出最终角度、距离、融合后信标灯总数，并用 360 度雷达图标出目标位置。
- 基础能力：保留视频读取、逐帧算法运行、检测结果覆盖层、人工纠错标注、标注视频导出和 CSV 导出。

旧版“多个算法处理同一个视频/图像并分屏对比”的功能已移除，当前版本只保留三摄像头特化工作流。

## 算法接口

单路摄像头处理代码需要导出：

```c
void beacon_image_init(void);
void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result
);
```

融合分析代码需要导出：

```c
void beacon_fusion_init(void);
void beacon_fusion_analyze(
    const beacon_result_t camera_results[BEACON_CAMERA_COUNT],
    beacon_fusion_result_t *result
);
```

相关结构体定义位于 `algorithm/beacon_image.h` 和 `algorithm/beacon_fusion.h`。

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
- `文件 / 导入三路融合分析代码`：导入融合分析 C 文件。
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

- `algorithm/`：默认 C 检测算法和三路融合接口。
- `core/`：视频读取、算法动态编译调用、融合动态编译调用、渲染和导出。
- `annotation/`：标注模型与 JSON 读写。
- `app/`：Qt GUI、三摄像头同步、播放控制、标注面板和雷达图。
- `tools/`：构建、运行和打包脚本。
