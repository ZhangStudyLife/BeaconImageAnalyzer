# BeaconImageAnalyzer 开发计划

## 1. 项目名称

**BeaconImageAnalyzer**

中文定位：红外信标图像离线分析上位机。

英文定位：Offline infrared beacon image analysis, replay, annotation, and visualization tool.

本项目面向 Windows 11 桌面端，目标是为无人机红外信标识别算法提供一个稳定、清晰、可复现的离线分析环境。

---

## 2. 项目背景

当前无人机项目需要在室内场地中识别若干个红外信标灯。信标灯波长约为  **840 nm** ，布置在场地内的固定坐标位置。无人机上安装一颗可以看到红外波段的摄像头，用于采集场地中的信标图像。

当前摄像头图像尺寸固定为：

```c
#define MT9V03X_W (188)
#define MT9V03X_H (120)
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
```

摄像头输出的是低分辨率灰度图像。飞行过程中采集到的每一帧图像会被合成为 AVI 视频文件。该 AVI 文件本质上对应连续的 `188 × 120` 灰度图像帧，但在 PC 端使用 OpenCV 解码时，可能被返回为 BGR 三通道图像。因此，软件内部必须统一转换为 `188 × 120` 的单通道 `uint8` 灰度图。

项目当前阶段的核心目标不是直接把信标识别算法做得非常完美，而是先建立一个完整的离线分析闭环：

```text
AVI 视频输入
→ 灰度帧读取
→ 离线运行 C 风格图像算法
→ 输出信标圆心、半径、编号
→ 在图像上画圆可视化
→ 播放、暂停、拖动查看
→ 人工快速标注算法错误
→ 导出带标注视频和结构化错误记录
```

后续可以把以下材料上传给 AI 或交给人工进一步分析：

```text
1. 原始 AVI 视频
2. 标注后的 AVI 视频
3. 当前 C 语言算法代码
4. 结构化错误标注文件 annotation.json
5. 可选的每帧检测结果 result.csv
```

这样可以清楚指出：第几秒、第几帧、第几个识别结果出现错误，具体是阳光干扰、漏检、误检、排序错误还是目标跳变，从而辅助后续算法迭代。

---

## 3. 项目最终目标

开发一个 **Windows 11 only** 的桌面 GUI 上位机： **BeaconImageAnalyzer** 。

它的核心定位是：

> 红外信标图像离线分析、算法回放、可视化验证、错误标注与视频导出工具。

第一版目标是把离线分析闭环搭建完整，不追求最终识别算法最优。

最终软件应支持：

1. 打开飞行采集得到的 AVI 视频；
2. 按 50 Hz 默认帧率播放视频；
3. 支持播放、暂停、拖动进度条、上一帧、下一帧、跳转指定帧；
4. 将 `188 × 120` 原始图像放大显示，便于人工观察；
5. 对每一帧运行 C 风格图像识别算法；
6. 获取算法输出的信标圆心、半径、编号和有效标志；
7. 在图像上画出 `valid = 1` 的识别圆；
8. 显示每个圆的编号、坐标和半径；
9. 允许用户快速标注当前帧或某一段时间内的算法错误；
10. 导出原始分辨率 `188 × 120` 的带标注 AVI 视频；
11. 导出结构化标注文件 `annotation.json`；
12. 可选导出每帧算法结果 `result.csv`；
13. 后续方便替换、重写、迭代 C 语言识别算法。

---

## 4. 第一版功能范围

第一版只做“离线回放 + 算法运行 + 画圆 + 标错 + 导出”的最小完整闭环。

### 4.1 视频输入

支持打开 AVI 文件。

第一版默认约束：

```text
图像宽度：188
图像高度：120
默认帧率：50 Hz
图像格式：灰度图，PC 解码后统一转 uint8 单通道
```

如果 OpenCV 读取到的视频帧是 BGR 三通道，软件内部应自动转换为灰度图。

如果 OpenCV 读取到的视频 FPS 与 50 Hz 不一致，第一版仍可按 50 Hz 作为默认时间基准，同时在界面上显示读取到的 FPS 和当前采用的 FPS。

### 4.2 GUI 播放器

第一版必须支持：

```text
打开视频
播放
暂停
拖动进度条
上一帧
下一帧
跳转指定帧
跳转指定时间
显示当前帧号
显示当前时间
显示总帧数
显示总时长
显示视频宽高
显示当前采用的 FPS
```

### 4.3 图像放大显示

原始图像只有 `188 × 120`，直接显示太小，因此 GUI 内部必须放大显示。

默认建议放大 5 倍：

```text
188 × 120 → 940 × 600
```

注意：

1. 算法永远运行在原始 `188 × 120` 图像上；
2. GUI 放大只影响显示，不影响算法输入；
3. 画圆时应先在原始坐标系下计算，再映射到放大显示坐标系；
4. 不允许因为 GUI 放大导致算法坐标或半径发生混乱。

### 4.4 算法运行

第一版需要接入一个 C 风格图像算法模块。

每一帧处理流程：

```text
OpenCV 解码视频帧
→ 转成 188×120 uint8 灰度图
→ 调用 beacon_image_process()
→ 获取 beacon_result_t
→ 缓存当前帧结果
→ GUI 显示检测结果
→ 图像上画圆和编号
```

第一版算法可以很简单，只需要跑通流程。可以采用：

```text
灰度图输入
→ 二值化
→ 连通域分析
→ 面积过滤
→ 质心计算
→ 面积估算半径
→ 输出圆结果
```

第一版不要求解决所有阳光干扰问题。

### 4.5 画圆可视化

图像上只显示 `valid = 1` 的圆。

每个有效圆应显示：

```text
圆心
圆圈
编号 #0 / #1 / #2 ...
```

右侧信息栏显示完整数据：

```text
Frame: 842
Time: 16.84 s

#0 valid=1 x=12.3 y=-5.8 r=7.2
#1 valid=1 x=-30.1 y=10.5 r=4.8
#2 valid=1 x=4.0 y=20.2 r=3.1
```

第一版不显示 `valid = 0` 的圆。

### 4.6 错误标注

错误标注是第一版核心功能之一。

用户需要能够快速记录：

```text
哪一帧出错
哪一段时间出错
第几个圆出错
错误类型是什么
备注是什么
```

第一版错误类型固定为：

```text
漏检
误检
排序错误
阳光干扰
目标跳变
其他
```

支持两种标注方式：

1. 当前帧标注；
2. 时间段标注。

标注结果保存为 `annotation.json`。

### 4.7 视频导出

第一版必须支持导出带标注的 AVI 视频。

导出要求：

```text
输出分辨率：188 × 120
输出帧率：默认 50 Hz
输出格式：AVI
输出内容：原始灰度图 + 检测圆 + 圆心 + 编号
```

由于原始分辨率很小，导出视频中不建议叠加大量文字。详细错误描述放在 `annotation.json` 中。

输出命名建议：

```text
input.avi
input_marked.avi
input_annotation.json
input_result.csv
```

### 4.8 可选 CSV 导出

第一版可以可选导出每帧检测结果。

CSV 格式建议：

```csv
frame,time_sec,index,valid,x,y,radius
842,16.84,0,1,12.3,-5.8,7.2
842,16.84,1,1,-30.1,10.5,4.8
```

CSV 主要用于后续算法分析，不是第一版核心交互功能。

---

## 5. 第一版非目标

为了防止项目跑偏，第一版明确不做以下内容：

```text
不做实时摄像头采集
不做飞控串口通信
不做无人机控制
不做地图坐标解算
不做多信标三角定位
不做复杂目标跟踪
不做神经网络识别
不做训练集管理系统
不做在线参数滑条
不做内置完整 C 编译器
不做 Web 版本
不做 Electron 版本
不做 Python 主程序版本
不做移动端
不做跨平台适配
不做复杂插件系统
```

第一版重点只有：

```text
离线 AVI → C 风格算法 → 可视化画圆 → 人工标错 → 导出视频/标注文件
```

---

## 6. 技术栈

第一版技术栈固定为：

```text
操作系统：Windows 11 only
开发语言：C++17 或 C++20
GUI 框架：Qt 6
视频处理：OpenCV
算法核心：C 风格模块
构建系统：CMake
```

### 6.1 技术路线要求

1. GUI 层使用 Qt 6；
2. 视频读取、帧转换、视频导出使用 OpenCV；
3. 算法模块保持纯 C 风格，不依赖 Qt；
4. 算法模块不依赖 OpenCV；
5. GUI 层不直接写复杂图像算法；
6. 视频读写逻辑不要塞进算法层；
7. 错误标注逻辑不要塞进算法层；
8. 使用 CMake 组织工程；
9. 先保证流程跑通，再考虑界面美化和性能优化。

### 6.2 不采用的技术路线

第一版不要改成：

```text
Python 主程序
Web 前端
Electron
C# / WPF
移动端 App
嵌入式端直接运行
```

后续如果需要快速算法验证，可以另开 Python 脚本，但不作为本项目主程序。

---

## 7. 输入输出文件

### 7.1 输入文件

主要输入：

```text
*.avi
```

视频内容：

```text
红外摄像头采集得到的灰度图像序列
尺寸固定为 188 × 120
默认按 50 Hz 处理
```

### 7.2 输出文件

第一版输出：

```text
input_marked.avi
input_annotation.json
```

可选输出：

```text
input_result.csv
```

后续扩展输出：

```text
input_marked_preview.mp4
input_debug_binary.avi
input_candidates.csv
```

---

## 8. 视频数据格式

### 8.1 原始图像格式

嵌入式端图像格式：

```c
uint8 mt9v03x_image[120][188];
```

含义：

```text
每个像素为 uint8 灰度值
0 表示黑
255 表示白
```

### 8.2 PC 端读取格式

OpenCV 读取 AVI 时，可能返回：

```text
单通道灰度 Mat
或 BGR 三通道 Mat
```

软件内部统一转换为：

```text
cv::Mat gray
类型：CV_8UC1
尺寸：188 × 120
```

然后再拷贝或映射为算法需要的二维数组：

```c
unsigned char image[120][188];
```

### 8.3 帧率处理

第一版默认采用：

```text
50 FPS
```

时间计算：

```text
time_sec = frame_index / 50.0
```

如果 OpenCV 能读取到视频自身 FPS，应在界面上显示：

```text
Video FPS: xxx
Using FPS: 50
```

第一版可以优先使用 50 Hz，避免不同图传或编码器导致时间基准混乱。

---

## 9. 算法接口设计

算法接口必须保持 C 风格，方便后续移植回单片机。

建议文件：

```text
algorithm/beacon_image.h
algorithm/beacon_image.c
algorithm/beacon_image_config.h
```

### 9.1 头文件接口

```c
#ifndef BEACON_IMAGE_H_
#define BEACON_IMAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_IMAGE_W 188
#define BEACON_IMAGE_H 120
#define BEACON_MAX_CIRCLE_COUNT 8

typedef struct
{
    float x;
    float y;
    float radius;
    unsigned char valid;
} beacon_circle_t;

typedef struct
{
    beacon_circle_t circles[BEACON_MAX_CIRCLE_COUNT];
    unsigned char count;
} beacon_result_t;

void beacon_image_init(void);

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result
);

#ifdef __cplusplus
}
#endif

#endif
```

### 9.2 算法层原则

算法层只做图像处理。

算法层不允许依赖：

```text
Qt
OpenCV
GUI 控件
AVI 文件
JSON 文件
CSV 文件
Windows API
```

算法层只允许处理：

```text
uint8 灰度图
内部缓存
阈值
连通域
圆结果
```

### 9.3 第一版算法内容

第一版算法可以采用简单流程：

```text
输入 188×120 灰度图
→ 二值化
→ 连通域扫描
→ 过滤小面积连通域
→ 计算连通域质心
→ 用面积估算半径
→ 输出若干个圆
```

排序规则第一版不强求最终正确。可以先按面积或半径降序输出。后续需要优化“最近信标优先”“排除阳光干扰”“可信度评分”时，只修改算法内部，不改变上位机整体框架。

---

## 10. 坐标系定义

这是项目中容易出错的部分，必须固定清楚。

图像尺寸：

```text
W = 188
H = 120
图像中心约为 (94, 60)
```

算法输出结构体：

```c
typedef struct
{
    float x;
    float y;
    float radius;
    unsigned char valid;
} beacon_circle_t;
```

坐标含义：

```text
x：飞机在图像圆心的右侧，X 为正
y：飞机在图像圆心的上方，Y 为正
radius：圆半径，单位像素
valid：是否有效
```

如果算法内部使用：

```c
x = W / 2 - center_x;
y = center_y - H / 2;
```

则画回图像时必须使用：

```c
center_x = W / 2 - x;
center_y = H / 2 + y;
```

注意：

1. 图像像素坐标的 `x` 向右为正；
2. 图像像素坐标的 `y` 向下为正；
3. 算法坐标不是普通图像坐标；
4. GUI 画圆前必须完成坐标转换；
5. 坐标转换逻辑必须集中封装，不要散落在多个文件中。

建议封装函数：

```cpp
QPointF algorithmToImagePoint(float x, float y)
{
    const float cx = BEACON_IMAGE_W * 0.5f - x;
    const float cy = BEACON_IMAGE_H * 0.5f + y;
    return QPointF(cx, cy);
}
```

---

## 11. GUI 布局设计

主窗口建议布局：

```text
┌──────────────────────────────────────────────┐
│ 菜单栏：打开视频 / 保存标注 / 导出视频        │
├──────────────────────────────┬───────────────┤
│                              │ 当前帧信息     │
│                              │ 检测结果列表   │
│        视频显示区             │ 错误标注面板   │
│     188×120 放大显示          │ 标注记录列表   │
│                              │               │
├──────────────────────────────┴───────────────┤
│ 播放控制：播放 暂停 上一帧 下一帧 进度条       │
└──────────────────────────────────────────────┘
```

### 11.1 菜单栏

包含：

```text
File
  Open Video
  Save Annotation
  Load Annotation
  Export Marked AVI
  Export CSV
  Exit

View
  Zoom 1x
  Zoom 2x
  Zoom 5x
  Show Detection Overlay

Help
  About
```

第一版可简化，但基本入口要清晰。

### 11.2 视频显示区

要求：

```text
默认 5 倍放大显示
显示当前帧
显示检测圆
显示圆心
显示编号
支持拖动进度条后立即刷新画面
```

### 11.3 右侧检测结果区

显示：

```text
视频文件名
视频宽高
总帧数
当前帧
当前时间
读取 FPS
采用 FPS
当前检测圆数量
当前检测圆列表
```

检测圆列表格式：

```text
#0 valid=1 x=12.3 y=-5.8 r=7.2
#1 valid=1 x=-30.1 y=10.5 r=4.8
```

### 11.4 错误标注面板

控件包括：

```text
错误类型下拉框
关联圆编号下拉框或输入框
备注输入框
标记当前帧按钮
设置片段开始按钮
设置片段结束按钮
保存片段标注按钮
删除选中标注按钮
标注记录列表
```

错误类型：

```text
Missed Detection / 漏检
False Positive / 误检
Wrong Order / 排序错误
Sunlight Interference / 阳光干扰
Target Jump / 目标跳变
Other / 其他
```

---

## 12. 错误标注设计

### 12.1 标注目标

标注功能不是简单写备注，而是为了后续能精确告诉 AI 或人工：

```text
哪一段视频中算法错了
错的是第几个输出圆
错误类型是什么
理论上应该怎样
这是否与阳光干扰有关
```

### 12.2 当前帧标注

用户停在某一帧，选择错误类型，点击“标记当前帧”。

自动记录：

```text
video
frame
time_sec
type
circle_index
note
```

示例：

```json
{
  "type": "false_positive",
  "start_frame": 842,
  "end_frame": 842,
  "start_time_sec": 16.84,
  "end_time_sec": 16.84,
  "circle_index": 0,
  "description": "阳光区域被识别为 #0"
}
```

### 12.3 时间段标注

用户操作流程：

```text
跳到错误开始帧
→ 点击 Set Start
→ 跳到错误结束帧
→ 点击 Set End
→ 选择错误类型
→ 选择关联圆编号
→ 填写备注
→ Save Segment
```

示例：

```json
{
  "type": "sunlight_interference",
  "start_frame": 842,
  "end_frame": 913,
  "start_time_sec": 16.84,
  "end_time_sec": 18.26,
  "circle_index": 0,
  "description": "阳光区域被识别为最近信标，导致 #0 排序错误"
}
```

### 12.4 annotation.json 格式

建议整体格式：

```json
{
  "project": "BeaconImageAnalyzer",
  "video": {
    "file": "2026_04_02_03_32_38_Video.avi",
    "width": 188,
    "height": 120,
    "fps_used": 50.0,
    "frame_count": 2934
  },
  "algorithm": {
    "name": "beacon_image_process",
    "version": "v1",
    "note": "simple threshold + connected components"
  },
  "annotations": [
    {
      "type": "false_positive",
      "start_frame": 842,
      "end_frame": 913,
      "start_time_sec": 16.84,
      "end_time_sec": 18.26,
      "circle_index": 0,
      "description": "阳光区域被识别为最近信标"
    }
  ]
}
```

### 12.5 多个错误并存

第一版允许同一帧或同一时间段存在多个标注。

例如同一段视频中可以同时标注：

```text
#0 误检
#1 排序错误
整体存在阳光干扰
```

不要强行限制每一帧只能有一个错误。

---

## 13. 导出视频设计

### 13.1 主输出

第一版主输出为原始分辨率标注 AVI：

```text
input_marked.avi
```

要求：

```text
分辨率：188 × 120
帧率：50 FPS
内容：原始灰度图 + 有效圆检测结果
```

### 13.2 画面内容

每帧画：

```text
valid=1 的圆圈
圆心
编号 #0 #1 #2
```

不要在 `188 × 120` 图像中叠加大量文字。

### 13.3 导出流程

导出时建议重新逐帧处理，而不是只导出当前缓存，以保证结果可复现。

流程：

```text
打开输入 AVI
→ 从第 0 帧开始读取
→ 转灰度
→ 调用算法
→ 画圆
→ 写入 output AVI
→ 直到最后一帧
```

如果后续增加结果缓存，可以提供选项：

```text
Use cached results
Re-run algorithm
```

第一版默认可以重新运行算法。

### 13.4 后续扩展

后续可以增加放大预览视频：

```text
input_marked_preview.mp4
```

预览版可以导出为：

```text
940 × 600
或 1280 × 720
```

但第一版主目标仍然是 `188 × 120` 原始分辨率 AVI。

---

## 14. CSV 导出设计

CSV 不是第一版最核心功能，但建议实现，便于后续分析。

文件名：

```text
input_result.csv
```

字段：

```csv
frame,time_sec,index,valid,x,y,radius
```

示例：

```csv
frame,time_sec,index,valid,x,y,radius
842,16.84,0,1,12.3,-5.8,7.2
842,16.84,1,1,-30.1,10.5,4.8
843,16.86,0,1,12.0,-5.6,7.1
```

后续可扩展字段：

```csv
area,score,mean_gray,max_gray,bbox_x,bbox_y,bbox_w,bbox_h,circularity
```

但第一版不强制。

---

## 15. 工程目录结构

建议工程目录如下：

```text
BeaconImageAnalyzer/
├── CMakeLists.txt
├── PLAN.md
├── README.md
├── app/
│   ├── main.cpp
│   ├── MainWindow.h
│   ├── MainWindow.cpp
│   ├── VideoWidget.h
│   ├── VideoWidget.cpp
│   ├── AnnotationPanel.h
│   └── AnnotationPanel.cpp
├── algorithm/
│   ├── beacon_image.h
│   ├── beacon_image.c
│   └── beacon_image_config.h
├── core/
│   ├── VideoReader.h
│   ├── VideoReader.cpp
│   ├── AlgorithmRunner.h
│   ├── AlgorithmRunner.cpp
│   ├── FrameRenderer.h
│   ├── FrameRenderer.cpp
│   ├── VideoExporter.h
│   └── VideoExporter.cpp
├── annotation/
│   ├── AnnotationModel.h
│   ├── AnnotationModel.cpp
│   ├── AnnotationJson.h
│   └── AnnotationJson.cpp
├── data/
│   └── sample/
└── build/
```

### 15.1 app/

GUI 主程序。

职责：

```text
主窗口
视频显示控件
按钮和菜单
用户交互
播放控制
标注面板
```

### 15.2 algorithm/

纯 C 风格算法。

职责：

```text
输入 188×120 uint8 灰度图
输出 beacon_result_t
内部完成阈值、连通域、圆检测
```

不得依赖 Qt 和 OpenCV。

### 15.3 core/

核心桥接层。

职责：

```text
视频读取
算法调用封装
坐标转换
画圆渲染
视频导出
```

### 15.4 annotation/

标注数据模型和 JSON 读写。

职责：

```text
错误类型定义
单帧标注
片段标注
annotation.json 保存
annotation.json 读取
```

---

## 16. 模块职责设计

### 16.1 VideoReader

职责：

```text
打开 AVI
读取视频元信息
读取指定帧
逐帧读取
转换为灰度图
提供当前帧号、总帧数、FPS、宽高
```

接口示例：

```cpp
class VideoReader
{
public:
    bool open(const std::string& path);
    bool readFrame(int frameIndex, cv::Mat& gray);
    int frameCount() const;
    double videoFps() const;
    int width() const;
    int height() const;
};
```

### 16.2 AlgorithmRunner

职责：

```text
把 cv::Mat 灰度图转为算法数组
调用 beacon_image_process()
返回检测结果
```

接口示例：

```cpp
class AlgorithmRunner
{
public:
    AlgorithmRunner();
    beacon_result_t process(const cv::Mat& gray);
};
```

### 16.3 FrameRenderer

职责：

```text
将灰度图转成可显示图像
将算法坐标转换为图像坐标
画圆、圆心、编号
生成 GUI 显示图
生成导出视频帧
```

### 16.4 VideoExporter

职责：

```text
打开输入 AVI
逐帧运行算法
调用 FrameRenderer 画圆
写出 output AVI
显示导出进度
```

### 16.5 AnnotationModel

职责：

```text
保存错误标注列表
新增标注
删除标注
修改标注
查询当前帧相关标注
```

### 16.6 AnnotationJson

职责：

```text
保存 annotation.json
读取 annotation.json
处理版本兼容
```

---

## 17. 开发阶段计划

### Phase 0：项目初始化

目标：搭建可编译运行的 Qt + OpenCV 工程。

任务：

```text
创建 CMakeLists.txt
配置 Qt 6
配置 OpenCV
创建 main.cpp
创建空白 MainWindow
确认 Windows 11 下可以编译运行
```

验收标准：

```text
程序能启动
显示主窗口
无视频功能也可以正常运行
```

### Phase 1：视频读取

目标：能打开 AVI 并读取基本信息。

任务：

```text
实现 VideoReader
支持打开 AVI
读取宽高、帧数、FPS
读取第 0 帧
转换为灰度图
显示视频元信息
```

验收标准：

```text
能打开用户提供的 AVI
确认帧尺寸为 188×120
能显示第一帧
```

### Phase 2：GUI 播放器

目标：实现基本视频播放。

任务：

```text
实现 VideoWidget
实现播放按钮
实现暂停按钮
实现进度条
实现上一帧/下一帧
实现跳转指定帧
默认 5 倍放大显示
```

验收标准：

```text
可以流畅查看 AVI
可以暂停到指定帧
可以拖动进度条查看画面
```

### Phase 3：算法接入

目标：每帧运行 C 风格算法。

任务：

```text
创建 algorithm/beacon_image.h
创建 algorithm/beacon_image.c
实现 beacon_image_process()
实现 AlgorithmRunner
每帧调用算法
在右侧显示结果
```

验收标准：

```text
当前帧能输出 circles[]
右侧能看到 #0 #1 等检测结果
```

### Phase 4：画圆可视化

目标：在图像上画出算法识别结果。

任务：

```text
实现坐标转换
实现圆心绘制
实现半径圆绘制
实现编号绘制
GUI 显示放大后的标注图
```

验收标准：

```text
播放视频时可以看到算法识别出的圆圈
圆圈位置和半径与算法结果对应
编号清晰可见
```

### Phase 5：错误标注

目标：支持人工快速记录算法错误。

任务：

```text
实现错误类型枚举
实现当前帧标注
实现片段开始/结束标注
实现关联圆编号
实现备注输入
实现标注列表显示
实现删除标注
实现保存 annotation.json
实现读取 annotation.json
```

验收标准：

```text
能记录当前帧错误
能记录一段时间错误
能保存 JSON
重新打开后能读取标注
```

### Phase 6：导出标注 AVI

目标：导出原始分辨率的带圆圈 AVI。

任务：

```text
实现 VideoExporter
逐帧读取原 AVI
逐帧运行算法
画圆、圆心、编号
写入 188×120 AVI
显示导出进度
```

验收标准：

```text
生成 input_marked.avi
输出视频能正常播放
每帧带检测圆圈和编号
```

### Phase 7：CSV 导出

目标：可选导出每帧算法结果。

任务：

```text
遍历所有帧
运行算法
写出 frame,time_sec,index,valid,x,y,radius
```

验收标准：

```text
生成 input_result.csv
CSV 可以被 Excel 或 Python 读取
```

---

## 18. 后续扩展方向

第一版完成后，可以逐步扩展。

### 18.1 算法调试视图

支持显示：

```text
原始灰度图
二值化图
连通域图
候选目标图
最终结果图
```

### 18.2 候选目标输出

算法输出分为两层：

```text
candidates：所有候选连通域
circles：最终有效信标
```

这样可以分析错误来源：

```text
二值化阶段错误
连通域阶段错误
筛选阶段错误
排序阶段错误
```

### 18.3 人工真值标注

支持用户点击真实信标中心，手动设置半径。

后续可计算：

```text
漏检率
误检率
圆心误差
半径误差
排序错误率
```

### 18.4 批量处理

支持选择文件夹，批量处理多个 AVI。

输出：

```text
每个 AVI 的 marked.avi
每个 AVI 的 annotation.json
每个 AVI 的 result.csv
```

### 18.5 算法版本记录

每次运行时记录：

```text
算法版本
算法文件 hash
运行时间
输入视频名
关键宏定义
```

避免后续分不清某个结果是哪版算法跑出来的。

### 18.6 双算法对比

支持同时加载两版算法结果，对比：

```text
旧算法输出
新算法输出
差异帧
误检减少情况
漏检减少情况
```

---

## 19. 风险点和注意事项

### 19.1 坐标系风险

算法坐标和图像坐标方向不同，容易画反。

必须集中封装转换函数，并在注释中写清楚：

```text
center_x = W / 2 - x
center_y = H / 2 + y
```

### 19.2 AVI 解码格式风险

AVI 看起来是灰度，但 OpenCV 可能返回 BGR 三通道。

必须统一转换：

```cpp
if frame is BGR → cvtColor(frame, gray, cv::COLOR_BGR2GRAY)
if frame is gray → directly use
```

### 19.3 帧率风险

图传生成的 AVI 帧率可能不稳定或元数据不可靠。

第一版统一默认使用 50 Hz 计算时间，并在界面显示当前采用 FPS。

### 19.4 分辨率太小

188×120 画面很小，导出视频中文字不能太多。

GUI 可以放大显示，但导出主视频仍为原始分辨率。

### 19.5 算法与 GUI 耦合风险

算法必须保持独立。

不要让 `beacon_image.c` 依赖 Qt、OpenCV、窗口控件或文件系统。

### 19.6 项目跑偏风险

本项目第一版不是实时飞控系统，不是最终定位系统，不是神经网络识别系统。

第一版只解决：

```text
离线视频回放
算法结果可视化
人工错误标注
视频和标注导出
```

---

## 20. 第一版完成标准

当以下条件全部满足时，BeaconImageAnalyzer V1 可以认为完成：

```text
1. Windows 11 下可以正常启动 GUI
2. 可以打开 188×120 AVI
3. 可以播放、暂停、拖动、逐帧查看
4. 可以每帧运行 C 风格算法
5. 可以在 GUI 中放大显示检测圆
6. 可以显示圆编号、x、y、radius
7. 可以标注当前帧错误
8. 可以标注时间段错误
9. 可以保存和读取 annotation.json
10. 可以导出 188×120 marked AVI
11. 可选导出 result.csv
12. 算法层不依赖 Qt/OpenCV
13. 坐标转换逻辑清楚且没有方向混乱
```

---

## 21. 当前执行原则

后续开发必须遵守：

```text
先完成框架，再优化算法。
先跑通单个视频，再做批量处理。
先支持人工标注，再做自动评价。
先保持 C 算法接口稳定，再讨论复杂算法。
先保证工程清晰，再考虑界面美化。
```

任何新增功能都必须判断是否服务于第一版核心目标：

```text
离线 AVI → C 风格算法 → 画圆 → 标错 → 导出
```

如果新增功能不服务于该目标，应推迟到后续版本。
