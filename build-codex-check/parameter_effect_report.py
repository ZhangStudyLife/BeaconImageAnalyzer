from __future__ import annotations

import argparse
import base64
import csv
import ctypes
import html
import json
import math
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(r"D:/smartcar/21_smartcar/BeaconImageAnalyzer")
SOURCE_DIR = ROOT / "instances_front&back/07141512_前后摄_信标面积标定算法/algorithm"
WORK_DIR = ROOT / "build-codex-check/parameter-report"
OUT_DIR = Path(
    r"C:/Users/lx/.codex/visualizations/2026/07/14/019f61e5-48cb-7330-9603-f69cb6d6d7ab"
)
GCC = Path(r"C:/msys64/mingw64/bin/gcc.exe")
MSYS_BIN = Path(r"C:/msys64/mingw64/bin")
OUTPUT_PREFIX = "45deg"
RENDER_STANDALONE = False

VIDEOS = {
    "beacon": Path(r"E:/Desktop/前后摄45度结构/有阳光开窗户2026_07_14_06_34_57_Video.avi"),
    "car": Path(r"E:/Desktop/前后摄45度结构/前摄_车灯.avi"),
    "flight": Path(r"E:/Desktop/前后摄45度结构/前摄像头_实际飞.avi"),
}
FRAME_LIMITS = {"beacon": 8000, "car": 7524, "flight": 8000}
DATASET_LABELS = {
    "beacon": "有阳光开窗户 2026-07-14（前摄）",
    "car": "前摄_车灯",
    "flight": "前摄像头_实际飞",
}


@dataclass(frozen=True)
class ParamSpec:
    key: str
    label: str
    group: str
    macro: str | None
    low: str
    current: str
    high: str
    dataset: str
    source_file: str = "c"
    board_id: int = 0
    mode: str = "macro"
    effect: str = ""


PARAMS = [
    ParamSpec("beacon-threshold", "普通信标阈值", "信标阈值", "BEACON_BINARY_THRESHOLD_DEFAULT", "90", "120", "150", "beacon", "config", effect="降低会保留弱信标并扩大白区；升高会压制弱光和背景噪声。"),
    ParamSpec("edge-threshold", "顶部/左右阈值", "信标阈值", "BEACON_EDGE_THRESHOLD", "65", "80", "95", "beacon", effect="只改变顶部和左右边缘区域的灵敏度。"),
    ParamSpec("track-threshold", "跟踪弱光恢复阈值", "信标阈值", "BEACON_TRACK_THRESHOLD", "90", "105", "120", "flight", effect="降低更容易续上弱轨迹；升高更不容易被附近亮点续命。"),
    ParamSpec("top-threshold-y", "顶部低阈值边界", "信标阈值", "BEACON_TOP_THRESHOLD_Y", "35", "45", "55", "beacon", effect="数值越大，使用边缘低阈值的顶部区域越高。"),
    ParamSpec("edge-left-x", "左侧低阈值边界", "信标阈值", "BEACON_EDGE_LEFT_X", "10", "19", "30", "beacon", effect="数值越大，左侧使用边缘阈值的区域越宽。"),
    ParamSpec("edge-right-x", "右侧低阈值边界", "信标阈值", "BEACON_EDGE_RIGHT_X", "160", "172", "180", "beacon", effect="数值越小，右侧使用边缘阈值的区域越宽。"),
    ParamSpec("beacon-min-area", "普通信标最小面积", "面积/形状", "BEACON_MIN_COMPONENT_AREA", "4", "6", "10", "beacon", effect="降低会接受更小连通域；升高会过滤小噪点。"),
    ParamSpec("beacon-edge-min-area", "边缘信标最小面积", "面积/形状", "BEACON_EDGE_MIN_AREA", "1", "2", "4", "beacon", effect="控制边缘、孤立和跟踪恢复目标的最小白像素数。"),
    ParamSpec("beacon-top-max-area", "顶部信标最大面积", "面积/形状", "BEACON_TOP_EDGE_MAX_AREA", "35", "50", "80", "beacon", effect="降低会更严格拒绝顶部大块反光；升高会容纳膨胀信标。"),
    ParamSpec("beacon-edge-max-area", "左右信标最大面积", "面积/形状", "BEACON_EDGE_MAX_AREA", "40", "60", "90", "beacon", effect="降低会更严格拒绝侧边大块；升高会容纳近处或过曝信标。"),
    ParamSpec("isolated-gray", "孤立小信标峰值", "背景判据", "BEACON_ISOLATED_GRAY_MIN", "100", "120", "150", "beacon", effect="升高要求小信标自身更亮，降低会放行较弱的小点。"),
    ParamSpec("isolated-bg", "孤立小信标背景上限", "背景判据", "BEACON_ISOLATED_BG_MAX", "0", "2", "8", "beacon", effect="升高允许更亮的局部背景，但会增加噪点通过概率。"),
    ParamSpec("ring-inner", "局部背景环内半径", "背景判据", "BEACON_LOCAL_RING_INNER", "2", "3", "5", "beacon", effect="改变计算局部背景时避开目标核心的范围。"),
    ParamSpec("ring-outer", "局部背景环外半径", "背景判据", "BEACON_LOCAL_RING_OUTER", "6", "8", "12", "beacon", effect="数值越大，背景平均值覆盖的邻域越广。"),
    ParamSpec("near-lamp-pad", "车灯附近判定范围", "车灯邻域", "LAMP_NEAR_BEACON_PAD", "4", "8", "12", "beacon", effect="升高会把更远的目标视为车灯邻域，过滤更严格。"),
    ParamSpec("near-lamp-min-area", "车灯附近信标最小面积", "车灯邻域", "LAMP_NEAR_BEACON_MIN_AREA", "12", "21", "30", "beacon", effect="升高会拒绝更多靠近车灯的小连通域。"),
    ParamSpec("near-lamp-gray", "车灯附近信标峰值", "车灯邻域", "LAMP_NEAR_BEACON_GRAY_MIN", "130", "150", "180", "beacon", effect="升高要求车灯附近的小信标具有更高峰值。"),
    ParamSpec("near-lamp-bg", "车灯附近背景上限", "车灯邻域", "LAMP_NEAR_BEACON_BACKGROUND_MAX", "10", "20", "35", "beacon", effect="升高会允许车灯附近更亮的背景通过孤立判据。"),
    ParamSpec("car-threshold", "车灯普通区域阈值", "车灯检测", "CAR_LAMP_BINARY_THRESHOLD", "180", "200", "230", "car", effect="降低会扩大车灯白区；升高会压缩过曝区域并减少背景候选。"),
    ParamSpec("car-upper-threshold", "车灯上部阈值", "车灯检测", "CAR_LAMP_UPPER_THRESHOLD", "130", "150", "185", "car", effect="只改变图像上部车灯检测灵敏度。"),
    ParamSpec("car-upper-y", "车灯上部区域高度", "车灯检测", "CAR_LAMP_UPPER_Y", "55", "64", "75", "car", effect="数值越大，使用上部低阈值的区域越高。"),
    ParamSpec("car-bridge-gap", "车灯横向桥接距离", "车灯检测", "CAR_LAMP_BRIDGE_MAX_GAP", "2", "4", "6", "car", effect="升高会填补更宽的横向断点，也更容易把相邻亮区粘连。"),
    ParamSpec("car-min-area", "车灯最小面积", "车灯检测", "CAR_LAMP_MIN_AREA", "16", "24", "40", "car", effect="升高会过滤更小的车灯候选和反光。"),
    ParamSpec("car-max-area", "车灯常规最大面积", "车灯检测", "CAR_LAMP_MAX_AREA", "80", "100", "180", "car", effect="升高会放行更大的候选，降低会更严格拒绝过曝白块。"),
    ParamSpec("car-front-max-area", "前摄车灯最大面积", "车灯检测", "CAR_LAMP_FRONT_MAX_AREA", "120", "180", "240", "car", effect="控制前摄允许的大型长条车灯面积上限。"),
    ParamSpec("car-elongation", "车灯最小长宽比", "车灯检测", "CAR_LAMP_MIN_ELONGATION", "1.4f", "1.6f", "2.2f", "car", effect="升高会更偏向细长目标，减少近圆形反光。"),
    ParamSpec("car-front-length", "前摄车灯最小长度", "车灯检测", "CAR_LAMP_FRONT_MIN_LENGTH", "8.0f", "10.0f", "15.0f", "car", effect="升高会拒绝较短的前摄车灯候选。"),
    ParamSpec("car-back-length", "后摄车灯最小长度", "车灯检测", "CAR_LAMP_MIN_LENGTH", "10.0f", "12.0f", "18.0f", "car", board_id=1, effect="仅在后摄规则下生效；升高会拒绝较短候选。"),
    ParamSpec("b0-match", "B0 匹配距离", "时序跟踪", "B0_MATCH_DISTANCE", "12.0f", "18.0f", "24.0f", "flight", effect="升高更不容易断轨，但更容易关联到邻近目标。"),
    ParamSpec("kalman-gate", "车灯关联距离", "时序跟踪", "KALMAN_GATE_DISTANCE", "18.0f", "24.0f", "30.0f", "car", effect="升高允许车灯在相邻帧移动更远仍保持同一轨迹。"),
    ParamSpec("kalman-new", "车灯新目标距离", "时序跟踪", "KALMAN_NEW_TARGET_DISTANCE", "28.0f", "36.0f", "45.0f", "car", effect="升高会减少重建轨迹，降低会更快把远跳目标当成新目标。"),
    ParamSpec("b0-confirm", "B0 建立确认帧数", "时序跟踪", "B0_INIT_CONFIRM_FRAMES", "1", "2", "3", "flight", effect="升高会减少单帧误建目标，但增加首次输出延迟。"),
    ParamSpec("beacon-misses", "信标最大丢失帧数", "时序跟踪", "BEACON_MAX_MISSES", "1", "3", "5", "flight", effect="升高会延长短暂丢失后的预测保留时间。"),
    ParamSpec("position-alpha", "位置滤波系数", "时序跟踪", "FILTER_POS_ALPHA", "0.50f", "0.65f", "0.80f", "flight", effect="升高更跟手也更抖；降低更平滑但滞后更明显。"),
    ParamSpec("velocity-alpha", "速度滤波系数", "时序跟踪", "FILTER_VEL_ALPHA", "0.15f", "0.30f", "0.45f", "flight", effect="升高让速度预测更灵敏，也更容易放大瞬时变化。"),
    ParamSpec("area-calibration-switch", "面积标定开关", "面积标定", "BEACON_AREA_CALIBRATION_ENABLED", "0", "1", "1", "beacon", "config", effect="当前源码未读取该宏，0 和 1 应产生完全相同的结果。"),
    ParamSpec("camera-board", "前摄/后摄规则", "面积标定", None, "前摄", "前摄", "后摄", "beacon", mode="board", effect="切换车灯规则和对应的信标面积上下限表。"),
]


class Circle(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("radius", ctypes.c_float),
        ("valid", ctypes.c_ubyte),
    ]


class Rect(ctypes.Structure):
    _fields_ = [
        ("cx", ctypes.c_float),
        ("cy", ctypes.c_float),
        ("width", ctypes.c_float),
        ("length", ctypes.c_float),
        ("angle", ctypes.c_float),
        ("valid", ctypes.c_ubyte),
    ]


class Result(ctypes.Structure):
    _fields_ = [
        ("circles", Circle * 8),
        ("count", ctypes.c_ubyte),
        ("beacons", Circle * 8),
        ("beacon_count", ctypes.c_ubyte),
        ("car_lamps", Rect * 4),
        ("car_lamp_count", ctypes.c_ubyte),
        ("temporal_beacons", Circle * 8),
        ("temporal_beacon_count", ctypes.c_ubyte),
        ("temporal_car_lamps", Rect * 4),
        ("temporal_car_lamp_count", ctypes.c_ubyte),
    ]


ROW = ctypes.c_ubyte * 188


@dataclass
class RunData:
    beacon_count: np.ndarray
    beacons: np.ndarray
    car_count: np.ndarray
    cars: np.ndarray
    temporal_beacon_count: np.ndarray
    temporal_beacons: np.ndarray
    temporal_car_count: np.ndarray
    temporal_cars: np.ndarray


def replace_define(text: str, name: str, value: str) -> str:
    pattern = re.compile(rf"(?m)^\s*#define\s+{re.escape(name)}\b.*$")
    replacement = f"#define {name} {value}"
    updated, count = pattern.subn(replacement, text, count=1)
    if count != 1:
        raise RuntimeError(f"未找到宏 {name}")
    return updated


def safe_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "-", value).strip("-._")
    return slug or "comparison"


def configure_from_args(args: argparse.Namespace) -> None:
    global OUT_DIR, OUTPUT_PREFIX, RENDER_STANDALONE

    selected = {
        "beacon": Path(args.beacon_video),
        "car": Path(args.car_video),
        "flight": Path(args.flight_video),
    }
    for name, path in selected.items():
        if not path.is_file():
            raise FileNotFoundError(f"{name} 视频不存在：{path}")
    VIDEOS.update(selected)
    FRAME_LIMITS.update({name: args.frame_limit for name in selected})
    DATASET_LABELS.update({name: path.stem for name, path in selected.items()})
    OUT_DIR = Path(args.output_dir)
    OUTPUT_PREFIX = safe_slug(args.output_prefix)
    RENDER_STANDALONE = bool(args.standalone)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="生成信标算法参数低值/当前值/高值对比报告")
    parser.add_argument("--beacon-video", default=str(VIDEOS["beacon"]), help="信标、面积和环境光参数使用的视频")
    parser.add_argument("--car-video", default=str(VIDEOS["car"]), help="车灯参数使用的视频")
    parser.add_argument("--flight-video", default=str(VIDEOS["flight"]), help="时序参数使用的视频")
    parser.add_argument("--frame-limit", type=int, default=8000, help="每段视频最多处理帧数")
    parser.add_argument("--output-dir", default=str(OUT_DIR), help="输出目录")
    parser.add_argument("--output-prefix", default=OUTPUT_PREFIX, help="输出文件名前缀，仅使用 ASCII 字符")
    parser.add_argument("--standalone", action="store_true", help="同时生成可直接用浏览器打开的独立 HTML")
    args = parser.parse_args()
    if args.frame_limit <= 0:
        parser.error("--frame-limit 必须大于 0")
    return args


def compile_variant(name: str, spec: ParamSpec | None, value: str | None) -> Path:
    variant_dir = WORK_DIR / "variants" / name
    variant_dir.mkdir(parents=True, exist_ok=True)
    c_text = (SOURCE_DIR / "beacon_image.c").read_text(encoding="utf-8")
    h_text = (SOURCE_DIR / "beacon_image.h").read_text(encoding="utf-8")
    config_text = (SOURCE_DIR / "beacon_image_config.h").read_text(encoding="utf-8")
    if spec is not None and spec.macro is not None and value is not None:
        if spec.source_file == "config":
            config_text = replace_define(config_text, spec.macro, value)
        else:
            c_text = replace_define(c_text, spec.macro, value)
    (variant_dir / "beacon_image.c").write_text(c_text, encoding="utf-8", newline="\n")
    (variant_dir / "beacon_image.h").write_text(h_text, encoding="utf-8", newline="\n")
    (variant_dir / "beacon_image_config.h").write_text(config_text, encoding="utf-8", newline="\n")
    output = variant_dir / f"{name}.dll"
    env = os.environ.copy()
    env["PATH"] = f"{MSYS_BIN};{env.get('PATH', '')}"
    command = [
        str(GCC),
        "-shared",
        "-O2",
        "-std=c11",
        "-I",
        str(variant_dir),
        "-I",
        str(ROOT / "algorithm"),
        "-o",
        str(output),
        str(variant_dir / "beacon_image.c"),
        "-lm",
    ]
    completed = subprocess.run(command, env=env, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(f"编译 {name} 失败：{completed.stderr}{completed.stdout}")
    return output


def load_frames(path: Path, limit: int) -> np.ndarray:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"无法打开视频：{path}")
    frame_count = min(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)), limit)
    frames = np.empty((frame_count, 120, 188), dtype=np.uint8)
    actual = 0
    while actual < frame_count:
        ok, frame = capture.read()
        if not ok:
            break
        frames[actual] = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        actual += 1
    capture.release()
    if actual == 0:
        raise RuntimeError(f"视频没有可读帧：{path}")
    print(f"loaded {path.name}: {actual} frames")
    return np.ascontiguousarray(frames[:actual])


def run_algorithm(dll_path: Path, frames: np.ndarray, board_id: int) -> RunData:
    library = ctypes.CDLL(str(dll_path))
    library.beacon_image_process.argtypes = [ctypes.POINTER(ROW), ctypes.POINTER(Result)]
    library.beacon_image_process.restype = None
    library.beacon_image_set_camera_board_id.argtypes = [ctypes.c_ubyte]
    library.beacon_image_init()
    library.beacon_image_reset_temporal()
    library.beacon_image_set_camera_board_id(board_id)

    count = len(frames)
    beacon_count = np.zeros(count, dtype=np.uint8)
    beacons = np.full((count, 3, 3), np.nan, dtype=np.float32)
    car_count = np.zeros(count, dtype=np.uint8)
    cars = np.full((count, 1, 5), np.nan, dtype=np.float32)
    temporal_beacon_count = np.zeros(count, dtype=np.uint8)
    temporal_beacons = np.full((count, 3, 3), np.nan, dtype=np.float32)
    temporal_car_count = np.zeros(count, dtype=np.uint8)
    temporal_cars = np.full((count, 1, 5), np.nan, dtype=np.float32)

    result = Result()
    for index, frame in enumerate(frames):
        image_ptr = ctypes.cast(frame.ctypes.data, ctypes.POINTER(ROW))
        library.beacon_image_process(image_ptr, ctypes.byref(result))

        raw_beacon_count = min(int(result.beacon_count), 3)
        beacon_count[index] = raw_beacon_count
        for item in range(raw_beacon_count):
            target = result.beacons[item]
            if target.valid:
                beacons[index, item] = (target.x, target.y, target.radius)

        raw_car_count = min(int(result.car_lamp_count), 1)
        car_count[index] = raw_car_count
        for item in range(raw_car_count):
            target = result.car_lamps[item]
            if target.valid:
                cars[index, item] = (target.cx, target.cy, target.width, target.length, target.angle)

        tracked_beacon_count = min(int(result.temporal_beacon_count), 3)
        temporal_beacon_count[index] = tracked_beacon_count
        for item in range(tracked_beacon_count):
            target = result.temporal_beacons[item]
            if target.valid:
                temporal_beacons[index, item] = (target.x, target.y, target.radius)

        tracked_car_count = min(int(result.temporal_car_lamp_count), 1)
        temporal_car_count[index] = tracked_car_count
        for item in range(tracked_car_count):
            target = result.temporal_car_lamps[item]
            if target.valid:
                temporal_cars[index, item] = (target.cx, target.cy, target.width, target.length, target.angle)

    return RunData(
        beacon_count,
        beacons,
        car_count,
        cars,
        temporal_beacon_count,
        temporal_beacons,
        temporal_car_count,
        temporal_cars,
    )


def nanmean_or_zero(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(finite.mean()) if finite.size else 0.0


def run_metrics(run: RunData) -> dict[str, float | int]:
    valid_track = run.temporal_beacon_count > 0
    positions = run.temporal_beacons[:, 0, :2]
    consecutive = valid_track[1:] & valid_track[:-1]
    if np.any(consecutive):
        delta = positions[1:] - positions[:-1]
        jitter = float(np.linalg.norm(delta[consecutive], axis=1).mean())
    else:
        jitter = 0.0
    return {
        "beacon_frames": int(np.count_nonzero(run.beacon_count)),
        "beacon_detections": int(run.beacon_count.sum()),
        "beacon_radius": round(nanmean_or_zero(run.beacons[:, :, 2]), 3),
        "tracked_beacon_frames": int(np.count_nonzero(run.temporal_beacon_count)),
        "track_step": round(jitter, 3),
        "car_frames": int(np.count_nonzero(run.car_count)),
        "car_detections": int(run.car_count.sum()),
        "car_length": round(nanmean_or_zero(run.cars[:, :, 3]), 3),
        "tracked_car_frames": int(np.count_nonzero(run.temporal_car_count)),
    }


def difference_score(left: RunData, right: RunData) -> np.ndarray:
    score = 80.0 * np.abs(left.beacon_count.astype(float) - right.beacon_count.astype(float))
    score += 80.0 * np.abs(left.car_count.astype(float) - right.car_count.astype(float))
    score += 35.0 * np.abs(left.temporal_beacon_count.astype(float) - right.temporal_beacon_count.astype(float))
    score += 35.0 * np.abs(left.temporal_car_count.astype(float) - right.temporal_car_count.astype(float))

    for item in range(3):
        left_valid = left.beacon_count > item
        right_valid = right.beacon_count > item
        both = left_valid & right_valid
        if np.any(both):
            position_delta = np.linalg.norm(left.beacons[:, item, :2] - right.beacons[:, item, :2], axis=1)
            radius_delta = np.abs(left.beacons[:, item, 2] - right.beacons[:, item, 2])
            score[both] += position_delta[both] + 8.0 * radius_delta[both]

        left_track_valid = left.temporal_beacon_count > item
        right_track_valid = right.temporal_beacon_count > item
        both_track = left_track_valid & right_track_valid
        if np.any(both_track):
            track_delta = np.linalg.norm(
                left.temporal_beacons[:, item, :2] - right.temporal_beacons[:, item, :2], axis=1
            )
            score[both_track] += 0.5 * track_delta[both_track]

    both_car = (left.car_count > 0) & (right.car_count > 0)
    if np.any(both_car):
        car_position = np.linalg.norm(left.cars[:, 0, :2] - right.cars[:, 0, :2], axis=1)
        car_size = np.abs(left.cars[:, 0, 2:4] - right.cars[:, 0, 2:4]).sum(axis=1)
        score[both_car] += car_position[both_car] + 3.0 * car_size[both_car]
    return np.nan_to_num(score, nan=0.0, posinf=0.0, neginf=0.0)


def choose_frame(score: np.ndarray, baseline: RunData, dataset: str) -> int:
    if score.size and float(score.max()) > 0.05:
        return int(np.argmax(score))
    activity = baseline.beacon_count.astype(float) + baseline.car_count.astype(float)
    activity += baseline.temporal_beacon_count.astype(float) * 0.25
    if np.any(activity > 0):
        candidates = np.flatnonzero(activity == activity.max())
        return int(candidates[len(candidates) // 2])
    return len(score) // (2 if dataset == "beacon" else 3)


def image_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = [
        Path(r"C:/Windows/Fonts/msyh.ttc"),
        Path(r"C:/Windows/Fonts/arial.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


def algorithm_to_image(x: float, y: float, scale: int) -> tuple[float, float]:
    return (94.0 - x) * scale, (60.0 + y) * scale


def draw_circle(draw: ImageDraw.ImageDraw, target: np.ndarray, scale: int, color: tuple[int, int, int], label: str, width: int) -> None:
    if not np.all(np.isfinite(target)):
        return
    cx, cy = algorithm_to_image(float(target[0]), float(target[1]), scale)
    radius = max(1.0, float(target[2]) * scale)
    draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), outline=color, width=width)
    draw.line((cx - 3, cy, cx + 3, cy), fill=color, width=width)
    draw.line((cx, cy - 3, cx, cy + 3), fill=color, width=width)
    if label:
        draw.text((cx + radius + 2, max(0, cy - radius - 12)), label, fill=color, font=image_font(12))


def rect_polygon(target: np.ndarray, scale: int) -> list[tuple[float, float]]:
    cx, cy = algorithm_to_image(float(target[0]), float(target[1]), scale)
    width = float(target[2]) * scale
    length = float(target[3]) * scale
    angle = math.radians(float(target[4]))
    major = (math.cos(angle) * length * 0.5, math.sin(angle) * length * 0.5)
    minor = (-math.sin(angle) * width * 0.5, math.cos(angle) * width * 0.5)
    return [
        (cx - major[0] - minor[0], cy - major[1] - minor[1]),
        (cx + major[0] - minor[0], cy + major[1] - minor[1]),
        (cx + major[0] + minor[0], cy + major[1] + minor[1]),
        (cx - major[0] + minor[0], cy - major[1] + minor[1]),
    ]


def draw_rect(draw: ImageDraw.ImageDraw, target: np.ndarray, scale: int, color: tuple[int, int, int], label: str, width: int) -> None:
    if not np.all(np.isfinite(target)):
        return
    points = rect_polygon(target, scale)
    draw.line(points + [points[0]], fill=color, width=width)
    cx, cy = algorithm_to_image(float(target[0]), float(target[1]), scale)
    if label:
        draw.text((cx + 5, max(0, cy - 16)), label, fill=color, font=image_font(12))


def render_panel(frame: np.ndarray, run: RunData, index: int, heading: str, value: str, scale: int = 2) -> Image.Image:
    image = Image.fromarray(frame).resize((188 * scale, 120 * scale), Image.Resampling.NEAREST).convert("RGB")
    draw = ImageDraw.Draw(image)
    for item in range(int(run.beacon_count[index])):
        draw_circle(draw, run.beacons[index, item], scale, (40, 255, 90), f"B{item}", 2)
    for item in range(int(run.car_count[index])):
        draw_rect(draw, run.cars[index, item], scale, (255, 105, 45), f"CAR{item}", 2)
    for item in range(int(run.temporal_beacon_count[index])):
        draw_circle(draw, run.temporal_beacons[index, item], scale, (255, 220, 40), "", 1)
    for item in range(int(run.temporal_car_count[index])):
        draw_rect(draw, run.temporal_cars[index, item], scale, (70, 155, 255), "", 1)

    header = 34
    footer = 26
    panel = Image.new("RGB", (image.width, image.height + header + footer), (24, 24, 24))
    panel.paste(image, (0, header))
    panel_draw = ImageDraw.Draw(panel)
    panel_draw.text((8, 5), f"{heading}  {value}", fill=(245, 245, 245), font=image_font(18))
    summary = (
        f"B {int(run.beacon_count[index])}/{int(run.temporal_beacon_count[index])}   "
        f"CAR {int(run.car_count[index])}/{int(run.temporal_car_count[index])}"
    )
    panel_draw.text((8, image.height + header + 5), summary, fill=(205, 205, 205), font=image_font(14))
    return panel


def beacon_binary(frame: np.ndarray, values: dict[str, float]) -> np.ndarray:
    threshold = np.full(frame.shape, values["BEACON_BINARY_THRESHOLD_DEFAULT"], dtype=np.float32)
    y_grid, x_grid = np.indices(frame.shape)
    edge = (
        (y_grid < values["BEACON_TOP_THRESHOLD_Y"])
        | (x_grid < values["BEACON_EDGE_LEFT_X"])
        | (x_grid >= values["BEACON_EDGE_RIGHT_X"])
    )
    threshold[edge] = values["BEACON_EDGE_THRESHOLD"]
    return np.where(frame >= threshold, 255, 0).astype(np.uint8)


def car_binary(frame: np.ndarray, values: dict[str, float]) -> np.ndarray:
    output = np.zeros_like(frame)
    upper_y = int(values["CAR_LAMP_UPPER_Y"])
    output[:upper_y] = np.where(frame[:upper_y] >= values["CAR_LAMP_UPPER_THRESHOLD"], 255, 0)
    output[upper_y:] = np.where(frame[upper_y:] >= values["CAR_LAMP_BINARY_THRESHOLD"], 255, 0)
    max_gap = int(values["CAR_LAMP_BRIDGE_MAX_GAP"])
    for y in range(min(upper_y, output.shape[0])):
        x = 0
        while x < output.shape[1]:
            if output[y, x] != 0:
                x += 1
                continue
            start = x
            while x < output.shape[1] and output[y, x] == 0:
                x += 1
            if start > 0 and x < output.shape[1] and x - start <= max_gap:
                output[y, start:x] = 255
    return output


def preview_values(spec: ParamSpec, value: str) -> dict[str, float]:
    values = {
        "BEACON_BINARY_THRESHOLD_DEFAULT": 120.0,
        "BEACON_EDGE_THRESHOLD": 80.0,
        "BEACON_TOP_THRESHOLD_Y": 45.0,
        "BEACON_EDGE_LEFT_X": 19.0,
        "BEACON_EDGE_RIGHT_X": 172.0,
        "CAR_LAMP_BINARY_THRESHOLD": 200.0,
        "CAR_LAMP_UPPER_THRESHOLD": 150.0,
        "CAR_LAMP_UPPER_Y": 64.0,
        "CAR_LAMP_BRIDGE_MAX_GAP": 4.0,
    }
    if spec.macro in values:
        values[spec.macro] = float(value.rstrip("fFUu"))
    return values


def render_binary_comparison(frame: np.ndarray, spec: ParamSpec, frame_index: int) -> Image.Image | None:
    beacon_macros = {
        "BEACON_BINARY_THRESHOLD_DEFAULT",
        "BEACON_EDGE_THRESHOLD",
        "BEACON_TOP_THRESHOLD_Y",
        "BEACON_EDGE_LEFT_X",
        "BEACON_EDGE_RIGHT_X",
    }
    car_macros = {
        "CAR_LAMP_BINARY_THRESHOLD",
        "CAR_LAMP_UPPER_THRESHOLD",
        "CAR_LAMP_UPPER_Y",
        "CAR_LAMP_BRIDGE_MAX_GAP",
    }
    if spec.macro not in beacon_macros | car_macros:
        return None
    panels = []
    for heading, value in (("LOW", spec.low), ("CURRENT", spec.current), ("HIGH", spec.high)):
        values = preview_values(spec, value)
        binary = beacon_binary(frame, values) if spec.macro in beacon_macros else car_binary(frame, values)
        panel = Image.fromarray(binary).resize((376, 240), Image.Resampling.NEAREST).convert("RGB")
        wrapped = Image.new("RGB", (376, 274), (24, 24, 24))
        wrapped.paste(panel, (0, 34))
        ImageDraw.Draw(wrapped).text((8, 5), f"{heading}  {value}", fill=(245, 245, 245), font=image_font(18))
        panels.append(wrapped)
    output = Image.new("RGB", (sum(panel.width for panel in panels), max(panel.height for panel in panels)), (20, 20, 20))
    x = 0
    for panel in panels:
        output.paste(panel, (x, 0))
        x += panel.width
    banner = Image.new("RGB", (output.width, 24), (12, 12, 12))
    ImageDraw.Draw(banner).text((8, 3), f"threshold preview before masks  |  frame {frame_index}", fill=(210, 210, 210), font=image_font(13))
    combined = Image.new("RGB", (output.width, output.height + banner.height), (12, 12, 12))
    combined.paste(banner, (0, 0))
    combined.paste(output, (0, banner.height))
    return combined


def render_comparison(frame: np.ndarray, runs: tuple[RunData, RunData, RunData], spec: ParamSpec, frame_index: int) -> Image.Image:
    panels = [
        render_panel(frame, runs[0], frame_index, "LOW", spec.low),
        render_panel(frame, runs[1], frame_index, "CURRENT", spec.current),
        render_panel(frame, runs[2], frame_index, "HIGH", spec.high),
    ]
    content = Image.new("RGB", (sum(panel.width for panel in panels), max(panel.height for panel in panels)), (20, 20, 20))
    x = 0
    for panel in panels:
        content.paste(panel, (x, 0))
        x += panel.width
    banner = Image.new("RGB", (content.width, 30), (12, 12, 12))
    ImageDraw.Draw(banner).text((8, 4), f"{spec.macro or 'CAMERA_BOARD_ID'}  |  frame {frame_index}", fill=(230, 230, 230), font=image_font(16))
    output = Image.new("RGB", (content.width, content.height + banner.height), (12, 12, 12))
    output.paste(banner, (0, 0))
    output.paste(content, (0, banner.height))
    return output


def image_data_uri(image: Image.Image, quality: int = 78) -> str:
    from io import BytesIO

    buffer = BytesIO()
    image.save(buffer, format="JPEG", quality=quality, optimize=True)
    return "data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def save_video(
    path: Path,
    frames: np.ndarray,
    runs: tuple[RunData, RunData, RunData],
    spec: ParamSpec,
    center_frame: int,
) -> None:
    start = max(0, center_frame - 150)
    end = min(len(frames), center_frame + 150)
    sample = render_comparison(frames[start], runs, spec, start)
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"MJPG"),
        50.0,
        sample.size,
    )
    if not writer.isOpened():
        raise RuntimeError(f"无法创建视频：{path}")
    for index in range(start, end):
        image = render_comparison(frames[index], runs, spec, index)
        writer.write(cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR))
    writer.release()


def metric_delta_text(metrics: dict[str, float | int], baseline: dict[str, float | int], dataset: str) -> str:
    if dataset == "car":
        frame_delta = int(metrics["car_frames"]) - int(baseline["car_frames"])
        length_delta = float(metrics["car_length"]) - float(baseline["car_length"])
        return f"车灯检出帧 {frame_delta:+d}，平均长度 {length_delta:+.2f}px"
    frame_delta = int(metrics["beacon_frames"]) - int(baseline["beacon_frames"])
    radius_delta = float(metrics["beacon_radius"]) - float(baseline["beacon_radius"])
    track_delta = int(metrics["tracked_beacon_frames"]) - int(baseline["tracked_beacon_frames"])
    return f"信标检出帧 {frame_delta:+d}，平均半径 {radius_delta:+.2f}px，跟踪帧 {track_delta:+d}"


def write_html(items: list[dict[str, object]]) -> Path:
    payload = json.dumps(items, ensure_ascii=False, separators=(",", ":"))
    groups = []
    for group in dict.fromkeys(item["group"] for item in items):
        options = "".join(
            f'<option value="{html.escape(str(item["key"]))}">{html.escape(str(item["label"]))}</option>'
            for item in items
            if item["group"] == group
        )
        groups.append(f'<optgroup label="{html.escape(str(group))}">{options}</optgroup>')
    options_html = "".join(groups)
    fragment = f'''<div id="beacon-parameter-effect-view">
  <style>
    #beacon-parameter-effect-view {{ color: var(--foreground); width: 100%; }}
    #beacon-parameter-effect-view .compare-grid {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; margin-top: 12px; }}
    #beacon-parameter-effect-view .compare-image {{ display: block; width: 100%; height: auto; image-rendering: pixelated; }}
    #beacon-parameter-effect-view .metric-table {{ width: 100%; border-collapse: collapse; margin-top: 12px; }}
    #beacon-parameter-effect-view .metric-table th,
    #beacon-parameter-effect-view .metric-table td {{ padding: 7px 8px; border-bottom: 1px solid var(--border); text-align: right; }}
    #beacon-parameter-effect-view .metric-table th:first-child,
    #beacon-parameter-effect-view .metric-table td:first-child {{ text-align: left; }}
    #beacon-parameter-effect-view .detail-line {{ margin-top: 10px; color: var(--muted-foreground); }}
    #beacon-parameter-effect-view .mode-buttons {{ display: none; }}
    #beacon-parameter-effect-view .mode-buttons.is-visible {{ display: flex; }}
    @media (max-width: 520px) {{
      #beacon-parameter-effect-view .compare-grid {{ grid-template-columns: 1fr; }}
      #beacon-parameter-effect-view .metric-table {{ font-size: 12px; }}
      #beacon-parameter-effect-view .metric-table th,
      #beacon-parameter-effect-view .metric-table td {{ padding: 6px 4px; }}
    }}
  </style>
  <div class="viz-controls">
    <label class="form-label">参数
      <select id="beacon-param-select" class="form-select">{options_html}</select>
    </label>
    <div id="beacon-view-mode" class="viz-row mode-buttons" aria-label="视图模式">
      <button type="button" class="btn btn-primary" data-mode="overlay" aria-pressed="true">检测结果</button>
      <button type="button" class="btn" data-mode="binary" aria-pressed="false">阈值预览</button>
    </div>
  </div>
  <div class="viz-grid">
    <div class="card viz-stat"><div class="text-muted">差异帧</div><div id="changed-frames" class="viz-stat-value">--</div><div id="changed-context" class="text-small"></div></div>
    <div class="card viz-stat"><div class="text-muted">低值相对当前</div><div id="low-delta" class="text-small">--</div></div>
    <div class="card viz-stat"><div class="text-muted">高值相对当前</div><div id="high-delta" class="text-small">--</div></div>
  </div>
  <div id="beacon-compare-grid" class="compare-grid">
    <img class="compare-image" alt="参数低值的同帧检测结果">
    <img class="compare-image" alt="参数当前值的同帧检测结果">
    <img class="compare-image" alt="参数高值的同帧检测结果">
  </div>
  <div id="effect-detail" class="detail-line"></div>
  <table class="metric-table">
    <thead><tr><th>取值</th><th>信标检出帧</th><th>跟踪帧</th><th>车灯检出帧</th><th>平均半径</th><th>轨迹步长</th></tr></thead>
    <tbody id="metric-body"></tbody>
  </table>
  <script>
    (() => {{
      const root = document.getElementById('beacon-parameter-effect-view');
      const items = {payload};
      const byKey = new Map(items.map(item => [item.key, item]));
      const select = root.querySelector('#beacon-param-select');
      const compareImages = Array.from(root.querySelectorAll('#beacon-compare-grid .compare-image'));
      const modeBox = root.querySelector('#beacon-view-mode');
      let mode = 'overlay';
      function setMode(nextMode) {{
        mode = nextMode;
        modeBox.querySelectorAll('button').forEach(button => {{
          const active = button.dataset.mode === mode;
          button.classList.toggle('btn-primary', active);
          button.setAttribute('aria-pressed', active ? 'true' : 'false');
        }});
        render();
      }}
      function render() {{
        const item = byKey.get(select.value) || items[0];
        const hasBinary = Array.isArray(item.binary_images);
        modeBox.classList.toggle('is-visible', hasBinary);
        if (!hasBinary && mode === 'binary') mode = 'overlay';
        const sources = mode === 'binary' && hasBinary ? item.binary_images : item.overlay_images;
        const labels = ['低值', '当前', '高值'];
        compareImages.forEach((image, index) => {{
          image.src = sources[index];
          image.alt = `${{item.label}}：${{labels[index]}} ${{item.values[index]}} 的同帧对比`;
        }});
        root.querySelector('#changed-frames').textContent = `${{item.changed_frames}} / ${{item.total_frames}}`;
        root.querySelector('#changed-context').textContent = `${{item.changed_percent.toFixed(1)}}% · ${{item.source}} · frame ${{item.frame}}`;
        root.querySelector('#low-delta').textContent = item.low_delta;
        root.querySelector('#high-delta').textContent = item.high_delta;
        root.querySelector('#effect-detail').textContent = item.effect;
        root.querySelector('#metric-body').innerHTML = item.metrics.map((metric, index) => `
          <tr><td>${{labels[index]}} · ${{item.values[index]}}</td><td>${{metric.beacon_frames}}</td><td>${{metric.tracked_beacon_frames}}</td><td>${{metric.car_frames}}</td><td>${{metric.beacon_radius.toFixed(2)}}</td><td>${{metric.track_step.toFixed(2)}}</td></tr>
        `).join('');
      }}
      select.addEventListener('change', () => {{ mode = 'overlay'; render(); }});
      modeBox.querySelectorAll('button').forEach(button => button.addEventListener('click', () => setMode(button.dataset.mode)));
      render();
    }})();
  </script>
</div>
'''
    target = OUT_DIR / f"beacon-parameter-effects-{OUTPUT_PREFIX}.html"
    target.write_text(fragment, encoding="utf-8", newline="\n")
    return target


def render_standalone(fragment_path: Path) -> Path:
    render_candidates = sorted(
        Path.home().glob(".codex/plugins/cache/openai-bundled/visualize/*/skills/visualize/scripts/render.py")
    )
    if not render_candidates:
        raise RuntimeError("未找到 visualize/scripts/render.py，无法生成独立 HTML")
    target = fragment_path.with_name(f"{fragment_path.stem}-standalone.html")
    completed = subprocess.run(
        [os.sys.executable, str(render_candidates[-1]), str(fragment_path), str(target)],
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"生成独立 HTML 失败：{completed.stderr}{completed.stdout}")
    return target


def main() -> None:
    WORK_DIR.mkdir(parents=True, exist_ok=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(MSYS_BIN))

    baseline_dll = compile_variant("baseline", None, None)
    path_limits: dict[str, tuple[Path, int]] = {}
    dataset_keys: dict[str, str] = {}
    for name, path in VIDEOS.items():
        cache_key = str(path.resolve()).casefold()
        dataset_keys[name] = cache_key
        previous = path_limits.get(cache_key)
        path_limits[cache_key] = (path, max(FRAME_LIMITS[name], previous[1] if previous else 0))
    frame_cache = {
        cache_key: load_frames(path, limit)
        for cache_key, (path, limit) in path_limits.items()
    }
    frames_by_dataset = {
        name: frame_cache[dataset_keys[name]][:FRAME_LIMITS[name]]
        for name in VIDEOS
    }
    required_runs = {(spec.dataset, spec.board_id) for spec in PARAMS}
    required_runs.add(("beacon", 0))
    required_runs.add(("beacon", 1))
    baseline_runs = {
        key: run_algorithm(baseline_dll, frames_by_dataset[key[0]], key[1])
        for key in sorted(required_runs)
    }

    output_items: list[dict[str, object]] = []
    summary_rows: list[dict[str, object]] = []
    video_jobs: dict[str, tuple[np.ndarray, tuple[RunData, RunData, RunData], ParamSpec, int]] = {}

    for position, spec in enumerate(PARAMS, start=1):
        print(f"[{position}/{len(PARAMS)}] {spec.key}")
        frames = frames_by_dataset[spec.dataset]
        if spec.mode == "board":
            low_run = baseline_runs[(spec.dataset, 0)]
            current_run = baseline_runs[(spec.dataset, 0)]
            high_run = baseline_runs[(spec.dataset, 1)]
        else:
            low_dll = compile_variant(f"{spec.key}-low", spec, spec.low)
            high_dll = compile_variant(f"{spec.key}-high", spec, spec.high)
            low_run = run_algorithm(low_dll, frames, spec.board_id)
            current_run = baseline_runs[(spec.dataset, spec.board_id)]
            high_run = run_algorithm(high_dll, frames, spec.board_id)

        score = difference_score(low_run, high_run)
        frame_index = choose_frame(score, current_run, spec.dataset)
        changed_frames = int(np.count_nonzero(score > 0.05))
        metrics = [run_metrics(run) for run in (low_run, current_run, high_run)]
        comparison = render_comparison(frames[frame_index], (low_run, current_run, high_run), spec, frame_index)
        comparison_path = OUT_DIR / f"{OUTPUT_PREFIX}-parameter-{spec.key}.jpg"
        comparison.save(comparison_path, quality=84, optimize=True)
        overlay_panels = [
            render_panel(frames[frame_index], run, frame_index, heading, value)
            for run, heading, value in zip(
                (low_run, current_run, high_run),
                ("LOW", "CURRENT", "HIGH"),
                (spec.low, spec.current, spec.high),
            )
        ]
        binary = render_binary_comparison(frames[frame_index], spec, frame_index)
        binary_uris = None
        if binary is not None:
            binary_path = OUT_DIR / f"{OUTPUT_PREFIX}-parameter-{spec.key}-binary.jpg"
            binary.save(binary_path, quality=82, optimize=True)
            binary_uris = [
                image_data_uri(binary.crop((panel * 376, 24, (panel + 1) * 376, binary.height)), quality=72)
                for panel in range(3)
            ]

        item = {
            "key": spec.key,
            "label": spec.label,
            "group": spec.group,
            "values": [spec.low, spec.current, spec.high],
            "frame": frame_index,
            "source": DATASET_LABELS[spec.dataset],
            "total_frames": len(frames),
            "changed_frames": changed_frames,
            "changed_percent": changed_frames * 100.0 / len(frames),
            "low_delta": metric_delta_text(metrics[0], metrics[1], spec.dataset),
            "high_delta": metric_delta_text(metrics[2], metrics[1], spec.dataset),
            "effect": spec.effect,
            "metrics": metrics,
            "overlay_images": [image_data_uri(panel, quality=72) for panel in overlay_panels],
            "binary_images": binary_uris,
        }
        output_items.append(item)
        summary_rows.append({
            "group": spec.group,
            "parameter": spec.label,
            "macro": spec.macro or "CAMERA_BOARD_ID",
            "low": spec.low,
            "current": spec.current,
            "high": spec.high,
            "dataset": spec.dataset,
            "frame": frame_index,
            "changed_frames": changed_frames,
            "total_frames": len(frames),
            "changed_percent": round(changed_frames * 100.0 / len(frames), 4),
            "low_delta": item["low_delta"],
            "high_delta": item["high_delta"],
        })

        if spec.key in {"beacon-threshold", "car-threshold"}:
            video_jobs[spec.key] = (frames, (low_run, current_run, high_run), spec, frame_index)

    summary_path = OUT_DIR / f"{OUTPUT_PREFIX}-parameter-effect-summary.csv"
    with summary_path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    data_path = OUT_DIR / f"{OUTPUT_PREFIX}-parameter-effect-data.json"
    compact_items = [{key: value for key, value in item.items() if not key.endswith("_images")} for item in output_items]
    data_path.write_text(json.dumps(compact_items, ensure_ascii=False, indent=2), encoding="utf-8")

    html_path = write_html(output_items)
    for key, (frames, runs, spec, center_frame) in video_jobs.items():
        save_video(OUT_DIR / f"{OUTPUT_PREFIX}-{key}-low-current-high.avi", frames, runs, spec, center_frame)

    standalone_path = render_standalone(html_path) if RENDER_STANDALONE else None

    print(f"html={html_path}")
    print(f"summary={summary_path}")
    print(f"data={data_path}")
    if standalone_path is not None:
        print(f"standalone={standalone_path}")
    print(f"html_bytes={html_path.stat().st_size}")


if __name__ == "__main__":
    configure_from_args(parse_args())
    main()
