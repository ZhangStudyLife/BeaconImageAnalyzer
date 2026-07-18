import ctypes
import math
import sys

import cv2
import numpy as np


VIDEO = r"E:/Desktop/前后摄45度结构/早上6点半，两个窗户都打开的前摄视频2026_07_12_06_12_57_Video.avi"
DLL = r"D:/smartcar/21_smartcar/BeaconImageAnalyzer/build-codex-check/front_area_gate_check.dll"
W = 188
H = 120
TARGETS = {
    2268: [],
    2271: [],
    2519: [],
    2941: [],
    3848: [],
    6004: [(10.0, 20.0)],
    6762: [(68.3, 42.7)],
    6783: [(68.6, 42.7)],
    6883: [],
    7677: [(122.0, 68.5)],
    8534: [(8.0, 22.0)],
    11204: [(97.2, 57.4)],
    12507: [(85.4, 46.1)],
    12574: [(115.0, 62.1)],
    12778: [(80.7, 73.0), (38.0, 42.4)],
    12946: [(85.7, 40.5), (6.7, 110.7)],
    14710: [(99.0, 45.8), (146.0, 76.0)],
    14723: [(94.6, 65.1), (150.2, 94.8)],
    14777: [(69.1, 61.5)],
}
TARGETS.update({frame: [] for frame in range(8525, 8535)})
TARGETS[8530] = [(6.5, 22.0)]

DATASET = "0712"
if len(sys.argv) > 1 and sys.argv[1] == "0711":
    DATASET = "0711"
    VIDEO = r"E:/Desktop/前后摄45度结构/2026_07_11_07_58_58_Video.avi"
    TARGETS = {
        1918: [(64.0, 79.5)],
        1976: [(88.0, 96.0)],
        2066: [(60.5, 25.0), (73.7, 89.2)],
        12493: [(101.5, 25.0), (129.5, 26.0)],
        15694: [(23.6, 5.2), (24.3, 18.0), (173.4, 84.4)],
    }


class Circle(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float),
                ("radius", ctypes.c_float), ("valid", ctypes.c_ubyte)]


class Rect(ctypes.Structure):
    _fields_ = [("cx", ctypes.c_float), ("cy", ctypes.c_float),
                ("width", ctypes.c_float), ("length", ctypes.c_float),
                ("angle", ctypes.c_float), ("valid", ctypes.c_ubyte)]


class Result(ctypes.Structure):
    _fields_ = [("circles", Circle * 8), ("count", ctypes.c_ubyte),
                ("beacons", Circle * 8), ("beacon_count", ctypes.c_ubyte),
                ("car_lamps", Rect * 4), ("car_lamp_count", ctypes.c_ubyte),
                ("temporal_beacons", Circle * 8),
                ("temporal_beacon_count", ctypes.c_ubyte),
                ("temporal_car_lamps", Rect * 4),
                ("temporal_car_lamp_count", ctypes.c_ubyte)]


def component_stats(xs, ys):
    area = len(xs)
    cx = float(xs.mean())
    cy = float(ys.mean())
    dx = xs.astype(np.float64) - cx
    dy = ys.astype(np.float64) - cy
    var_x = float(np.mean(dx * dx))
    var_y = float(np.mean(dy * dy))
    cov_xy = float(np.mean(dx * dy))
    trace = var_x + var_y
    det = var_x * var_y - cov_xy * cov_xy
    disc = max(0.0, trace * trace * 0.25 - det)
    eig_major = trace * 0.5 + math.sqrt(disc)
    eig_minor = max(0.0, trace * 0.5 - math.sqrt(disc))
    major = 4.0 * math.sqrt(eig_major + 0.0001)
    minor = max(1.0, 4.0 * math.sqrt(eig_minor + 0.0001))
    return {
        "area": area, "cx": cx, "cy": cy,
        "min_x": int(xs.min()), "max_x": int(xs.max()),
        "min_y": int(ys.min()), "max_y": int(ys.max()),
        "major": major, "minor": minor, "elong": major / minor,
    }


def components(gray, mode="beacon"):
    if mode == "car":
        threshold = np.full((H, W), 200, dtype=np.uint8)
        threshold[:64, :] = 150
    elif mode == "car200":
        threshold = np.full((H, W), 200, dtype=np.uint8)
    else:
        threshold = np.full((H, W), 120, dtype=np.uint8)
        threshold[:45, :] = 80
        threshold[:, :19] = 80
        threshold[:, 172:] = 80
    binary = (gray >= threshold).astype(np.uint8)
    count, labels = cv2.connectedComponents(binary, connectivity=8)
    result = []
    for label in range(1, count):
        ys, xs = np.nonzero(labels == label)
        result.append(component_stats(xs, ys))
    return result


def format_circle(circle):
    area = math.pi * circle.radius * circle.radius
    return f"({circle.x:.1f},{circle.y:.1f},a={area:.0f})"


def main():
    lib = ctypes.CDLL(DLL)
    lib.beacon_image_init()
    lib.beacon_image_set_camera_board_id(0)
    image_type = (ctypes.c_ubyte * W) * H
    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        raise RuntimeError("无法打开视频")

    last_frame = max(TARGETS)
    no_beacon_false_frames = []
    for frame_index in range(last_frame + 1):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"读取第 {frame_index} 帧失败")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if gray.shape != (H, W):
            gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        c_image = image_type.from_buffer_copy(gray)
        output = Result()
        lib.beacon_image_process(ctypes.byref(c_image), ctypes.byref(output))
        if DATASET == "0712" and 611 <= frame_index <= 986 and output.beacon_count > 0:
            no_beacon_false_frames.append(frame_index)
        if frame_index not in TARGETS:
            continue

        raw = ", ".join(format_circle(output.beacons[i])
                        for i in range(output.beacon_count)) or "-"
        temporal = ", ".join(format_circle(output.temporal_beacons[i])
                             for i in range(output.temporal_beacon_count)) or "-"
        cars = ", ".join(
            f"({output.car_lamps[i].cx:.1f},{output.car_lamps[i].cy:.1f},"
            f"w={output.car_lamps[i].width:.1f},l={output.car_lamps[i].length:.1f})"
            for i in range(output.car_lamp_count)) or "-"
        temporal_cars = ", ".join(
            f"({output.temporal_car_lamps[i].cx:.1f},"
            f"{output.temporal_car_lamps[i].cy:.1f})"
            for i in range(output.temporal_car_lamp_count)) or "-"
        print(f"\nFRAME {frame_index}: B={raw} KB={temporal} C={cars} KC={temporal_cars}")

        comps = components(gray)
        car_comps = components(gray, "car")
        car200_comps = components(gray, "car200")
        for target_x, target_y in TARGETS[frame_index]:
            nearest = sorted(
                comps,
                key=lambda item: (item["cx"] - target_x) ** 2 +
                                 (item["cy"] - target_y) ** 2)[:3]
            print(f" target pixel=({target_x:.1f},{target_y:.1f})")
            for item in nearest:
                distance = math.hypot(item["cx"] - target_x,
                                      item["cy"] - target_y)
                print(
                    "  "
                    f"d={distance:.1f} a={item['area']} "
                    f"c=({item['cx']:.1f},{item['cy']:.1f}) "
                    f"box={item['max_x'] - item['min_x'] + 1}x"
                    f"{item['max_y'] - item['min_y'] + 1} "
                    f"major={item['major']:.2f} minor={item['minor']:.2f} "
                    f"elong={item['elong']:.2f}"
                )
            for mode, source in (("car", car_comps), ("car200", car200_comps)):
                nearest_car = min(
                    source,
                    key=lambda item: (item["cx"] - target_x) ** 2 +
                                     (item["cy"] - target_y) ** 2,
                    default=None)
                if nearest_car is not None:
                    distance = math.hypot(nearest_car["cx"] - target_x,
                                          nearest_car["cy"] - target_y)
                    print(
                        f"  {mode}: d={distance:.1f} a={nearest_car['area']} "
                        f"c=({nearest_car['cx']:.1f},{nearest_car['cy']:.1f}) "
                        f"major={nearest_car['major']:.2f} "
                        f"minor={nearest_car['minor']:.2f} "
                        f"elong={nearest_car['elong']:.2f}"
                    )

    cap.release()
    if DATASET == "0712":
        print(
            f"\nNO_BEACON_611_986: {len(no_beacon_false_frames)} frames "
            f"{no_beacon_false_frames}"
        )


if __name__ == "__main__":
    main()
