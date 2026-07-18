import ctypes
import math

import cv2
import numpy as np


VIDEO = r"E:/Desktop/前后摄45度结构/2026_07_16_07_17_03_Video.avi"
DLL = r"D:/smartcar/21_smartcar/BeaconImageAnalyzer/build-codex-check/instance_0716_probe.dll"
W = 188
H = 120
REPORT_FRAMES = set(range(393, 425)) | {
    1007, 1255, 1909, 2715, 2805, 3042, 3080, 3239, 5871, 12942, 17353, 17931
}
REPORT_FRAMES.update(range(2535, 2556))
REPORT_FRAMES.update(range(3229, 3245))
REPORT_FRAMES.update(range(5861, 5881))
REPORT_FRAMES.update(range(17395, 17416))
REPORT_FRAMES.update(range(17343, 17361))
CAR_SCAN_START = 2700
CAR_SCAN_END = 2805
DETAILED_FRAMES = {2545, 17405}


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


def component_stats(xs, ys, gray):
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
        "area": area,
        "cx": cx,
        "cy": cy,
        "min_x": int(xs.min()),
        "max_x": int(xs.max()),
        "min_y": int(ys.min()),
        "max_y": int(ys.max()),
        "major": major,
        "minor": minor,
        "elong": major / minor,
        "max_gray": int(gray[ys, xs].max()),
        "mean_gray": float(gray[ys, xs].mean()),
    }


def components(gray, mode):
    if mode == "car":
        threshold = np.full((H, W), 200, dtype=np.uint8)
        threshold[:64, :] = 150
        binary = (gray >= threshold).astype(np.uint8)
        for y in range(64):
            x = 0
            while x < W:
                if binary[y, x]:
                    x += 1
                    continue
                start = x
                while x < W and not binary[y, x]:
                    x += 1
                if start > 0 and x < W and x - start <= 4:
                    binary[y, start:x] = 1
    elif mode == "beacon":
        threshold = np.full((H, W), 120, dtype=np.uint8)
        threshold[:45, :] = 80
        threshold[:, :19] = 80
        threshold[:, 172:] = 80
        binary = (gray >= threshold).astype(np.uint8)
    else:
        binary = (gray >= 70).astype(np.uint8)

    count, labels = cv2.connectedComponents(binary, connectivity=8)
    result = []
    for label in range(1, count):
        ys, xs = np.nonzero(labels == label)
        result.append(component_stats(xs, ys, gray))
    return result


def output_pixel(circle):
    return 94.0 - circle.x, 60.0 + circle.y


def print_components(frame_index, gray, output):
    targets = [output_pixel(output.beacons[i]) for i in range(output.beacon_count)]
    for mode in ("car", "beacon", "weak"):
        items = components(gray, mode)
        selected = []
        for item in items:
            near_output = any(
                math.hypot(item["cx"] - tx, item["cy"] - ty) <= 8.0
                for tx, ty in targets
            )
            detailed_beacon = (
                frame_index in DETAILED_FRAMES
                and mode == "beacon"
                and item["area"] >= 2
                and item["max_gray"] >= 120
            )
            detailed_car = (
                frame_index in DETAILED_FRAMES
                and mode == "car"
                and item["area"] >= 20
            )
            if near_output or detailed_beacon or detailed_car or (
                mode == "car"
                and item["area"] >= 20
                and item["cy"] >= 35
                and item["elong"] >= 1.4
            ):
                selected.append(item)
        selected.sort(key=lambda item: item["area"], reverse=True)
        for item in selected[:8]:
            print(
                f"  {mode}: a={item['area']:3d} c=({item['cx']:6.1f},{item['cy']:5.1f}) "
                f"box={item['max_x'] - item['min_x'] + 1:2d}x{item['max_y'] - item['min_y'] + 1:2d} "
                f"major={item['major']:5.1f} minor={item['minor']:4.1f} "
                f"elong={item['elong']:4.2f} gray={item['mean_gray']:5.1f}/{item['max_gray']}"
            )


def main():
    lib = ctypes.CDLL(DLL)
    lib.beacon_image_init()
    lib.beacon_image_reset_temporal()
    image_type = (ctypes.c_ubyte * W) * H
    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        raise RuntimeError("video open failed")

    for frame_index in range(max(REPORT_FRAMES) + 1):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"frame {frame_index} read failed")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        c_image = image_type.from_buffer_copy(gray)
        output = Result()
        lib.beacon_image_process(ctypes.byref(c_image), ctypes.byref(output))
        if CAR_SCAN_START <= frame_index <= CAR_SCAN_END and output.car_lamp_count > 0:
            car = output.car_lamps[0]
            print(
                f"CAR_SCAN {frame_index}: ({car.cx:.1f},{car.cy:.1f}) "
                f"l={car.length:.1f} w={car.width:.1f}"
            )
        if frame_index not in REPORT_FRAMES:
            continue

        beacons = [
            (
                round(output.beacons[i].x, 1),
                round(output.beacons[i].y, 1),
                round(math.pi * output.beacons[i].radius ** 2),
            )
            for i in range(output.beacon_count)
        ]
        cars = [
            (
                round(output.car_lamps[i].cx, 1),
                round(output.car_lamps[i].cy, 1),
                round(output.car_lamps[i].length, 1),
                round(output.car_lamps[i].width, 1),
            )
            for i in range(output.car_lamp_count)
        ]
        temporal_cars = [
            (
                round(output.temporal_car_lamps[i].cx, 1),
                round(output.temporal_car_lamps[i].cy, 1),
                round(output.temporal_car_lamps[i].length, 1),
                round(output.temporal_car_lamps[i].width, 1),
            )
            for i in range(output.temporal_car_lamp_count)
        ]
        temporal_beacons = [
            (
                round(output.temporal_beacons[i].x, 1),
                round(output.temporal_beacons[i].y, 1),
                round(math.pi * output.temporal_beacons[i].radius ** 2),
            )
            for i in range(output.temporal_beacon_count)
        ]
        print(
            f"FRAME {frame_index}: B={beacons} KB={temporal_beacons} "
            f"C={cars} KC={temporal_cars}"
        )
        print_components(frame_index, gray, output)

    cap.release()


if __name__ == "__main__":
    main()
