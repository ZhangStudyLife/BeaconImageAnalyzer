import argparse
import ctypes
import math

import cv2
import numpy as np


W = 188
H = 120
MAX_CIRCLES = 8
MAX_BEACONS = 8
MAX_LAMPS = 4


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
        ("circles", Circle * MAX_CIRCLES),
        ("count", ctypes.c_ubyte),
        ("beacons", Circle * MAX_BEACONS),
        ("beacon_count", ctypes.c_ubyte),
        ("car_lamps", Rect * MAX_LAMPS),
        ("car_lamp_count", ctypes.c_ubyte),
        ("temporal_beacons", Circle * MAX_BEACONS),
        ("temporal_beacon_count", ctypes.c_ubyte),
        ("temporal_car_lamps", Rect * MAX_LAMPS),
        ("temporal_car_lamp_count", ctypes.c_ubyte),
    ]


def connected_components(gray, threshold):
    mask = (gray >= threshold).astype(np.uint8)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    components = []
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        if area < 3:
            continue
        points_y, points_x = np.where(mask[y:y + height, x:x + width] != 0)
        points_x = points_x.astype(np.float32) + x
        points_y = points_y.astype(np.float32) + y
        cx, cy = centroids[index]
        var_x = float(np.mean(points_x * points_x) - cx * cx)
        var_y = float(np.mean(points_y * points_y) - cy * cy)
        cov_xy = float(np.mean(points_x * points_y) - cx * cy)
        trace = var_x + var_y
        determinant = var_x * var_y - cov_xy * cov_xy
        disc = max(0.0, trace * trace * 0.25 - determinant)
        eig_major = trace * 0.5 + math.sqrt(disc)
        eig_minor = max(0.0, trace * 0.5 - math.sqrt(disc))
        major = 4.0 * math.sqrt(eig_major + 0.0001)
        minor = max(1.0, 4.0 * math.sqrt(eig_minor + 0.0001))
        values = gray[y:y + height, x:x + width][mask[y:y + height, x:x + width] != 0]
        components.append((
            int(area), float(cx), float(cy), int(x), int(y), int(width), int(height),
            major, minor, major / minor, float(np.mean(values)), int(np.max(values))))
    return sorted(components, reverse=True)


def format_result(result):
    beacons = [
        (round(result.beacons[i].x, 1), round(result.beacons[i].y, 1),
         round(math.pi * result.beacons[i].radius ** 2))
        for i in range(result.beacon_count)
    ]
    lamps = [
        (round(result.car_lamps[i].cx, 1), round(result.car_lamps[i].cy, 1),
         round(result.car_lamps[i].length, 1), round(result.car_lamps[i].width, 1))
        for i in range(result.car_lamp_count)
    ]
    temporal_beacons = [
        (round(result.temporal_beacons[i].x, 1), round(result.temporal_beacons[i].y, 1))
        for i in range(result.temporal_beacon_count)
    ]
    temporal_lamps = [
        (round(result.temporal_car_lamps[i].cx, 1), round(result.temporal_car_lamps[i].cy, 1))
        for i in range(result.temporal_car_lamp_count)
    ]
    return f"B={beacons} KB={temporal_beacons} C={lamps} KC={temporal_lamps}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dll")
    parser.add_argument("video")
    parser.add_argument("--frames", required=True, help="Comma-separated frame indexes")
    parser.add_argument("--components", action="store_true")
    args = parser.parse_args()

    requested = sorted({int(value) for value in args.frames.split(",")})
    library = ctypes.CDLL(args.dll)
    image_type = (ctypes.c_ubyte * W) * H
    library.beacon_image_init.argtypes = []
    library.beacon_image_process.argtypes = [ctypes.POINTER(image_type), ctypes.POINTER(Result)]
    library.beacon_image_init()

    capture = cv2.VideoCapture(args.video)
    next_frame = 0
    requested_set = set(requested)
    while next_frame <= requested[-1]:
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"Cannot read frame {next_frame}")
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        if gray.shape != (H, W):
            gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        result = Result()
        image = image_type.from_buffer_copy(gray)
        library.beacon_image_process(ctypes.byref(image), ctypes.byref(result))
        if next_frame in requested_set:
            print(f"FRAME {next_frame}: {format_result(result)}")
            if args.components:
                for threshold in (100, 120, 150, 200, 250):
                    print(f"  threshold={threshold}")
                    for comp in connected_components(gray, threshold)[:8]:
                        print(
                            "    area=%d center=(%.1f,%.1f) box=(%d,%d,%d,%d) "
                            "major=%.1f minor=%.1f elong=%.2f mean=%.1f peak=%d" % comp)
        next_frame += 1
    capture.release()


if __name__ == "__main__":
    main()
