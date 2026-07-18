import argparse
import ctypes
import math

import cv2
import numpy as np

from probe_down_regression import H, W, Result


def load_library(path):
    image_type = (ctypes.c_ubyte * W) * H
    library = ctypes.CDLL(path)
    library.beacon_image_init.argtypes = []
    library.beacon_image_process.argtypes = [ctypes.POINTER(image_type), ctypes.POINTER(Result)]
    library.beacon_image_init()
    return library, image_type


def close(left, right, tolerance=1e-3):
    return math.isclose(float(left), float(right), abs_tol=tolerance)


def circle_equal(left, right):
    return (
        left.valid == right.valid
        and close(left.x, right.x)
        and close(left.y, right.y)
        and close(left.radius, right.radius)
    )


def rect_equal(left, right):
    return (
        left.valid == right.valid
        and close(left.cx, right.cx)
        and close(left.cy, right.cy)
        and close(left.width, right.width)
        and close(left.length, right.length)
        and close(left.angle, right.angle)
    )


def result_differences(left, right):
    differences = []
    groups = (
        ("B", left.beacon_count, right.beacon_count, left.beacons, right.beacons, circle_equal),
        ("KB", left.temporal_beacon_count, right.temporal_beacon_count,
         left.temporal_beacons, right.temporal_beacons, circle_equal),
        ("C", left.car_lamp_count, right.car_lamp_count,
         left.car_lamps, right.car_lamps, rect_equal),
        ("KC", left.temporal_car_lamp_count, right.temporal_car_lamp_count,
         left.temporal_car_lamps, right.temporal_car_lamps, rect_equal),
    )
    for name, left_count, right_count, left_items, right_items, equal in groups:
        if left_count != right_count:
            differences.append(f"{name}:count {left_count}->{right_count}")
            continue
        if any(not equal(left_items[index], right_items[index]) for index in range(left_count)):
            differences.append(f"{name}:value")
    return differences


def compress_ranges(frames):
    if not frames:
        return []
    ranges = []
    start = previous = frames[0]
    for frame in frames[1:]:
        if frame == previous + 1:
            previous = frame
            continue
        ranges.append((start, previous))
        start = previous = frame
    ranges.append((start, previous))
    return ranges


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("left_dll")
    parser.add_argument("right_dll")
    parser.add_argument("video")
    args = parser.parse_args()

    left_library, left_image_type = load_library(args.left_dll)
    right_library, right_image_type = load_library(args.right_dll)
    capture = cv2.VideoCapture(args.video)
    frame_index = 0
    changed_frames = []
    details = []

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if gray.shape != (H, W):
            gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
        gray = np.ascontiguousarray(gray, dtype=np.uint8)

        left_result = Result()
        right_result = Result()
        left_image = left_image_type.from_buffer_copy(gray)
        right_image = right_image_type.from_buffer_copy(gray)
        left_library.beacon_image_process(ctypes.byref(left_image), ctypes.byref(left_result))
        right_library.beacon_image_process(ctypes.byref(right_image), ctypes.byref(right_result))
        differences = result_differences(left_result, right_result)
        if differences:
            changed_frames.append(frame_index)
            details.append((frame_index, ", ".join(differences)))
        frame_index += 1

    capture.release()
    print(f"frames={frame_index}")
    print(f"changed={len(changed_frames)}")
    print("ranges=" + ",".join(
        str(start) if start == end else f"{start}-{end}"
        for start, end in compress_ranges(changed_frames)))
    for frame, difference in details:
        print(f"frame={frame} {difference}")


if __name__ == "__main__":
    main()
