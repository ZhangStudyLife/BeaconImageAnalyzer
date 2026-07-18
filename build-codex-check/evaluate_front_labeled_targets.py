import argparse
import ctypes
import math

import cv2
import numpy as np

from probe_down_regression import H, W, Result


DATASETS = {
    "0712": {
        "video": r"E:/Desktop/前后摄45度结构/早上6点半，两个窗户都打开的前摄视频2026_07_12_06_12_57_Video.avi",
        "targets": {
            6004: [(10.0, 20.0)],
            6762: [(68.3, 42.7)],
            6783: [(68.6, 42.7)],
            7677: [(122.0, 68.5)],
            8530: [(6.5, 22.0)],
            8534: [(8.0, 22.0)],
            11204: [(97.2, 57.4)],
            12507: [(85.4, 46.1)],
            12574: [(115.0, 62.1)],
            12778: [(80.7, 73.0), (38.0, 42.4)],
            12946: [(85.7, 40.5), (6.7, 110.7)],
            14710: [(99.0, 45.8), (146.0, 76.0)],
            14723: [(94.6, 65.1), (150.2, 94.8)],
            14777: [(69.1, 61.5)],
        },
    },
    "0711": {
        "video": r"E:/Desktop/前后摄45度结构/2026_07_11_07_58_58_Video.avi",
        "targets": {
            1918: [(64.0, 79.5)],
            1976: [(88.0, 96.0)],
            2066: [(60.5, 25.0), (73.7, 89.2)],
            12493: [(101.5, 25.0), (129.5, 26.0)],
            15694: [(23.6, 5.2), (24.3, 18.0), (173.4, 84.4)],
        },
    },
}


def load_library(path):
    image_type = (ctypes.c_ubyte * W) * H
    library = ctypes.CDLL(path)
    library.beacon_image_init.argtypes = []
    library.beacon_image_process.argtypes = [ctypes.POINTER(image_type), ctypes.POINTER(Result)]
    library.beacon_image_init()
    return library, image_type


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dll")
    parser.add_argument("dataset", choices=DATASETS)
    args = parser.parse_args()

    dataset = DATASETS[args.dataset]
    library, image_type = load_library(args.dll)
    capture = cv2.VideoCapture(dataset["video"])
    targets = dataset["targets"]
    hits = 0
    total = sum(len(items) for items in targets.values())

    for frame_index in range(max(targets) + 1):
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"Cannot read frame {frame_index}")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if gray.shape != (H, W):
            gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        result = Result()
        image = image_type.from_buffer_copy(gray)
        library.beacon_image_process(ctypes.byref(image), ctypes.byref(result))
        if frame_index not in targets:
            continue

        outputs = [
            (W * 0.5 - result.beacons[index].x,
             H * 0.5 + result.beacons[index].y,
             math.pi * result.beacons[index].radius ** 2)
            for index in range(result.beacon_count)
        ]
        for target_x, target_y in targets[frame_index]:
            nearest = min(
                outputs,
                key=lambda item: (item[0] - target_x) ** 2 + (item[1] - target_y) ** 2,
                default=None,
            )
            distance = math.inf if nearest is None else math.hypot(
                nearest[0] - target_x, nearest[1] - target_y)
            hit = distance <= 8.0
            hits += int(hit)
            print(
                f"frame={frame_index} target=({target_x:.1f},{target_y:.1f}) "
                f"hit={int(hit)} nearest={nearest} distance={distance:.1f}"
            )

    capture.release()
    print(f"dataset={args.dataset} hits={hits}/{total}")


if __name__ == "__main__":
    main()
