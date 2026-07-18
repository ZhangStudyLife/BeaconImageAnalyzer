import argparse
import ctypes

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument("video")
    args = parser.parse_args()

    baseline, baseline_image_type = load_library(args.baseline)
    candidate, candidate_image_type = load_library(args.candidate)
    capture = cv2.VideoCapture(args.video)
    frame_index = 0
    baseline_totals = [0, 0, 0, 0]
    candidate_totals = [0, 0, 0, 0]
    changed_count_frames = 0
    changed_frames = []

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame.ndim == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        if gray.shape != (H, W):
            gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
        gray = np.ascontiguousarray(gray, dtype=np.uint8)

        old_result = Result()
        new_result = Result()
        old_image = baseline_image_type.from_buffer_copy(gray)
        new_image = candidate_image_type.from_buffer_copy(gray)
        baseline.beacon_image_process(ctypes.byref(old_image), ctypes.byref(old_result))
        candidate.beacon_image_process(ctypes.byref(new_image), ctypes.byref(new_result))

        old_counts = (
            old_result.beacon_count,
            old_result.car_lamp_count,
            old_result.temporal_beacon_count,
            old_result.temporal_car_lamp_count,
        )
        new_counts = (
            new_result.beacon_count,
            new_result.car_lamp_count,
            new_result.temporal_beacon_count,
            new_result.temporal_car_lamp_count,
        )
        for index in range(4):
            baseline_totals[index] += old_counts[index]
            candidate_totals[index] += new_counts[index]
        if old_counts != new_counts:
            changed_count_frames += 1
            if len(changed_frames) < 80:
                changed_frames.append((frame_index, old_counts, new_counts))
        frame_index += 1

    capture.release()
    print(f"video={args.video}")
    print(f"frames={frame_index}")
    print(f"baseline totals B,C,KB,KC={tuple(baseline_totals)}")
    print(f"candidate totals B,C,KB,KC={tuple(candidate_totals)}")
    print(f"count-changed frames={changed_count_frames}")
    for item in changed_frames:
        print(f"  frame={item[0]} old={item[1]} new={item[2]}")


if __name__ == "__main__":
    main()
