import argparse
import ctypes
import math

import cv2
import numpy as np

from probe_down_regression import H, W, Result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dll")
    parser.add_argument("video")
    args = parser.parse_args()

    image_type = (ctypes.c_ubyte * W) * H
    library = ctypes.CDLL(args.dll)
    library.beacon_image_init.argtypes = []
    library.beacon_image_process.argtypes = [ctypes.POINTER(image_type), ctypes.POINTER(Result)]
    library.beacon_image_init()

    capture = cv2.VideoCapture(args.video)
    frame_index = 0
    matches = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if gray.shape != (H, W):
            gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        image = image_type.from_buffer_copy(gray)
        result = Result()
        library.beacon_image_process(ctypes.byref(image), ctypes.byref(result))

        if result.car_lamp_count:
            car = result.car_lamps[0]
            car_x = W * 0.5 - car.cx
            car_y = H * 0.5 + car.cy
            for index in range(result.beacon_count):
                beacon = result.beacons[index]
                area = math.pi * beacon.radius * beacon.radius
                beacon_x = W * 0.5 - beacon.x
                beacon_y = H * 0.5 + beacon.y
                dx = abs(beacon_x - car_x)
                dy = beacon_y - car_y
                if area <= 8.5 and dx <= 12.0 and 25.0 <= dy <= 40.0:
                    matches.append((frame_index, index, area, beacon_x, beacon_y,
                                    car_x, car_y, dx, dy))
        frame_index += 1

    capture.release()
    print(f"video={args.video}")
    print(f"frames={frame_index} matches={len(matches)}")
    for item in matches[:200]:
        print(
            "frame=%d B%d area=%.0f beacon=(%.1f,%.1f) car=(%.1f,%.1f) dx=%.1f dy=%.1f"
            % item)


if __name__ == "__main__":
    main()
