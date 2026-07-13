import ctypes
import os
import sys

video = r"E:/Desktop/下摄手持-为解决下摄远距离车灯识别为信标灯的问题2026_07_09_18_39_56_Video.avi"
dll_path = r"D:/smartcar/21_smartcar/BeaconImageAnalyzer/build-codex-check/down_edge_reject_check.dll"
try:
    import cv2
    import numpy as np
except Exception as e:
    print("IMPORT_ERR", repr(e))
    sys.exit(2)

W, H = 188, 120
MAXC = 8
MAXB = 8
MAXL = 4

class Circle(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("radius", ctypes.c_float), ("valid", ctypes.c_ubyte)]
class Rect(ctypes.Structure):
    _fields_ = [("cx", ctypes.c_float), ("cy", ctypes.c_float), ("width", ctypes.c_float), ("length", ctypes.c_float), ("angle", ctypes.c_float), ("valid", ctypes.c_ubyte)]
class Result(ctypes.Structure):
    _fields_ = [
        ("circles", Circle * MAXC), ("count", ctypes.c_ubyte),
        ("beacons", Circle * MAXB), ("beacon_count", ctypes.c_ubyte),
        ("car_lamps", Rect * MAXL), ("car_lamp_count", ctypes.c_ubyte),
        ("temporal_beacons", Circle * MAXB), ("temporal_beacon_count", ctypes.c_ubyte),
        ("temporal_car_lamps", Rect * MAXL), ("temporal_car_lamp_count", ctypes.c_ubyte),
    ]

lib = ctypes.CDLL(dll_path)
lib.beacon_image_init.argtypes = []
lib.beacon_image_init.restype = None
lib.beacon_image_process.argtypes = [ctypes.POINTER((ctypes.c_ubyte * W) * H), ctypes.POINTER(Result)]
lib.beacon_image_process.restype = None

cap = cv2.VideoCapture(video)
if not cap.isOpened():
    print("VIDEO_OPEN_FAIL", video)
    sys.exit(3)
lib.beacon_image_init()
arr_type = (ctypes.c_ubyte * W) * H
summary = []
interesting = []
for idx in range(0, 5459):
    ok, frame = cap.read()
    if not ok:
        print("READ_FAIL", idx)
        break
    if frame.ndim == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame
    if gray.shape != (H, W):
        gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_AREA)
    gray = np.ascontiguousarray(gray.astype(np.uint8))
    c_img = arr_type.from_buffer_copy(gray)
    res = Result()
    lib.beacon_image_process(ctypes.byref(c_img), ctypes.byref(res))
    if 5401 <= idx <= 5458:
        b = res.beacons[0] if res.beacon_count else None
        k = res.temporal_beacons[0] if res.temporal_beacon_count else None
        car = res.car_lamps[0] if res.car_lamp_count else None
        kc = res.temporal_car_lamps[0] if res.temporal_car_lamp_count else None
        def circ_s(c):
            if not c: return "-"
            area = 3.1415926 * c.radius * c.radius
            return f"x={c.x:.1f} y={c.y:.1f} iy={c.y+60:.1f} r={c.radius:.1f} area={area:.0f}"
        def rect_s(r):
            if not r: return "-"
            return f"cx={r.cx:.1f} cy={r.cy:.1f} iy={r.cy+60:.1f} w={r.width:.1f} l={r.length:.1f}"
        if res.beacon_count or res.car_lamp_count or res.temporal_beacon_count or res.temporal_car_lamp_count:
            interesting.append(f"{idx}: B{res.beacon_count} {circ_s(b)} | KB{res.temporal_beacon_count} {circ_s(k)} | C{res.car_lamp_count} {rect_s(car)} | KC{res.temporal_car_lamp_count} {rect_s(kc)}")
cap.release()
print("\n".join(interesting[:200]))
print("lines", len(interesting))
