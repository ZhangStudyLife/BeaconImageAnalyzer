import ctypes, cv2, numpy as np, math, sys
video = r"E:/Desktop/下摄手持-为解决下摄远距离车灯识别为信标灯的问题2026_07_09_18_39_56_Video.avi"
dll_path = r"D:/smartcar/21_smartcar/BeaconImageAnalyzer/build-codex-check/down_edge_reject_check.dll"
W,H=188,120
frame_idx=5016
class Circle(ctypes.Structure):
    _fields_=[('x',ctypes.c_float),('y',ctypes.c_float),('radius',ctypes.c_float),('valid',ctypes.c_ubyte)]
class Rect(ctypes.Structure):
    _fields_=[('cx',ctypes.c_float),('cy',ctypes.c_float),('width',ctypes.c_float),('length',ctypes.c_float),('angle',ctypes.c_float),('valid',ctypes.c_ubyte)]
class Result(ctypes.Structure):
    _fields_=[('circles',Circle*8),('count',ctypes.c_ubyte),('beacons',Circle*8),('beacon_count',ctypes.c_ubyte),('car_lamps',Rect*4),('car_lamp_count',ctypes.c_ubyte),('temporal_beacons',Circle*8),('temporal_beacon_count',ctypes.c_ubyte),('temporal_car_lamps',Rect*4),('temporal_car_lamp_count',ctypes.c_ubyte)]
lib=ctypes.CDLL(dll_path); lib.beacon_image_init()
arr_type=(ctypes.c_ubyte*W)*H
cap=cv2.VideoCapture(video)
if not cap.isOpened(): print('open fail'); sys.exit(1)
res=None
for idx in range(frame_idx+1):
    ok,frame=cap.read()
    if not ok: print('read fail',idx); sys.exit(2)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
    if gray.shape!=(H,W): gray=cv2.resize(gray,(W,H),interpolation=cv2.INTER_AREA)
    gray=np.ascontiguousarray(gray.astype(np.uint8))
    c_img=arr_type.from_buffer_copy(gray); res=Result()
    lib.beacon_image_process(ctypes.byref(c_img), ctypes.byref(res))
cap.release()
print('ALG frame',frame_idx,'B',res.beacon_count,'KB',res.temporal_beacon_count,'C',res.car_lamp_count,'KC',res.temporal_car_lamp_count)
for i in range(res.beacon_count):
    b=res.beacons[i]; print(f' B{i} x={b.x:.1f} y={b.y:.1f} iy={b.y+60:.1f} area={math.pi*b.radius*b.radius:.1f}')
for i in range(res.temporal_beacon_count):
    b=res.temporal_beacons[i]; print(f' KB{i} x={b.x:.1f} y={b.y:.1f} iy={b.y+60:.1f} area={math.pi*b.radius*b.radius:.1f}')
for i in range(res.car_lamp_count):
    c=res.car_lamps[i]; print(f' C{i} cx={c.cx:.1f} cy={c.cy:.1f} iy={c.cy+60:.1f} w={c.width:.1f} l={c.length:.1f}')
for i in range(res.temporal_car_lamp_count):
    c=res.temporal_car_lamps[i]; print(f' KC{i} cx={c.cx:.1f} cy={c.cy:.1f} iy={c.cy+60:.1f} w={c.width:.1f} l={c.length:.1f}')
