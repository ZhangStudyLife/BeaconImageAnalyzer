import cv2, numpy as np, math, ctypes, sys
video = r"E:/Desktop/下摄-车信标都有2026_07_06_15_58_49_Video.avi"
dll_path = r"D:/smartcar/21_smartcar/BeaconImageAnalyzer/build-codex-check/down_edge_reject_check.dll"
W,H=188,120
frame_idx=3613

class Circle(ctypes.Structure):
    _fields_=[('x',ctypes.c_float),('y',ctypes.c_float),('radius',ctypes.c_float),('valid',ctypes.c_ubyte)]
class Rect(ctypes.Structure):
    _fields_=[('cx',ctypes.c_float),('cy',ctypes.c_float),('width',ctypes.c_float),('length',ctypes.c_float),('angle',ctypes.c_float),('valid',ctypes.c_ubyte)]
class Result(ctypes.Structure):
    _fields_=[('circles',Circle*8),('count',ctypes.c_ubyte),('beacons',Circle*8),('beacon_count',ctypes.c_ubyte),('car_lamps',Rect*4),('car_lamp_count',ctypes.c_ubyte),('temporal_beacons',Circle*8),('temporal_beacon_count',ctypes.c_ubyte),('temporal_car_lamps',Rect*4),('temporal_car_lamp_count',ctypes.c_ubyte)]

def comps(gray, thr):
    bw=(gray>=thr).astype(np.uint8)
    n, labels, stats, cent=cv2.connectedComponentsWithStats(bw,8)
    rows=[]
    for i in range(1,n):
        area=int(stats[i,cv2.CC_STAT_AREA])
        if area<3: continue
        x=int(stats[i,cv2.CC_STAT_LEFT]); y=int(stats[i,cv2.CC_STAT_TOP]); w=int(stats[i,cv2.CC_STAT_WIDTH]); h=int(stats[i,cv2.CC_STAT_HEIGHT])
        pts=np.column_stack(np.where(labels==i)) # y,x
        xs=pts[:,1].astype(float); ys=pts[:,0].astype(float)
        cx=float(xs.mean()); cy=float(ys.mean())
        vx=float((xs*xs).mean()-cx*cx); vy=float((ys*ys).mean()-cy*cy); cxy=float((xs*ys).mean()-cx*cy)
        tr=vx+vy; det=vx*vy-cxy*cxy; disc=max(0,tr*tr*0.25-det)
        eigmaj=tr*0.5+math.sqrt(disc); eigmin=max(0,tr*0.5-math.sqrt(disc))
        major=4*math.sqrt(eigmaj+0.0001); minor=max(1.0,4*math.sqrt(eigmin+0.0001)); elong=major/minor
        # local bg ring pad 3
        pad=3
        minx=max(0,x-pad); maxx=min(W-1,x+w-1+pad); miny=max(0,y-pad); maxy=min(H-1,y+h-1+pad)
        mask=np.ones((maxy-miny+1,maxx-minx+1), dtype=bool)
        mask[y-miny:y-miny+h, x-minx:x-minx+w]=False
        vals=gray[miny:maxy+1,minx:maxx+1][mask]
        bg=float(vals.mean()) if vals.size else 0
        rows.append(dict(area=area,cx=cx,cy=cy,ox=94-cx,oy=cy-60,minx=x,maxx=x+w-1,miny=y,maxy=y+h-1,w=w,h=h,major=major,minor=minor,elong=elong,score=area*elong,bg=bg,maxv=int(gray[labels==i].max())))
    return sorted(rows, key=lambda c:c['area'], reverse=True)

cap=cv2.VideoCapture(video)
if not cap.isOpened(): print('open fail'); sys.exit(1)
# run algorithm causally to frame
lib=ctypes.CDLL(dll_path)
lib.beacon_image_init()
arr_type=(ctypes.c_ubyte*W)*H
res=None; gray_target=None
for idx in range(frame_idx+1):
    ok,frame=cap.read()
    if not ok: print('read fail',idx); sys.exit(2)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
    if gray.shape!=(H,W): gray=cv2.resize(gray,(W,H),interpolation=cv2.INTER_AREA)
    gray=np.ascontiguousarray(gray.astype(np.uint8))
    c_img=arr_type.from_buffer_copy(gray)
    res=Result()
    lib.beacon_image_process(ctypes.byref(c_img), ctypes.byref(res))
    if idx==frame_idx: gray_target=gray.copy()
cap.release()
print('ALG frame',frame_idx,'B',res.beacon_count,'KB',res.temporal_beacon_count,'C',res.car_lamp_count,'KC',res.temporal_car_lamp_count)
for i in range(res.beacon_count):
    b=res.beacons[i]; print(f' B{i} x={b.x:.1f} y={b.y:.1f} iy={b.y+60:.1f} area={math.pi*b.radius*b.radius:.1f}')
for i in range(res.temporal_beacon_count):
    b=res.temporal_beacons[i]; print(f' KB{i} x={b.x:.1f} y={b.y:.1f} iy={b.y+60:.1f} area={math.pi*b.radius*b.radius:.1f}')
for i in range(res.car_lamp_count):
    c=res.car_lamps[i]; print(f' C{i} cx={c.cx:.1f} cy={c.cy:.1f} ix={94-c.cx:.1f} iy={c.cy+60:.1f} w={c.width:.1f} l={c.length:.1f} angle={c.angle:.1f}')
for i in range(res.temporal_car_lamp_count):
    c=res.temporal_car_lamps[i]; print(f' KC{i} cx={c.cx:.1f} cy={c.cy:.1f} ix={94-c.cx:.1f} iy={c.cy+60:.1f} w={c.width:.1f} l={c.length:.1f} angle={c.angle:.1f}')

for thr in (200,120,150):
    print('\nTHR',thr)
    for c in comps(gray_target, thr)[:12]:
        print(' area={area:4d} out=({ox:5.1f},{oy:5.1f}) img=({cx:5.1f},{cy:5.1f}) bbox=({minx},{miny})-({maxx},{maxy}) wh=({w},{h}) maj={major:4.1f} min={minor:4.1f} el={elong:4.2f} bg={bg:5.1f} max={maxv}'.format(**c))
