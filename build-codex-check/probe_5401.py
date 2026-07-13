import cv2
video=r"E:/Desktop/下摄手持-为解决下摄远距离车灯识别为信标灯的问题2026_07_09_18_39_56_Video.avi"
W,H=188,120

def largest(gray, thr):
    bw=(gray>=thr).astype('uint8')
    n, labels, stats, cent=cv2.connectedComponentsWithStats(bw,8)
    rows=[]
    for i in range(1,n):
        area=int(stats[i,cv2.CC_STAT_AREA])
        if area<3: continue
        x=int(stats[i,cv2.CC_STAT_LEFT]); y=int(stats[i,cv2.CC_STAT_TOP]); w=int(stats[i,cv2.CC_STAT_WIDTH]); h=int(stats[i,cv2.CC_STAT_HEIGHT])
        rows.append((area,round(float(cent[i][0]),1),round(float(cent[i][1]),1),x,y,x+w-1,y+h-1))
    return sorted(rows, reverse=True)[:3]
cap=cv2.VideoCapture(video)
for f in range(5401,5405):
    cap.set(cv2.CAP_PROP_POS_FRAMES,f)
    ok,frame=cap.read()
    if not ok: print('fail',f); continue
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
    if gray.shape!=(H,W): gray=cv2.resize(gray,(W,H),interpolation=cv2.INTER_AREA)
    print('F',f,'200',largest(gray,200),'120',largest(gray,120))
cap.release()
