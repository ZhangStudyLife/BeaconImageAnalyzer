import cv2, numpy as np, math
video = r"E:/Desktop/下摄手持-为解决下摄远距离车灯识别为信标灯的问题2026_07_09_18_39_56_Video.avi"
W,H=188,120
frames=[5404,5414,5424,5434,5444,5454]

def comps(gray, thr):
    bw=(gray>=thr).astype(np.uint8)
    vis=np.zeros_like(bw)
    out=[]
    for y in range(H):
        for x in range(W):
            if bw[y,x]==0 or vis[y,x]: continue
            q=[(x,y)]; vis[y,x]=1; pts=[]
            for px,py in q:
                pts.append((px,py))
                for dy in (-1,0,1):
                    for dx in (-1,0,1):
                        if dx==0 and dy==0: continue
                        nx,ny=px+dx,py+dy
                        if 0<=nx<W and 0<=ny<H and bw[ny,nx] and not vis[ny,nx]:
                            vis[ny,nx]=1; q.append((nx,ny))
            arr=np.array(pts,dtype=float)
            area=len(pts)
            cx=float(arr[:,0].mean()); cy=float(arr[:,1].mean())
            minx=int(arr[:,0].min()); maxx=int(arr[:,0].max()); miny=int(arr[:,1].min()); maxy=int(arr[:,1].max())
            vx=float((arr[:,0]**2).mean()-cx*cx); vy=float((arr[:,1]**2).mean()-cy*cy); cxy=float((arr[:,0]*arr[:,1]).mean()-cx*cy)
            tr=vx+vy; det=vx*vy-cxy*cxy; disc=max(0,tr*tr*0.25-det)
            eigmaj=tr*0.5+math.sqrt(disc); eigmin=max(0,tr*0.5-math.sqrt(disc))
            major=4*math.sqrt(eigmaj+0.0001); minor=max(1.0,4*math.sqrt(eigmin+0.0001)); elong=major/minor
            out.append(dict(area=area,cx=cx,cy=cy,minx=minx,maxx=maxx,miny=miny,maxy=maxy,major=major,minor=minor,elong=elong,score=area*elong))
    return sorted(out,key=lambda c:c['area'],reverse=True)
cap=cv2.VideoCapture(video)
for f in frames:
    cap.set(cv2.CAP_PROP_POS_FRAMES, f)
    ok,frame=cap.read()
    if not ok:
        print('read fail',f); continue
    gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
    if gray.shape!=(H,W): gray=cv2.resize(gray,(W,H),interpolation=cv2.INTER_AREA)
    print('FRAME',f)
    for thr in (200,120):
        cs=[c for c in comps(gray,thr) if c['area']>=3][:6]
        print(' thr',thr)
        for c in cs:
            print('  area={area} cx={cx:.1f} cy={cy:.1f} out=({ox:.1f},{oy:.1f}) bbox=({minx},{miny})-({maxx},{maxy}) maj={major:.1f} min={minor:.1f} el={elong:.2f} score={score:.0f}'.format(ox=94-c['cx'], oy=c['cy']-60, **c))
cap.release()
