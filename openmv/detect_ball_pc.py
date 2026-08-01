# PC端钢球检测: 自适应+鲁棒, 不依赖固定阈值
import cv2
import numpy as np
import os

INPUT_DIR = r"E:\电赛H题\openmv"
OUTPUT_DIR = r"E:\电赛H题\openmv\result"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def imread_unicode(path):
    try:
        with open(path, 'rb') as f:
            data = np.frombuffer(f.read(), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except:
        return None

def find_pipe(gray):
    """自适应找水管: 找图像中最亮的水平带"""
    h, w = gray.shape
    # 每行平均亮度 → 高斯平滑 → 找峰值
    row_mean = np.mean(gray.astype(np.float32), axis=1)
    row_mean = cv2.GaussianBlur(row_mean.reshape(-1,1), (7,1), 2).flatten()

    # 动态阈值: 全局平均+1标准差以上的行=管壁候选
    global_mean = np.mean(row_mean)
    global_std = np.std(row_mean)
    thresh = global_mean + global_std * 0.5

    candidates = np.where(row_mean > thresh)[0]
    if len(candidates) < 4:
        candidates = np.argsort(row_mean)[-20:]
    candidates = np.sort(candidates)

    if len(candidates) > 6:
        y1 = candidates[0]
        y2 = candidates[-1]
        return max(y1-3, 0), min(y2+3, h)
    return h//4, 3*h//4

def detect_ball(img_path):
    img = imread_unicode(img_path)
    if img is None: return
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # ── Step1: 自适应找水管 ──
    ty, by = find_pipe(gray)
    tube = gray[ty:by, :]
    th, tw = tube.shape[:2]
    print(f"Pipe: y={ty}-{by}")

    # ── Step2: 管内找球(亮度无关) ──
    # 自适应阈值: 管内的局部二值化
    tube_blur = cv2.GaussianBlur(tube, (9,9), 2)
    # 用Otsu找球(比管壁暗)
    _, ball_bin = cv2.threshold(tube_blur, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

    # 形态学清理
    k = np.ones((3,3), np.uint8)
    ball_bin = cv2.erode(ball_bin, k, iterations=1)
    ball_bin = cv2.dilate(ball_bin, k, iterations=2)

    # 找最圆的轮廓
    contours, _ = cv2.findContours(ball_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None; best_score = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 6 or area > 500: continue
        peri = cv2.arcLength(cnt, True)
        if peri < 2: continue
        circ = 4*np.pi*area/(peri*peri)
        (cx, cy), r = cv2.minEnclosingCircle(cnt)
        if cx < tw*0.1 or cx > tw*0.9: continue
        if r < 2 or r > th*0.6: continue
        if area > best_score:  # 找最大暗色块=球
            best_score = area*circ
            best = (int(cx), int(cy)+ty, int(r))

    # ── Step3: 画结果 ──
    result = img.copy()
    cv2.rectangle(result, (0, ty), (w, by), (128,128,128), 2)
    cv2.line(result, (w//2, ty), (w//2, by), (128,128,128), 1)

    if best:
        cx, cy, r = best
        cv2.circle(result, (cx, cy), r, (0,0,255), 2)
        cv2.line(result, (cx-8, cy), (cx+8, cy), (0,0,255), 2)
        cv2.line(result, (cx, cy-8), (cx, cy+8), (0,0,255), 2)
        print(f"  BALL: cx={cx} cy={cy} r={r}")
    else:
        print("  NO BALL")

    # ── 右下角调试: 二值化图 ──
    debug = cv2.cvtColor(ball_bin, cv2.COLOR_GRAY2BGR)
    for cnt in contours:
        cv2.drawContours(debug, [cnt], -1, (0,255,0), 1)
    dh = min(80, h//2)
    scale = dh / th
    dw = min(int(tw*scale), w//2)
    if dw>0 and dh>0:
        debug = cv2.resize(debug, (dw, dh))
        result[h-dh:, w-dw:] = debug

    name = os.path.splitext(os.path.basename(img_path))[0]
    out = os.path.join(OUTPUT_DIR, f"{name}.jpg")
    _, buf = cv2.imencode('.jpg', result)
    with open(out, 'wb') as f: f.write(buf)

# ── 主程序 ──
files = [f for f in os.listdir(INPUT_DIR) if f in ['capture_%d.bmp'%i for i in range(5)]]
print(f"Found {len(files)} images")
for f in sorted(files): detect_ball(os.path.join(INPUT_DIR, f))
print("Done!")
