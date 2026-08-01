# 反光背景下钢球检测 - Canny边缘 + 形态学
# DEBUG_STEP: 0=原图 1=均衡化 2=Canny 3=膨胀 4=腐蚀 5=检测结果
import sensor
import image
import time
import gc
from pyb import UART

DEBUG_STEP = 5  # 改这个值: 0~5

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# OpenMV UART(3): P4=TX, P5=RX. It sends $B,<0.1mm position># at 9600 baud.
uart = UART(3, 9600, timeout_char=10)

CENTER_X = 160
SCALE_MM = 0.38
ROI_X = 20
ROI_Y = 80
ROI_W = 280
ROI_H = 80

last_uart_ms = 0
labels = ["0:Original", "1:HistEQ", "2:Canny", "3:Dilate", "4:Erode", "5:Detect"]
print("Step:", labels[DEBUG_STEP])

while True:
    gc.collect()
    img = sensor.snapshot()

    # ── 裁ROI ──
    roi = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))

    # ── 均衡化增强对比 ──
    roi.histeq()

    # ── Canny边缘检测（光照不变） ──
    roi.find_edges(image.EDGE_CANNY, threshold=(30, 80))

    # ── 膨胀连通边缘 ──
    roi.dilate(2)

    # ── 腐蚀去噪 ──
    roi.erode(1)

    # ── 找球 ──
    blobs = roi.find_blobs([(128, 255)], pixels_threshold=15, area_threshold=15,
                            merge=True, margin=5)

    # ── 调试显示 ──
    if DEBUG_STEP == 0:
        d = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
        img.draw_image(d, 0, 0, x_scale=1.15, y_scale=3)
    elif DEBUG_STEP == 1:
        d = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
        d.histeq()
        img.draw_image(d, 0, 0, x_scale=1.15, y_scale=3)
    elif DEBUG_STEP == 2:
        d = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
        d.histeq()
        d.find_edges(image.EDGE_CANNY, threshold=(30, 80))
        img.draw_image(d, 0, 0, x_scale=1.15, y_scale=3)
    elif DEBUG_STEP == 3:
        d = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
        d.histeq()
        d.find_edges(image.EDGE_CANNY, threshold=(30, 80))
        d.dilate(2)
        img.draw_image(d, 0, 0, x_scale=1.15, y_scale=3)
    elif DEBUG_STEP == 4:
        d = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))
        d.histeq()
        d.find_edges(image.EDGE_CANNY, threshold=(30, 80))
        d.dilate(2)
        d.erode(1)
        img.draw_image(d, 0, 0, x_scale=1.15, y_scale=3)
    elif DEBUG_STEP == 5:
        img.draw_line((CENTER_X, ROI_Y, CENTER_X, ROI_Y + ROI_H), color=128)
        if blobs:
            best = blobs[0]
            best_score = 0
            for b in blobs:
                if b.area() > 0:
                    r = 1.0 - abs(b.elongation() - 1.0)
                    if r < 0: r = 0
                    s = b.area() * r
                    if s > best_score:
                        best_score = s
                        best = b
            cx = best.cx() + ROI_X
            cy = best.cy() + ROI_Y
            img.draw_cross(cx, cy, color=255, size=10)

    img.draw_string(5, 5, labels[DEBUG_STEP], color=255, scale=1.5)

    # ── UART output: $B,<signed position in 0.1mm># ──
    position_0p1mm = 0
    valid = 0
    if blobs:
        best = blobs[0]
        best_score = 0
        for b in blobs:
            if b.area() > 0:
                r = 1.0 - abs(b.elongation() - 1.0)
                if r < 0: r = 0
                s = b.area() * r
                if s > best_score:
                    best_score = s
                    best = b
        ball_cx = best.cx() + ROI_X
        position_0p1mm = int((ball_cx - CENTER_X) * SCALE_MM * 10)
        position_0p1mm = max(-32768, min(32767, position_0p1mm))
        valid = 1

    # UART frame: $B,<signed position in 0.1mm>#. Send only confirmed balls;
    # no frame for 120 ms makes the STM32 enter its camera-lost safety state.
    now = time.ticks_ms()
    if valid and time.ticks_diff(now, last_uart_ms) >= 40:
        uart.write("$B,%d#" % position_0p1mm)
        last_uart_ms = now
