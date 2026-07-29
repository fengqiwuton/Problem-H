# 反光背景下钢球检测 - 直方图均衡化 + 腐蚀膨胀 + find_blobs/find_circles
import sensor
import image
import time
import gc
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

uart = UART(3, 115200, timeout_char=10)

# ── 校准参数 ──
CENTER_X = 160       # 摆杆凹槽中心x坐标
SCALE_MM = 0.38      # 像素→mm换算

# ── ROI 限制在摆杆凹槽区域 ──
ROI_X = 20
ROI_Y = 80
ROI_W = 280
ROI_H = 80

last_uart_ms = 0

print("Ball Detect Ready")

while True:
    gc.collect()
    img = sensor.snapshot()

    # ── Step1: 裁剪ROI ──
    roi = img.copy(roi=(ROI_X, ROI_Y, ROI_W, ROI_H))

    # ── Step2: 直方图均衡化 ──
    roi.histeq()

    # ── Step3: 二值化（钢球比背景暗） ──
    # 自适应阈值：先看均值，低于均值的视为钢球
    threshold = roi.get_histogram().get_threshold()
    threshold = (0, threshold)

    # ── Step4: 腐蚀去除亮点噪声 ──
    roi.binary([threshold])
    roi.erode(2)

    # ── Step5: 膨胀恢复球体形状 ──
    roi.dilate(3)

    # ── Step6: find_blobs找钢球 ──
    blobs = roi.find_blobs([(0, 127)], pixels_threshold=20, area_threshold=20,
                            merge=True, margin=5)

    ball_found = False
    ball_cx = 0

    if blobs:
        # 选面积最大、最圆的
        best = blobs[0]
        best_score = 0
        for b in blobs:
            # 偏心率小 = 更圆
            roundness = 1.0 - abs(b.elongation() - 1.0)
            score = b.area() * roundness
            if score > best_score:
                best_score = score
                best = b

        # 在原图上画标记
        cx = best.cx() + ROI_X
        cy = best.cy() + ROI_Y
        img.draw_cross(cx, cy, color=(255, 0, 0), size=8)
        img.draw_circle(cx, cy, int(best.w() / 2), color=(255, 0, 0))

        ball_cx = cx
        ball_found = True

    # ── Step7: find_circles 辅助检测 ──
    if not ball_found:
        # 在原图上用霍夫圆检测
        circles = img.find_circles(
            threshold=3500,
            x_margin=10, y_margin=10,
            r_margin=5,
            r_min=3, r_max=15,
            roi=(ROI_X, ROI_Y, ROI_W, ROI_H)
        )
        if circles:
            c = circles[0]
            img.draw_circle(c.x(), c.y(), c.r(), color=(0, 255, 0))
            ball_cx = c.x()
            ball_found = True

    # ── 画中心参考线 ──
    img.draw_line((CENTER_X, ROI_Y, CENTER_X, ROI_Y + ROI_H), color=(0, 255, 0))

    # ── UART发送 ──
    now = time.ticks_ms()
    if ball_found and time.ticks_diff(now, last_uart_ms) > 50:
        ball_mm = int((ball_cx - CENTER_X) * SCALE_MM)
        uart.write("$B,%d#" % ball_mm)
        last_uart_ms = now

    time.sleep_ms(10)
