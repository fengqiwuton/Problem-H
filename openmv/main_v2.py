# 钢球检测v2: 行亮度找管 → Otsu二值化 → 最大暗色块=球 (PC验证通过)
import sensor, image, time
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(True, gain_db=20)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True, exposure_us=25000)
sensor.skip_frames(time=2000)

uart = UART(3, 9600, timeout_char=10)
clock = time.clock()
CENTER_X = 80; SCALE_MM = 0.76
last_uart_ms = 0

# 管壁记忆: 每200帧重锁
tube_roi = (0, 30, 160, 60)
frame_n = 0

while True:
    clock.tick()
    frame_n += 1
    img = sensor.snapshot()

    # ── 管壁: 行亮度峰值(自适应) ──
    if frame_n % 200 == 0 or tube_roi[3] < 10:
        row_avg = [0]*120
        for y in range(120):
            s = 0
            for x in range(160):
                s += img.get_pixel(x, y)
            row_avg[y] = s // 160
        # 找最亮的连续区域
        mx_row = max(range(120), key=lambda y: row_avg[y])
        y1 = mx_row
        y2 = mx_row
        while y1 > 0  and row_avg[y1-1] > row_avg[mx_row]*0.7: y1 -= 1
        while y2 < 119 and row_avg[y2+1] > row_avg[mx_row]*0.7: y2 += 1
        if y2 - y1 > 8:
            tube_roi = (0, y1-2, 160, y2 - y1 + 4)

    # ── 管内自适应找球 ──
    r = tube_roi
    roi = img.copy(roi=(r[0], r[1], r[2], r[3]))
    if roi.width() < 1 or roi.height() < 1:
        continue

    # 球=比管内中值暗25以上的像素(自适应)
    hist = roi.get_histogram()
    med = hist.get_percentile(0.5)
    median = med.value() if hasattr(med, 'value') else int(med)
    lo = 0
    hi = max(median - 25, 10)
    roi.binary([(lo, hi)])
    roi.erode(1)
    roi.dilate(2)

    blobs = roi.find_blobs([(128, 255)], pixels_threshold=4, merge=True)
    ball_found = False; ball_cx = 0
    if blobs:
        best = max(blobs, key=lambda b: b.area())
        bx, by, bw, bh = best.x(), best.y(), best.w(), best.h()
        if best.area() > 5 and bw > 2 and bh > 2:
            if bx > 1 and by > 1 and bx+bw < roi.width()-1 and by+bh < roi.height()-1:
                ball_cx = best.cx() + r[0]
                ball_found = True

    img.draw_image(roi, 100, 90, x_scale=0.35, y_scale=0.35)

    # ── UART ──
    now = time.ticks_ms()
    if ball_found:
        cy = best.cy() + r[1]
        img.draw_circle(ball_cx, cy, int(best.w()/2), color=255, thickness=2)
        img.draw_cross(ball_cx, cy, color=255, size=8)
        if time.ticks_diff(now, last_uart_ms) > 50:
            pos_01mm = int((ball_cx - CENTER_X) * SCALE_MM * 10)
            uart.write("$B,%d#" % pos_01mm)
            last_uart_ms = now
    elif time.ticks_diff(now, last_uart_ms) > 100:
        uart.write("$L#")
        last_uart_ms = now

    img.draw_rectangle(r, color=128)
    img.draw_line((CENTER_X, r[1], CENTER_X, r[1]+r[3]), color=128)

    # 终端打印
    tc = (r[1] + r[1] + r[3]) // 2  # 管中心y
    if ball_found:
        print("Ball: cx=%d  Pipe: cy=%d  FPS:%.1f" % (ball_cx, tc, clock.fps()))
    else:
        print("LOST  Pipe: cy=%d  FPS:%.1f" % (tc, clock.fps()))
