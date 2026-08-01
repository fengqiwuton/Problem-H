# 钢球检测: QQVGA高速 + 管壁定位 + binary + find_circles
import sensor, image, time
from pyb import UART

BALL_BINARY = (85, 130)
LEARN_FRAMES = 30
RELOCK_EVERY = 200   # 每200帧重锁, 减少开销

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)    # 160x120 高速
sensor.set_auto_gain(True, gain_db=20)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True, exposure_us=25000)
sensor.skip_frames(time=2000)

uart = UART(3, 9600, timeout_char=10)   # 9600匹配STM32软件串口
clock = time.clock()

CENTER_X = 80; SCALE_MM = 0.76  # 需实测标定!
last_uart_ms = 0

tube_roi = (0, 30, 160, 60)
y1_sum = 0; y2_sum = 0; sample_cnt = 0
frame_n = 0
locked = False

save_count = 0  # 保存前5帧

while True:
    clock.tick()
    frame_n += 1
    img = sensor.snapshot()

    # 保存前5帧原图
    if save_count < 5:
        try:
            img.save("capture_%d.bmp" % save_count)
            print("SAVED capture_%d.bmp OK" % save_count)
        except Exception as e:
            print("SAVE ERR: %s" % str(e))
        save_count += 1

    # ── 管壁重锁: 管壁=图里最亮的横条 ──
    if not locked or frame_n % RELOCK_EVERY == 0:
        if frame_n % RELOCK_EVERY == 0:
            y1_sum = 0; y2_sum = 0; sample_cnt = 0
        # 管壁=横跨画面亮条: 统计每行亮像素(>180)数
        row_bright = [0]*120
        for y in range(120):
            cnt = 0
            for x in range(160):
                if img.get_pixel(x, y) > 180:
                    cnt += 1
            row_bright[y] = cnt
        # 只取横跨>80%画面的行(>128像素)
        candidates = [y for y in range(120) if row_bright[y] > 128]
        if len(candidates) < 2:
            candidates = sorted(range(120), key=lambda y: row_bright[y], reverse=True)[:8]
        # 按位置分组
        candidates.sort()
        groups = []
        cur = [candidates[0]]
        for i in range(1, len(candidates)):
            if candidates[i] - cur[-1] < 6:
                cur.append(candidates[i])
            else:
                groups.append(sum(cur)//len(cur))
                cur = [candidates[i]]
        groups.append(sum(cur)//len(cur))

        if len(groups) >= 2:
            groups.sort()
            y1 = groups[0]; y2 = groups[-1]
            if y2 - y1 > 10:
                tube_roi = (0, y1-2, 160, y2 - y1 + 4)
                y1_sum += y1; y2_sum += y2; sample_cnt += 1
        if sample_cnt >= LEARN_FRAMES:
            y1a = y1_sum//sample_cnt; y2a = y2_sum//sample_cnt
            tube_roi = (0, y1a-2, 160, y2a - y1a + 4)
            locked = True

    # ── 球检测: ROI内缩避管壁 + 多膨胀让球变圆 + blob找最圆 ──
    r = tube_roi
    # 内缩3像素避开管壁边缘
    rx, ry, rw, rh = r[0]+3, r[1]+3, r[2]-6, r[3]-6
    if rh < 10: rx, ry, rw, rh = r[0], r[1], r[2], r[3]
    roi = img.copy(roi=(rx, ry, rw, rh))
    roi.binary([BALL_BINARY])
    roi.dilate(4)   # 多膨胀填成圆形
    roi.erode(2)

    # 先用blob找最圆的, 限制在画面中心附近(排除左边胶带)
    blobs = roi.find_blobs([(200, 255)], pixels_threshold=6, merge=True)
    ball_found = False; ball_cx = 0
    if blobs:
        best = None; best_score = 0
        for b in blobs:
            # 球在画面内(仅排除左右边缘10px)
            if b.cx() < 10 or b.cx() > 150: continue
            rnd = 1.0 - abs(b.elongation() - 1.0)
            if rnd < 0: rnd = 0
            score = b.area() * rnd
            if score > best_score: best_score = score; best = b
        if best and best.elongation() < 2.0:
            ball_cx = best.cx() + rx
            ball_found = True

    # ── 右下角调试: 显示处理后的ROI ──
    img.draw_image(roi, 100, 90, x_scale=0.35, y_scale=0.35)
    img.draw_rectangle(r, color=128)
    img.draw_line((CENTER_X, r[1], CENTER_X, r[1]+r[3]), color=128)

    now = time.ticks_ms()
    if ball_found:
        cy = best.cy() + ry
        r2 = int(best.w() / 2)
        img.draw_circle(ball_cx, cy, r2, color=255, thickness=2)
        img.draw_cross(ball_cx, cy, color=255, size=8)
        if time.ticks_diff(now, last_uart_ms) > 50:
            # 发送0.1mm单位 (STM32内部使用)
            ball_01mm = int((ball_cx - CENTER_X) * SCALE_MM * 10)
            uart.write("$B,%d#" % ball_01mm)
            last_uart_ms = now
    elif time.ticks_diff(now, last_uart_ms) > 100:
        uart.write("$L#")   # 没找到也发信号
        last_uart_ms = now

    print("FPS: %.1f" % clock.fps())
