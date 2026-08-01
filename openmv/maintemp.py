# 钢球检测: QQVGA高速 + 管壁定位 + binary + find_circles + WiFi推流
import sensor, image, time, socket, network
from pyb import UART

BALL_BINARY = (85, 130)
LEARN_FRAMES = 30
RELOCK_EVERY = 200

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

tube_roi = (0, 30, 160, 60)
y1_sum = 0; y2_sum = 0; sample_cnt = 0
frame_n = 0
locked = False

# ===== WiFi 推流 =====
_wifi_client = None
_wifi_sock = None

def wifi_init():
    global _wifi_sock
    wlan = network.WLAN(network.AP_IF)
    wlan.config("BallCar", channel=1)
    wlan.active(True)
    for i in range(20):
        if wlan.active():
            break
        time.sleep_ms(200)
    ip = wlan.ifconfig()[0]
    _wifi_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    _wifi_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _wifi_sock.bind(('0.0.0.0', 8080))
    _wifi_sock.listen(1)
    _wifi_sock.setblocking(False)
    print("WiFi: http://%s:8080" % ip)

def wifi_stream(img):
    global _wifi_client, _wifi_sock
    try:
        _wifi_client, addr = _wifi_sock.accept()
        _wifi_client.send(
            b'HTTP/1.1 200 OK\r\n'
            b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n'
            b'\r\n'
        )
    except OSError:
        pass
    if _wifi_client:
        try:
            jpeg = img.compress(quality=30)
            _wifi_client.send(
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n'
                + jpeg + b'\r\n'
            )
        except OSError:
            _wifi_client.close()
            _wifi_client = None

wifi_init()
# =====================

while True:
    clock.tick()
    frame_n += 1
    img = sensor.snapshot()

    # ── 管壁重锁 ──
    if not locked or frame_n % RELOCK_EVERY == 0:
        if frame_n % RELOCK_EVERY == 0:
            y1_sum = 0; y2_sum = 0; sample_cnt = 0
        row_bright = [0]*120
        for y in range(120):
            cnt = 0
            for x in range(160):
                if img.get_pixel(x, y) > 180:
                    cnt += 1
            row_bright[y] = cnt
        candidates = [y for y in range(120) if row_bright[y] > 128]
        if len(candidates) < 2:
            candidates = sorted(range(120), key=lambda y: row_bright[y], reverse=True)[:8]
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

    # ── 球检测 ──
    r = tube_roi
    rx, ry, rw, rh = r[0]+3, r[1]+3, r[2]-6, r[3]-6
    if rh < 10: rx, ry, rw, rh = r[0], r[1], r[2], r[3]
    roi = img.copy(roi=(rx, ry, rw, rh))
    roi.binary([BALL_BINARY])
    roi.dilate(4)
    roi.erode(2)

    blobs = roi.find_blobs([(200, 255)], pixels_threshold=6, merge=True)
    ball_found = False; ball_cx = 0
    if blobs:
        best = None; best_score = 0
        for b in blobs:
            if b.cx() < 10 or b.cx() > 150: continue
            rnd = 1.0 - abs(b.elongation() - 1.0)
            if rnd < 0: rnd = 0
            score = b.area() * rnd
            if score > best_score: best_score = score; best = b
        if best and best.elongation() < 2.0:
            ball_cx = best.cx() + rx
            ball_found = True

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
            ball_01mm = int((ball_cx - CENTER_X) * SCALE_MM * 10)
            uart.write("$B,%d#" % ball_01mm)
            last_uart_ms = now
    elif time.ticks_diff(now, last_uart_ms) > 100:
        uart.write("$L#")
        last_uart_ms = now

    wifi_stream(img)                     # ← 推流
    print("FPS: %.1f" % clock.fps())
