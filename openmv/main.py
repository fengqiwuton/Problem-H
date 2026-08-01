# Robust OpenMV ball tracking for the bright horizontal pipe.
# UART protocol to STM32: "$B,<whole_mm>#" when found, "$L#" when lost.

import sensor
import time
from pyb import UART


# ---------------- Tunable constants ----------------

IMG_W = 160
WIFI_STREAM_ENABLED = False

if WIFI_STREAM_ENABLED:
    try:
        import socket
        import network
    except Exception:
        socket = None
        network = None
else:
    socket = None
    network = None
IMG_H = 120
CENTER_X = 80

# Measure on the real pipe. Current sample set is close to 0.76 mm/pixel.
MM_PER_PIXEL = 0.76

UART_ID = 3
UART_BAUD = 9600
UART_FOUND_PERIOD_MS = 35
UART_LOST_PERIOD_MS = 100

DEBUG_DRAW = True
PRINT_STATUS = True
STATUS_PERIOD_MS = 100

# Let auto control find a usable brightness at startup, then lock it.
AUTO_TUNE_MS = 1500
LOCK_SETTLE_MS = 200
GAIN_DB_MIN = 0
GAIN_DB_MAX = 24
EXPOSURE_US_MIN = 3000
EXPOSURE_US_MAX = 30000

# Pipe search is expensive, so run it only when needed.
PIPE_RELOCK_FRAMES = 120
PIPE_LOST_RELOCK = 8
PIPE_SCAN_X_STEP = 4
PIPE_BRIGHT_RATIO = 0.68
PIPE_MIN_H = 8
PIPE_MAX_H = 36
PIPE_MARGIN_Y = 5
PIPE_SEARCH_BOTTOM_MARGIN = 3
PIPE_USABLE_MARGIN_X = 4
PIPE_DRAW_BOTTOM_TRIM = 2
PIPE_TILE_COUNT = 4
PIPE_TILE_OVERLAP = 6

# Ball geometry in QQVGA. Tune if the camera distance changes a lot.
BALL_MIN_PIXELS = 5
BALL_MAX_PIXELS = 260
BALL_MIN_WH = 3
BALL_MAX_WH = 24
BALL_MAX_ASPECT_X100 = 230
BALL_MIN_DENSITY_X100 = 28
DARK_MARGIN = 22
DARK_MAX = 175

TRACK_HALF_W = 28
TRACK_HALF_W_LOST_STEP = 8
TRACK_USE_LOST_FRAMES = 4
SMOOTH_OLD = 3
SMOOTH_NEW = 1


# ---------------- Small helpers ----------------

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def clip_roi(x, y, w, h):
    if x < 0:
        w += x
        x = 0
    if y < 0:
        h += y
        y = 0
    if x + w > IMG_W:
        w = IMG_W - x
    if y + h > IMG_H:
        h = IMG_H - y
    if w < 1:
        w = 1
    if h < 1:
        h = 1
    return (int(x), int(y), int(w), int(h))


def percentile_value(hist, p, fallback):
    try:
        item = hist.get_percentile(p)
        try:
            return int(item.value())
        except Exception:
            return int(item)
    except Exception:
        return int(fallback)


def blob_pixels(blob):
    try:
        return blob.pixels()
    except Exception:
        return blob.area()


def blob_density_x100(blob):
    try:
        return int(blob.density() * 100)
    except Exception:
        area = blob.area()
        if area <= 0:
            return 0
        return int(blob_pixels(blob) * 100 / area)


def packet_for_ball(x_px):
    offset_mm = int((x_px - CENTER_X) * MM_PER_PIXEL)
    return "$B,%d#" % offset_mm


def packet_lost():
    return "$L#"


def auto_tune_and_lock(camera):
    camera.set_auto_gain(True)
    camera.set_auto_exposure(True)
    camera.skip_frames(time=AUTO_TUNE_MS)

    gain_db = clamp(int(camera.get_gain_db()), GAIN_DB_MIN, GAIN_DB_MAX)
    exposure_us = clamp(int(camera.get_exposure_us()),
                        EXPOSURE_US_MIN,
                        EXPOSURE_US_MAX)

    camera.set_auto_gain(False, gain_db=gain_db)
    camera.set_auto_exposure(False, exposure_us=exposure_us)
    camera.skip_frames(time=LOCK_SETTLE_MS)
    return gain_db, exposure_us


def format_status(fps, ball_x, ball_y, offset_mm, pipe_roi, threshold):
    if ball_x is None:
        return "FPS=%.1f BALL=LOST PIPE=%s TH=%d" % (
            fps, str(pipe_roi), threshold)
    return "FPS=%.1f BALL=(%d,%d) MM=%d PIPE=%s TH=%d" % (
        fps, ball_x, ball_y, offset_mm, str(pipe_roi), threshold)


def pipe_display_roi(pipe_roi):
    x, y, w, h = pipe_roi
    top_trim = h // 3
    return clip_roi(x + PIPE_USABLE_MARGIN_X,
                    y + top_trim,
                    w - PIPE_USABLE_MARGIN_X * 2,
                    h - top_trim - PIPE_DRAW_BOTTOM_TRIM)


# ---------------- Vision pipeline ----------------

def find_pipe_roi(img, old_roi):
    row_score = [0] * IMG_H
    best_y = 0
    best_score = -1

    for y in range(0, IMG_H):
        s = 0
        for x in range(0, IMG_W, PIPE_SCAN_X_STEP):
            s += img.get_pixel(x, y)
        row_score[y] = s
        if s > best_score:
            best_score = s
            best_y = y

    if best_score <= 0:
        if old_roi:
            return old_roi
        return (0, 40, IMG_W, 40)

    threshold = int(best_score * PIPE_BRIGHT_RATIO)
    groups = []
    start = -1

    for y in range(0, IMG_H):
        if row_score[y] >= threshold:
            if start < 0:
                start = y
        elif start >= 0:
            groups.append((start, y - 1))
            start = -1
    if start >= 0:
        groups.append((start, IMG_H - 1))

    best_group = None
    best_group_score = -1
    for g in groups:
        h = g[1] - g[0] + 1
        if h < 2 or h > PIPE_MAX_H + PIPE_MARGIN_Y * 2:
            continue
        score = h * 1000
        if old_roi:
            old_cy = old_roi[1] + old_roi[3] // 2
            cy = (g[0] + g[1]) // 2
            score -= abs(cy - old_cy) * 20
        if score > best_group_score:
            best_group_score = score
            best_group = g

    if best_group is None:
        y1 = best_y
        y2 = best_y
        while y1 > 0 and row_score[y1 - 1] >= threshold:
            y1 -= 1
        while y2 < IMG_H - 1 and row_score[y2 + 1] >= threshold:
            y2 += 1
    else:
        y1 = best_group[0]
        y2 = best_group[1]

    h = y2 - y1 + 1
    if h < PIPE_MIN_H:
        mid = (y1 + y2) // 2
        y1 = mid - PIPE_MIN_H // 2
        y2 = y1 + PIPE_MIN_H - 1

    y1 -= PIPE_MARGIN_Y
    y2 += PIPE_MARGIN_Y
    new_roi = clip_roi(0, y1, IMG_W, y2 - y1 + 1)

    if old_roi:
        y = (old_roi[1] * 3 + new_roi[1]) // 4
        h = (old_roi[3] * 3 + new_roi[3]) // 4
        return clip_roi(0, y, IMG_W, h)

    return new_roi


def make_search_roi(pipe_roi, smooth_x, lost_frames):
    x, y, w, h = pipe_roi
    usable_left = x + PIPE_USABLE_MARGIN_X
    usable_right = x + w - PIPE_USABLE_MARGIN_X
    inner_margin_top = 3
    if h > 16:
        inner_margin_top = h // 4
        if inner_margin_top > 8:
            inner_margin_top = 8

    inner_y = y + inner_margin_top
    inner_h = h - inner_margin_top - PIPE_SEARCH_BOTTOM_MARGIN
    if inner_h < 5:
        inner_y = y
        inner_h = h

    if smooth_x >= 0 and lost_frames <= TRACK_USE_LOST_FRAMES:
        half_w = TRACK_HALF_W + lost_frames * TRACK_HALF_W_LOST_STEP
        track_left = smooth_x - half_w
        track_right = smooth_x + half_w
        if track_left < usable_left:
            track_left = usable_left
        if track_right > usable_right:
            track_right = usable_right
        return clip_roi(track_left,
                        inner_y,
                        track_right - track_left,
                        inner_h)

    return clip_roi(usable_left,
                    inner_y,
                    usable_right - usable_left,
                    inner_h)


def make_tile_rois(search_roi):
    x, y, w, h = search_roi
    tiles = []
    right_limit = x + w

    for i in range(PIPE_TILE_COUNT):
        core_left = x + (w * i) // PIPE_TILE_COUNT
        core_right = x + (w * (i + 1)) // PIPE_TILE_COUNT
        tile_left = core_left - PIPE_TILE_OVERLAP
        tile_right = core_right + PIPE_TILE_OVERLAP
        if tile_left < x:
            tile_left = x
        if tile_right > right_limit:
            tile_right = right_limit
        tiles.append((tile_left, y, tile_right - tile_left, h))

    return tiles


def dark_threshold(img, roi):
    hist = img.get_histogram(roi=roi)
    p08 = percentile_value(hist, 0.08, 70)
    p50 = percentile_value(hist, 0.50, 120)
    hi = p08 + 18

    if hi > p50 - DARK_MARGIN:
        hi = p50 - DARK_MARGIN
    hi = clamp(hi, 15, DARK_MAX)
    return int(hi)


def score_blob(blob, search_roi, smooth_x):
    bw = blob.w()
    bh = blob.h()
    if bw < BALL_MIN_WH or bh < BALL_MIN_WH:
        return -1
    if bw > BALL_MAX_WH or bh > BALL_MAX_WH:
        return -1

    pixels = blob_pixels(blob)
    if pixels < BALL_MIN_PIXELS or pixels > BALL_MAX_PIXELS:
        return -1

    small = bw if bw < bh else bh
    large = bh if bw < bh else bw
    if small <= 0:
        return -1
    aspect_x100 = int(large * 100 / small)
    if aspect_x100 > BALL_MAX_ASPECT_X100:
        return -1

    density_x100 = blob_density_x100(blob)
    if density_x100 < BALL_MIN_DENSITY_X100:
        return -1

    # Reject pipe borders and ROI clipping artifacts.
    if blob.y() <= search_roi[1] + 1:
        return -1
    if blob.y() + blob.h() >= search_roi[1] + search_roi[3] - 1:
        return -1

    score = pixels * 10 + density_x100 * 2 - abs(bw - bh) * 8
    if smooth_x >= 0:
        score -= abs(blob.cx() - smooth_x) * 2
    return score


def best_blob_in_roi(img, blob_roi, score_roi, smooth_x, merge_blobs):
    th = dark_threshold(img, blob_roi)
    margin = 2 if merge_blobs else 0
    blobs = img.find_blobs([(0, th)],
                           roi=blob_roi,
                           pixels_threshold=BALL_MIN_PIXELS,
                           area_threshold=BALL_MIN_PIXELS,
                           merge=merge_blobs,
                           margin=margin)

    best = None
    best_score = -1
    for b in blobs:
        s = score_blob(b, score_roi, smooth_x)
        if s > best_score:
            best_score = s
            best = b

    return best, best_score, th


def find_ball(img, pipe_roi, smooth_x, lost_frames):
    search_roi = make_search_roi(pipe_roi, smooth_x, lost_frames)
    best, best_score, th = best_blob_in_roi(img,
                                            search_roi,
                                            search_roi,
                                            smooth_x,
                                            True)

    if best is None:
        full_roi = make_search_roi(pipe_roi, -1, TRACK_USE_LOST_FRAMES + 1)
        for tile_roi in make_tile_rois(full_roi):
            candidate, candidate_score, tile_th = best_blob_in_roi(
                img, tile_roi, full_roi, smooth_x, False)
            if candidate_score > best_score:
                best = candidate
                best_score = candidate_score
                th = tile_th

    return best, search_roi, th


def draw_debug(img, pipe_roi, search_roi, ball, fps):
    img.draw_rectangle(pipe_display_roi(pipe_roi), color=128)
    img.draw_line((CENTER_X, pipe_roi[1], CENTER_X, pipe_roi[1] + pipe_roi[3]), color=160)
    if ball:
        img.draw_rectangle(ball.rect(), color=255, thickness=2)
        r = (ball.w() + ball.h()) // 4
        img.draw_circle(ball.cx(), ball.cy(), r, color=255, thickness=2)
        img.draw_cross(ball.cx(), ball.cy(), color=255, size=8)
    img.draw_string(2, 2, "FPS:%d" % int(fps), color=255)


# ---------------- WiFi ----------------

_wifi_client = None
_wifi_sock = None

def wifi_init():
    global _wifi_sock

    if not WIFI_STREAM_ENABLED or socket is None or network is None:
        return False

    try:
        wlan = network.WLAN(network.AP_IF)
        wlan.config(ssid="BallCar", password="12345678", channel=1)
        wlan.active(True)
        for i in range(30):
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
        return True
    except Exception:
        if _wifi_sock:
            try:
                _wifi_sock.close()
            except Exception:
                pass
        _wifi_sock = None
        return False

def wifi_stream(img):
    global _wifi_client, _wifi_sock

    if not WIFI_STREAM_ENABLED or _wifi_sock is None:
        return

    try:
        client, addr = _wifi_sock.accept()
    except OSError:
        pass
    else:
        try:
            client.setblocking(False)
            client.send(
                b'HTTP/1.1 200 OK\r\n'
                b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n'
                b'\r\n'
            )
            _wifi_client = client
        except OSError:
            try:
                client.close()
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
            try:
                _wifi_client.close()
            except OSError:
                pass
            _wifi_client = None

# ---------------- Runtime ----------------

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_whitebal(False)
locked_gain_db, locked_exposure_us = auto_tune_and_lock(sensor)
print("CAM GAIN=%ddB EXPOSURE=%dus" % (locked_gain_db, locked_exposure_us))

uart = UART(UART_ID, UART_BAUD, timeout_char=10)
clock = time.clock()

pipe_roi = (0, 40, IMG_W, 40)
smooth_x = -1
lost_frames = PIPE_LOST_RELOCK
frame_n = 0
last_uart_ms = 0
last_status_ms = 0
wifi_ready = False

if WIFI_STREAM_ENABLED:
    wifi_ready = wifi_init()

while True:
    clock.tick()
    frame_n += 1
    img = sensor.snapshot()

    need_pipe_lock = False
    if frame_n <= 5:
        need_pipe_lock = True
    elif frame_n % PIPE_RELOCK_FRAMES == 0:
        need_pipe_lock = True
    elif lost_frames >= PIPE_LOST_RELOCK:
        need_pipe_lock = True

    if need_pipe_lock:
        pipe_roi = find_pipe_roi(img, pipe_roi)

    ball, search_roi, th = find_ball(img, pipe_roi, smooth_x, lost_frames)
    now = time.ticks_ms()
    offset_mm = None

    if ball:
        x = ball.cx()
        if smooth_x < 0:
            smooth_x = x
        else:
            smooth_x = int((smooth_x * SMOOTH_OLD + x * SMOOTH_NEW) / (SMOOTH_OLD + SMOOTH_NEW))
        lost_frames = 0
        offset_mm = int((smooth_x - CENTER_X) * MM_PER_PIXEL)

        if time.ticks_diff(now, last_uart_ms) >= UART_FOUND_PERIOD_MS:
            uart.write(packet_for_ball(smooth_x))
            last_uart_ms = now
    else:
        if lost_frames < 255:
            lost_frames += 1
        if lost_frames > TRACK_USE_LOST_FRAMES:
            smooth_x = -1

        if time.ticks_diff(now, last_uart_ms) >= UART_LOST_PERIOD_MS:
            uart.write(packet_lost())
            last_uart_ms = now

    if WIFI_STREAM_ENABLED and wifi_ready:
        wifi_stream(img)

    if DEBUG_DRAW:
        draw_debug(img, pipe_roi, search_roi, ball, clock.fps())

    if PRINT_STATUS and time.ticks_diff(now, last_status_ms) >= STATUS_PERIOD_MS:
        if ball:
            print(format_status(clock.fps(),
                                ball.cx(),
                                ball.cy(),
                                offset_mm,
                                pipe_roi,
                                th))
        else:
            print(format_status(clock.fps(), None, None, None, pipe_roi, th))
        last_status_ms = now
