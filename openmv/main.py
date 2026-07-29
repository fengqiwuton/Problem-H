# 车载平衡滚球 - asyncio非阻塞: MJPEG推流 + 球检测 + UART
import asyncio
import csi
import gc
import network
from microdot import Microdot, Response
from pyb import UART

# ── 初始化 ──
uart = UART(3, 115200, timeout_char=10)
app = Microdot()
BOUNDARY = b'frame'

CENTER_X = 160
SCALE_MM = 0.38
BALL_THRESHOLD = (0, 80, -30, 30, -20, 40)
latest_ball_mm = 0
latest_ball_cx = 0
has_ball = False

# ── 非阻塞摄像头 ──
class AsyncCSI:
    def __init__(self):
        self._csi = csi.CSI()
        self._csi.reset()
        self._csi.pixformat(csi.RGB565)
        self._csi.framesize(csi.QVGA)
        self._csi.skip_frames(20)

    def __getattr__(self, name):
        return getattr(self._csi, name)

    async def snapshot(self):
        while True:
            img = self._csi.snapshot(blocking=False)
            if img is not None:
                return img
            await asyncio.sleep_ms(0)

csi0 = AsyncCSI()

# ── MJPEG流迭送器 ──
class FrameStream:
    def __aiter__(self):
        return self

    async def __anext__(self):
        img = await csi0.snapshot()
        if has_ball:
            img.draw_cross(latest_ball_cx, 120, color=(255, 0, 0), size=8)
        img.draw_line((CENTER_X, 0, CENTER_X, 240), color=(0, 255, 0))
        jpeg = bytes(img.compress(quality=50).bytearray())
        return (b'--' + BOUNDARY + b'\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')

# ── 球检测协程(独立运行, UART发给STM32) ──
async def ball_detect():
    global latest_ball_mm, latest_ball_cx, has_ball
    last_uart_ms = 0

    while True:
        gc.collect()
        img = await csi0.snapshot()
        blobs = img.find_blobs([BALL_THRESHOLD], pixels_threshold=80,
                               area_threshold=80, merge=True, margin=5)

        if blobs:
            best = blobs[0]
            for b in blobs:
                if b.area() > best.area():
                    best = b

            latest_ball_cx = best.cx()
            latest_ball_mm = int((best.cx() - CENTER_X) * SCALE_MM)
            has_ball = True

            now = asyncio.ticks_ms()
            if asyncio.ticks_diff(now, last_uart_ms) > 50:
                uart.write("$B,%d#" % latest_ball_mm)
                last_uart_ms = now
        else:
            has_ball = False

        await asyncio.sleep_ms(5)

# ── HTTP路由 ──
@app.get('/')
async def index(request):
    return """<!DOCTYPE html>
<html><body style="margin:0;background:#000">
<img src="/stream.jpg" style="width:100%">
</body></html>"""

@app.get('/stream.jpg')
async def stream(request):
    return Response(
        body=FrameStream(),
        headers={'Content-Type': b'multipart/x-mixed-replace; boundary=' + BOUNDARY},
    )

# ── WiFi ──
wlan = network.WLAN(network.AP_IF)
wlan.config(essid="BallCar", password="12345678", channel=1)
wlan.active(True)
while not wlan.active():
    pass
ip = wlan.ifconfig()[0]

# ── 启动 ──
async def main():
    asyncio.create_task(ball_detect())
    await app.start_server(host='0.0.0.0', port=80)

print("WiFi: BallCar / 12345678")
print("Open http://" + ip + ":80")
asyncio.run(main())
