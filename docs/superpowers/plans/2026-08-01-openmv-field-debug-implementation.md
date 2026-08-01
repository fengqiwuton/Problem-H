# OpenMV Field Debug Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the OpenMV preview bright and observable while showing pipe/ball boxes and printing useful coordinates and FPS without flooding the terminal.

**Architecture:** Keep the existing grayscale QQVGA detection and UART path. During startup, let sensor auto controls settle and then freeze their measured values; during the loop, render debug overlays every frame and print a diagnostic line at a separate 100 ms cadence.

**Tech Stack:** OpenMV MicroPython (`sensor`, `time`, `pyb.UART`), Python static tests, pytest.

## Global Constraints

- Keep OpenMV UART at `UART(3, 9600)`.
- Keep valid packets as `$B,<whole_mm>#` and lost packets as `$L#`.
- Keep QQVGA grayscale processing and adaptive ROI-relative ball thresholding.
- Terminal diagnostics must not change UART transmission cadence.
- Do not add dependencies to the OpenMV target.

---

### Task 1: Startup Auto-Tuning and Lock

**Files:**
- Modify: `tests/test_openmv_main_static.py`
- Modify: `openmv/main.py`

**Interfaces:**
- Consumes: OpenMV `sensor.set_auto_gain`, `sensor.set_auto_exposure`, `sensor.get_gain_db`, and `sensor.get_exposure_us`.
- Produces: startup constants `AUTO_TUNE_MS`, `GAIN_DB_MIN`, `GAIN_DB_MAX`, `EXPOSURE_US_MIN`, and `EXPOSURE_US_MAX`; locked camera settings before the main loop.

- [x] **Step 1: Write the failing startup-tuning test**

```python
def test_camera_auto_tunes_then_locks_brightness():
    text = read_main()
    auto_gain = text.index("sensor.set_auto_gain(True)")
    auto_exposure = text.index("sensor.set_auto_exposure(True)")
    lock_gain = text.index("sensor.set_auto_gain(False, gain_db=locked_gain_db)")
    lock_exposure = text.index(
        "sensor.set_auto_exposure(False, exposure_us=locked_exposure_us)"
    )

    assert "AUTO_TUNE_MS = 1500" in text
    assert "sensor.get_gain_db()" in text
    assert "sensor.get_exposure_us()" in text
    assert auto_gain < lock_gain
    assert auto_exposure < lock_exposure
```

- [x] **Step 2: Run the test and confirm RED**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py::test_camera_auto_tunes_then_locks_brightness -q`

Expected: FAIL because startup auto-tuning is absent.

- [x] **Step 3: Implement startup auto-tuning**

Replace the fixed 12 ms/10 dB setup with:

```python
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.skip_frames(time=AUTO_TUNE_MS)
locked_gain_db = clamp(int(sensor.get_gain_db()), GAIN_DB_MIN, GAIN_DB_MAX)
locked_exposure_us = clamp(
    int(sensor.get_exposure_us()), EXPOSURE_US_MIN, EXPOSURE_US_MAX
)
sensor.set_auto_gain(False, gain_db=locked_gain_db)
sensor.set_auto_exposure(False, exposure_us=locked_exposure_us)
sensor.skip_frames(time=200)
```

- [x] **Step 4: Run the startup-tuning test and confirm GREEN**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py::test_camera_auto_tunes_then_locks_brightness -q`

Expected: PASS.

### Task 2: Overlay Boxes and Throttled Terminal Status

**Files:**
- Modify: `tests/test_openmv_main_static.py`
- Modify: `openmv/main.py`

**Interfaces:**
- Consumes: `pipe_roi`, selected `ball`, adaptive threshold `th`, `clock.fps()`, and `packet_for_ball` distance conversion.
- Produces: `draw_debug(img, pipe_roi, search_roi, ball, fps)` overlays and 100 ms terminal status lines.

- [x] **Step 1: Write failing debug-output tests**

```python
def test_field_debug_draws_pipe_and_ball_boxes():
    text = read_main()
    assert "DEBUG_DRAW = True" in text
    assert "img.draw_rectangle(pipe_roi" in text
    assert "img.draw_rectangle(ball.rect()" in text


def test_terminal_status_is_enabled_and_rate_limited():
    text = read_main()
    assert "PRINT_STATUS = True" in text
    assert "STATUS_PERIOD_MS = 100" in text
    assert 'BALL=(%d,%d)' in text
    assert 'BALL=LOST' in text
    assert "last_status_ms" in text
```

- [x] **Step 2: Run both tests and confirm RED**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py -q`

Expected: the new debug tests FAIL because the ball rectangle and rate-limited status are absent.

- [x] **Step 3: Implement overlays and status output**

Set debug defaults to enabled, add `img.draw_rectangle(ball.rect(), color=255, thickness=2)`, preserve pipe/search rectangles and center markers, retain `th` from `find_ball`, and print at most once per `STATUS_PERIOD_MS` using `time.ticks_diff`.

- [x] **Step 4: Run all static tests and confirm GREEN**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py -q`

Expected: all tests PASS.

- [x] **Step 5: Run syntax and sample-image regression checks**

Run: `python -B -c "compile(open('openmv/main.py', encoding='utf-8').read(), 'openmv/main.py', 'exec'); print('syntax ok')"`

Expected: `syntax ok`.

Run the existing sample-image detector over every `openmv/capture_*.bmp` and `openmv/capture_*.jpg` and verify it still finds the ball in the supplied scene set.
