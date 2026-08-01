# OpenMV Ball Vision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a robust high-frame-rate OpenMV script for pipe detection, steel ball localization, distance estimation, and UART reporting.

**Architecture:** `openmv/main.py` owns the full embedded vision loop. It uses QQVGA grayscale images, periodic pipe ROI locking, local adaptive ball thresholding, blob scoring, short-term tracking, and fixed-rate UART output.

**Tech Stack:** OpenMV MicroPython, `sensor`, `image`, `time`, `pyb.UART`; optional PC-side sample image inspection with Python/OpenCV.

## Global Constraints

- OpenMV UART is `UART(3, 9600)`.
- Valid packet format is `$B,<mm>#`.
- Lost packet format is `$L#`.
- The packet payload is whole millimetres; STM32 converts to 0.1 mm internally.
- Use grayscale QQVGA for speed.
- Keep drawing behind a `DEBUG_DRAW` switch.
- Keep calibration constants at the top of `openmv/main.py`.

---

### Task 1: Replace OpenMV Runtime Script

**Files:**
- Modify: `openmv/main.py`

**Interfaces:**
- Consumes: sample image geometry from `openmv/capture_*.bmp`; STM32 protocol from `code/openmv_uart.c`.
- Produces: an OpenMV script that sends `$B,<mm>#` or `$L#`.

- [ ] **Step 1: Define constants and camera setup**

Configure QQVGA grayscale, UART 3 at 9600, camera exposure/gain defaults, center pixel, mm-per-pixel calibration, pipe search cadence, tracking ROI size, and debug drawing switch.

- [ ] **Step 2: Implement pipe ROI locking**

Scan row brightness only during startup, after repeated losses, or every configured interval. Select the brightest horizontal band, smooth it with the previous ROI, and clamp it to image bounds.

- [ ] **Step 3: Implement adaptive ball detection**

Inside the pipe ROI or tracking ROI, compute a local histogram percentile and search dark blobs with `find_blobs`. Score candidates by size, density, aspect ratio, and proximity to the previous position.

- [ ] **Step 4: Implement smoothing and distance output**

Smooth x with an IIR filter, calculate `int((x - CENTER_X) * MM_PER_PIXEL)`, and send whole millimetres using `$B,<mm>#`.

- [ ] **Step 5: Implement loss handling**

Send `$L#` at a slower cadence when no confident ball is found, expand the search ROI after losses, and force pipe re-lock after repeated losses.

- [ ] **Step 6: Add optional debug overlay**

When `DEBUG_DRAW` is true, draw the pipe rectangle, search ROI, center line, ball circle, crosshair, and FPS text.

### Task 2: Verify Against Current Project

**Files:**
- Inspect: `openmv/main.py`
- Inspect: `code/openmv_uart.c`

**Interfaces:**
- Consumes: UART protocol parser in STM32 code.
- Produces: confidence that OpenMV and STM32 use the same packet format and unit.

- [ ] **Step 1: Review generated OpenMV syntax**

Read `openmv/main.py` after editing and check for unsupported standard-Python features, missing imports, and unit mistakes.

- [ ] **Step 2: Confirm protocol compatibility**

Ensure found packets look like `$B,0#`, `$B,-35#`, `$B,42#`, and lost packets look like `$L#`.

- [ ] **Step 3: Note hardware calibration needs**

Record that `MM_PER_PIXEL` and exposure/gain may need on-device tuning because sample images cannot fully represent real lighting.
