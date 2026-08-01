# OpenMV Uneven Lighting Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reacquire the steel ball on the darker left bright-pipe section without reducing normal tracking frame rate, and draw a pipe box that matches the physical pipe.

**Architecture:** Preserve the current single-ROI detector as the fast path. If it misses, search four overlapping horizontal tiles with independent thresholds and no blob merging; derive the preview-only pipe box from the padded detection ROI.

**Tech Stack:** OpenMV MicroPython, Python AST helper loader, pytest.

## Global Constraints

- Keep OpenMV UART at `UART(3, 9600)`.
- Keep valid packets as `$B,<whole_mm>#` and lost packets as `$L#`.
- Exclude 4 pixels at each horizontal image end and do not attempt to identify a ball inside the black left end cap.
- Use four fallback tiles with 6-pixel expansion on both sides.
- Run tiled fallback only after the fast path misses.
- Keep existing blob geometry filters and target dependencies unchanged.

---

### Task 1: Physical Pipe Display Box

**Files:**
- Modify: `tests/test_openmv_main_static.py`
- Modify: `openmv/main.py`

**Interfaces:**
- Consumes: padded `(x, y, w, h)` pipe detection ROI.
- Produces: `pipe_display_roi(pipe_roi) -> (x, y, w, h)` used only by `draw_debug`.

- [x] **Step 1: Write failing display-box tests**

```python
def test_pipe_display_box_trims_detection_margin():
    helpers = load_main_helpers()
    assert helpers["pipe_display_roi"]((0, 54, 160, 26)) == (4, 62, 152, 16)
```

Update the existing draw test to assert that `(4, 50, 152, 20)` is drawn for
the padded `(0, 40, 160, 32)` ROI and that the padded ROI itself is not drawn.

- [x] **Step 2: Run tests and confirm RED**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py -q`

Expected: FAIL because `pipe_display_roi` does not exist and the padded ROI is still drawn.

- [x] **Step 3: Implement the display-only ROI**

```python
def pipe_display_roi(pipe_roi):
    x, y, w, h = pipe_roi
    top_trim = h // 3
    return clip_roi(x + PIPE_USABLE_MARGIN_X,
                    y + top_trim,
                    w - PIPE_USABLE_MARGIN_X * 2,
                    h - top_trim - PIPE_DRAW_BOTTOM_TRIM)
```

Change `draw_debug` to draw `pipe_display_roi(pipe_roi)`.

- [x] **Step 4: Run tests and confirm GREEN**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py -q`

Expected: display-box tests PASS.

### Task 2: Local-Tile Lost-Ball Fallback

**Files:**
- Modify: `tests/test_openmv_main_static.py`
- Modify: `openmv/main.py`

**Interfaces:**
- Consumes: the full bright-section search ROI from `make_search_roi`.
- Produces: `make_tile_rois(search_roi) -> list[(x, y, w, h)]` and a fallback branch in `find_ball` that returns the highest-scoring locally thresholded candidate.

- [x] **Step 1: Write failing tile geometry test**

```python
def test_tile_rois_cover_bright_pipe_with_overlap():
    helpers = load_main_helpers()
    assert helpers["make_tile_rois"]((4, 60, 152, 17)) == [
        (4, 60, 44, 17),
        (36, 60, 50, 17),
        (74, 60, 50, 17),
        (112, 60, 44, 17),
    ]
```

- [x] **Step 2: Write failing fallback behavior test**

```python
def test_find_ball_uses_unmerged_local_fallback_after_fast_miss():
    helpers = load_main_helpers()

    class LiveBall:
        def w(self): return 4
        def h(self): return 7
        def pixels(self): return 21
        def density(self): return 0.75
        def y(self): return 68
        def cx(self): return 20

    ball = LiveBall()

    class FakeImage:
        def __init__(self):
            self.calls = []

        def find_blobs(self, thresholds, roi, pixels_threshold,
                       area_threshold, merge, margin):
            self.calls.append((roi, merge, margin))
            if not merge and roi == (4, 60, 44, 17):
                return [ball]
            return []

    image = FakeImage()
    helpers["dark_threshold"] = lambda img, roi: 100
    found, search, threshold = helpers["find_ball"](
        image, (0, 54, 160, 26), -1, 8
    )

    assert found is ball
    assert search == (4, 60, 152, 17)
    assert threshold == 100
    assert image.calls[0][1:] == (True, 2)
    assert any(call[1:] == (False, 0) for call in image.calls[1:])
```

- [x] **Step 3: Run tests and confirm RED**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py -q`

Expected: FAIL because tile generation and fallback do not exist.

- [x] **Step 4: Implement tile generation and fallback**

Add `PIPE_TILE_COUNT = 4`, `PIPE_TILE_OVERLAP = 6`, and a pure
`make_tile_rois(search_roi)` helper. Extract the current threshold, blob search,
and scoring loop into
`best_blob_in_roi(img, blob_roi, score_roi, smooth_x, merge_blobs)`, returning
`(best_blob, best_score, threshold)`. Call it once with the fast ROI and
`merge_blobs=True`, then call it for every local tile with `merge_blobs=False`
only when the fast result is absent.

- [x] **Step 5: Run full tests and syntax verification**

Run: `python -B -m pytest -p no:cacheprovider -q`

Expected: all tests PASS.

Run: `python -B -c "compile(open('openmv/main.py', encoding='utf-8').read(), 'openmv/main.py', 'exec'); print('syntax ok')"`

Expected: `syntax ok`.

- [x] **Step 6: Re-evaluate supplied live frames**

Downscale the supplied IDE frames to QQVGA, verify the left bright-section tile uses its own lower threshold, and verify the second frame yields a valid candidate. The first frame at the black end cap is outside the required range.
