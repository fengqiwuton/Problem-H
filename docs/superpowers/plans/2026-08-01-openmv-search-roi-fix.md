# OpenMV Search ROI Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect the live 4 x 7 pixel steel-ball candidate that is currently rejected at the bottom of the internal search ROI, while showing only one pipe box.

**Architecture:** Keep the existing pipe localization and blob filters. Change only the internal search ROI from symmetric vertical margins to a height-derived top margin and fixed 3-pixel bottom margin, then stop rendering that internal ROI in the IDE preview.

**Tech Stack:** OpenMV MicroPython, Python AST helper loader, pytest.

## Global Constraints

- Keep OpenMV UART at `UART(3, 9600)`.
- Keep valid packets as `$B,<whole_mm>#` and lost packets as `$L#`.
- Keep QQVGA grayscale processing and the existing edge, size, aspect-ratio, and density filters.
- Draw the pipe ROI and detected ball, but do not draw the internal search ROI.
- Do not add dependencies to the OpenMV target.

---

### Task 1: Asymmetric Search ROI Regression

**Files:**
- Modify: `tests/test_openmv_main_static.py`
- Modify: `openmv/main.py`

**Interfaces:**
- Consumes: `make_search_roi(pipe_roi, smooth_x, lost_frames)` and `score_blob(blob, search_roi, smooth_x)`.
- Produces: `PIPE_SEARCH_BOTTOM_MARGIN = 3` and a search ROI that preserves the strict top margin while extending its lower edge.

- [x] **Step 1: Write the failing live-frame regression test**

```python
def test_live_ball_candidate_fits_asymmetric_search_roi():
    helpers = load_main_helpers()
    search = helpers["make_search_roi"]((0, 54, 160, 29), -1, 8)

    class LiveBall:
        def w(self): return 4
        def h(self): return 7
        def pixels(self): return 21
        def density(self): return 0.75
        def x(self): return 81
        def y(self): return 68
        def cx(self): return 83

    assert search == (8, 61, 144, 19)
    assert helpers["score_blob"](LiveBall(), search, -1) > 0
```

- [x] **Step 2: Strengthen the overlay regression test**

Add `assert search not in image.rectangles` after calling `draw_debug`; it must still assert that the pipe and ball rectangles are present.

- [x] **Step 3: Run both tests and confirm RED**

Run: `python -B -m pytest -p no:cacheprovider tests/test_openmv_main_static.py -q`

Expected: FAIL because the current search ROI is `(8, 61, 144, 15)` and because it is still drawn.

- [x] **Step 4: Implement the minimal production correction**

In `make_search_roi`, compute `inner_margin_top` with the current height-derived rule, set `inner_margin_bottom = PIPE_SEARCH_BOTTOM_MARGIN`, and calculate `inner_h = h - inner_margin_top - inner_margin_bottom`. In `draw_debug`, remove only the `img.draw_rectangle(search_roi, ...)` call.

- [x] **Step 5: Run regression and full tests**

Run: `python -B -m pytest -p no:cacheprovider -q`

Expected: all tests PASS.

- [x] **Step 6: Run syntax and screenshot-derived candidate checks**

Run: `python -B -c "compile(open('openmv/main.py', encoding='utf-8').read(), 'openmv/main.py', 'exec'); print('syntax ok')"`

Expected: `syntax ok` and the live candidate no longer reports `edge_reject=True` when evaluated against the new search ROI.
