import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "openmv" / "main.py"


def read_main():
    return MAIN.read_text(encoding="utf-8", errors="ignore")


def load_main_helpers():
    tree = ast.parse(read_main(), filename=str(MAIN))
    body = []
    for node in tree.body:
        is_constant = (
            isinstance(node, ast.Assign)
            and all(
                isinstance(target, ast.Name) and target.id.isupper()
                for target in node.targets
            )
        )
        if is_constant or isinstance(node, ast.FunctionDef):
            body.append(node)
    namespace = {}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(MAIN), "exec"), namespace)
    return namespace


def test_openmv_uart_protocol_uses_whole_millimetres():
    text = read_main()

    assert "UART_ID = 3" in text
    assert "UART_BAUD = 9600" in text
    assert "UART(UART_ID, UART_BAUD" in text
    assert "$B,%d#" in text
    assert "$L#" in text
    assert "SCALE_MM * 10" not in text
    assert "MM_PER_PIXEL * 10" not in text


def test_openmv_runtime_has_tunable_debug_draw_switch():
    text = read_main()

    assert "DEBUG_DRAW" in text


def test_camera_auto_tunes_then_locks_brightness():
    helpers = load_main_helpers()

    class FakeSensor:
        def __init__(self):
            self.calls = []

        def set_auto_gain(self, enabled, gain_db=None):
            self.calls.append(("gain", enabled, gain_db))

        def set_auto_exposure(self, enabled, exposure_us=None):
            self.calls.append(("exposure", enabled, exposure_us))

        def skip_frames(self, time):
            self.calls.append(("wait", time))

        def get_gain_db(self):
            self.calls.append(("read_gain",))
            return 30.0

        def get_exposure_us(self):
            self.calls.append(("read_exposure",))
            return 40000

    fake = FakeSensor()
    gain_db, exposure_us = helpers["auto_tune_and_lock"](fake)

    assert (gain_db, exposure_us) == (24, 30000)
    assert fake.calls == [
        ("gain", True, None),
        ("exposure", True, None),
        ("wait", 1500),
        ("read_gain",),
        ("read_exposure",),
        ("gain", False, 24),
        ("exposure", False, 30000),
        ("wait", 200),
    ]


def test_field_debug_draws_pipe_and_ball_boxes():
    helpers = load_main_helpers()

    class FakeImage:
        def __init__(self):
            self.rectangles = []

        def draw_rectangle(self, rect, **kwargs):
            self.rectangles.append(rect)

        def draw_line(self, *args, **kwargs):
            pass

        def draw_circle(self, *args, **kwargs):
            pass

        def draw_cross(self, *args, **kwargs):
            pass

        def draw_string(self, *args, **kwargs):
            pass

    class FakeBall:
        def rect(self):
            return (120, 52, 8, 7)

        def w(self):
            return 8

        def h(self):
            return 7

        def cx(self):
            return 124

        def cy(self):
            return 55

    image = FakeImage()
    pipe = (0, 40, 160, 32)
    search = (8, 48, 144, 16)
    helpers["draw_debug"](image, pipe, search, FakeBall(), 42.5)

    assert (4, 50, 152, 20) in image.rectangles
    assert pipe not in image.rectangles
    assert (120, 52, 8, 7) in image.rectangles
    assert search not in image.rectangles


def test_pipe_display_box_trims_detection_margin():
    helpers = load_main_helpers()

    assert helpers["pipe_display_roi"]((0, 54, 160, 26)) == (4, 62, 152, 16)


def test_live_ball_candidate_fits_asymmetric_search_roi():
    helpers = load_main_helpers()
    search = helpers["make_search_roi"]((0, 54, 160, 29), -1, 8)

    class LiveBall:
        def w(self):
            return 4

        def h(self):
            return 7

        def pixels(self):
            return 21

        def density(self):
            return 0.75

        def x(self):
            return 81

        def y(self):
            return 68

        def cx(self):
            return 83

    assert search == (4, 61, 152, 19)
    assert helpers["score_blob"](LiveBall(), search, -1) > 0


def test_terminal_status_contains_coordinates_distance_and_threshold():
    helpers = load_main_helpers()
    pipe = (0, 40, 160, 32)

    found = helpers["format_status"](42.5, 124, 55, 33, pipe, 87)
    lost = helpers["format_status"](41.0, None, None, None, pipe, 90)

    assert found == "FPS=42.5 BALL=(124,55) MM=33 PIPE=(0, 40, 160, 32) TH=87"
    assert lost == "FPS=41.0 BALL=LOST PIPE=(0, 40, 160, 32) TH=90"


def test_tile_rois_cover_bright_pipe_with_overlap():
    helpers = load_main_helpers()

    assert helpers["make_tile_rois"]((4, 60, 152, 17)) == [
        (4, 60, 44, 17),
        (36, 60, 50, 17),
        (74, 60, 50, 17),
        (112, 60, 44, 17),
    ]


def test_find_ball_uses_unmerged_local_fallback_after_fast_miss():
    helpers = load_main_helpers()

    class LiveBall:
        def w(self):
            return 4

        def h(self):
            return 7

        def pixels(self):
            return 21

        def density(self):
            return 0.75

        def y(self):
            return 68

        def cx(self):
            return 20

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
