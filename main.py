"""
K230 vision-only aiming module for the school contest.

Responsibilities:
- Detect the A4 black target frame and estimate its center.
- Detect the laser spot.
- Send visual status and gimbal delta commands to MSPM0 over UART.
- Support offline threshold tuning by UART commands and touch regions.

MSPM0 owns car motion, laser power output, and gimbal actuation.
"""

try:
    import time
    import os
    try:
        import ujson as json
    except Exception:
        import json
except Exception:
    time = None
    os = None
    json = None

try:
    from media.sensor import Sensor
    from media.display import Display
    from media.media import MediaManager
    import image
    from machine import TOUCH
    from ybUtils.YbUart import YbUart
except Exception:
    Sensor = None
    Display = None
    MediaManager = None
    image = None
    TOUCH = None
    YbUart = None


DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
DISPLAY_X_OFFSET = (DISPLAY_WIDTH - IMAGE_WIDTH) // 2
CAMERA_CHN = globals().get("CAM_CHN_ID_0", 0)
TUNE_PANEL_WIDTH = 120
TUNE_PREVIEW_WIDTH = DISPLAY_WIDTH - TUNE_PANEL_WIDTH * 2
TUNE_PREVIEW_HEIGHT = 480
TUNE_PREVIEW_ROI = ((IMAGE_WIDTH - TUNE_PREVIEW_WIDTH) // 2, 0, TUNE_PREVIEW_WIDTH, TUNE_PREVIEW_HEIGHT)

DEFAULT_BLACK_THRESHOLD = [(0, 60)]
DEFAULT_LASER_THRESHOLD = [(20, 90, 10, 127, -30, 90)]

A4_RATIO = 297.0 / 210.0
MAX_RATIO_ERROR = 0.65
MIN_FRAME_AREA = 2200
MAX_FRAME_AREA_RATIO = 0.88
RECT_THRESHOLDS = (2500, 5000, 8000, 12000)

MIN_LASER_AREA = 3
MAX_LASER_AREA = 1800
LASER_SEARCH_MARGIN = 36

SMOOTH_NUM = 2
SMOOTH_DEN = 5
LOCK_DEADBAND = 4
GIMBAL_PIXELS_PER_STEP = 7
GIMBAL_MAX_STEP_DELTA = 35
PAN_REVERSE = False
TILT_REVERSE = False

UART_BAUDRATE = 115200
DIAG_INTERVAL_MS = 160
HEART_INTERVAL_MS = 1000
DISPLAY_EVERY_N = 2
THRESHOLD_CONFIG_PATH = "/sdcard/k230_aim_thresholds.json"


def ticks_ms():
    if time and hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    if time:
        return int(time.time() * 1000)
    return 0


def ticks_diff(now, before):
    if time and hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, before)
    return now - before


def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def rect_tuple(feature):
    try:
        rect = feature.rect()
    except Exception:
        rect = feature
    return int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])


def rect_area(rect):
    return int(rect[2]) * int(rect[3])


def rect_center(rect):
    x, y, w, h = rect
    return int(x + w // 2), int(y + h // 2)


def inflate_rect(rect, margin, max_w=IMAGE_WIDTH, max_h=IMAGE_HEIGHT):
    x, y, w, h = rect
    x0 = clamp(int(x - margin), 0, max_w - 1)
    y0 = clamp(int(y - margin), 0, max_h - 1)
    x1 = clamp(int(x + w + margin), x0 + 1, max_w)
    y1 = clamp(int(y + h + margin), y0 + 1, max_h)
    return x0, y0, x1 - x0, y1 - y0


def distance2(point_a, point_b):
    dx = int(point_a[0]) - int(point_b[0])
    dy = int(point_a[1]) - int(point_b[1])
    return dx * dx + dy * dy


def feature_corners(feature):
    try:
        corners = feature.corners()
    except Exception:
        corners = None
    if corners:
        points = []
        for point in corners:
            points.append((int(point[0]), int(point[1])))
        if len(points) >= 4:
            return tuple(points[:4])

    x, y, w, h = rect_tuple(feature)
    return ((x, y), (x + w, y), (x + w, y + h), (x, y + h))


def sort_quad_corners(corners):
    points = [(int(p[0]), int(p[1])) for p in corners[:4]]
    if len(points) < 4:
        return None
    tl = min(points, key=lambda p: p[0] + p[1])
    br = max(points, key=lambda p: p[0] + p[1])
    tr = min(points, key=lambda p: p[0] - p[1])
    bl = max(points, key=lambda p: p[0] - p[1])
    ordered = (tl, tr, br, bl)
    if len(set(ordered)) != 4:
        return None
    return ordered


def line_intersection(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1:
        return ((x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4)
    a = x1 * y2 - y1 * x2
    b = x3 * y4 - y3 * x4
    px = (a * (x3 - x4) - (x1 - x2) * b) / den
    py = (a * (y3 - y4) - (y1 - y2) * b) / den
    return int(px), int(py)


def quad_center(corners):
    ordered = sort_quad_corners(corners)
    if not ordered:
        return None
    tl, tr, br, bl = ordered
    return line_intersection(tl, br, tr, bl)


def rect_ratio_error(rect):
    _x, _y, w, h = rect
    if w <= 0 or h <= 0:
        return 999.0
    ratio = max(float(w) / float(h), float(h) / float(w))
    return abs(ratio - A4_RATIO)


def make_frame_candidate(feature):
    rect = rect_tuple(feature)
    area = rect_area(rect)
    if area < MIN_FRAME_AREA:
        return None
    if area > IMAGE_WIDTH * IMAGE_HEIGHT * MAX_FRAME_AREA_RATIO:
        return None
    ratio_error = rect_ratio_error(rect)
    if ratio_error > MAX_RATIO_ERROR:
        return None

    corners = feature_corners(feature)
    center = quad_center(corners) or rect_center(rect)
    return {
        "rect": rect,
        "area": area,
        "center": center,
        "corners": corners,
        "ratio_error": ratio_error,
    }


def candidate_score(candidate, previous_center=None):
    score = min(candidate["area"] // 80, 4000)
    score -= int(candidate["ratio_error"] * 1200)
    if previous_center:
        score -= distance2(candidate["center"], previous_center) // 10
    else:
        score -= distance2(candidate["center"], (IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2)) // 22
    return score


def select_best_candidate(candidates, previous_center=None):
    best = None
    best_score = None
    for candidate in candidates:
        if not candidate:
            continue
        score = candidate_score(candidate, previous_center)
        if best_score is None or score > best_score:
            best = candidate
            best_score = score
    return best


def smooth_point(previous, current):
    if not previous:
        return current
    return (
        int((previous[0] * (SMOOTH_DEN - SMOOTH_NUM) + current[0] * SMOOTH_NUM) // SMOOTH_DEN),
        int((previous[1] * (SMOOTH_DEN - SMOOTH_NUM) + current[1] * SMOOTH_NUM) // SMOOTH_DEN),
    )


def blob_rect(blob):
    return int(blob.x()), int(blob.y()), int(blob.w()), int(blob.h())


def blob_center(blob):
    return int(blob.x() + blob.w() // 2), int(blob.y() + blob.h() // 2)


def blob_area(blob):
    try:
        return int(blob.pixels())
    except Exception:
        return int(blob.w() * blob.h())


def select_laser_blob(blobs, target_center=None, previous_laser=None):
    best = None
    best_score = None
    for blob in blobs:
        area = blob_area(blob)
        if area < MIN_LASER_AREA or area > MAX_LASER_AREA:
            continue
        center = blob_center(blob)
        score = 500 - abs(area - 40)
        if previous_laser:
            score -= distance2(center, previous_laser) // 8
        elif target_center:
            score -= distance2(center, target_center) // 20
        if best_score is None or score > best_score:
            best = blob
            best_score = score
    return best


def format_gimbal_delta(err_x, err_y):
    pan = int(round(float(err_x) / GIMBAL_PIXELS_PER_STEP))
    tilt = int(round(float(err_y) / GIMBAL_PIXELS_PER_STEP))
    if PAN_REVERSE:
        pan = -pan
    if TILT_REVERSE:
        tilt = -tilt
    pan = clamp(pan, -GIMBAL_MAX_STEP_DELTA, GIMBAL_MAX_STEP_DELTA)
    tilt = clamp(tilt, -GIMBAL_MAX_STEP_DELTA, GIMBAL_MAX_STEP_DELTA)
    return "GIMBALD,%d,%d" % (pan, tilt)


def format_vstat(frame_ok, laser_ok, err_x, err_y):
    if not frame_ok or not laser_ok:
        return "VSTAT,LOST,0,0"
    if abs(err_x) <= LOCK_DEADBAND and abs(err_y) <= LOCK_DEADBAND:
        return "VSTAT,LOCKED,%d,%d" % (int(err_x), int(err_y))
    return "VSTAT,TRACKING,%d,%d" % (int(err_x), int(err_y))


def format_vdiag(frame_ok, laser_ok, black_count, laser_count, frame, laser_blob):
    if frame:
        fx, fy, fw, fh = frame["rect"]
        fa = frame["area"]
    else:
        fx = fy = fw = fh = fa = 0
    if laser_blob:
        lx, ly = blob_center(laser_blob)
        la = blob_area(laser_blob)
    else:
        lx = ly = la = 0
    return "VDIAG,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d" % (
        1 if frame_ok else 0,
        1 if laser_ok else 0,
        int(black_count),
        int(laser_count),
        int(fa),
        int(la),
        int(fx),
        int(fy),
        int(fw),
        int(fh),
        int(lx),
        int(ly),
    )


class RuntimeState:
    def __init__(self):
        self.mode = "AIM"
        self.view = "AIM"
        self.black_threshold = list(DEFAULT_BLACK_THRESHOLD)
        self.laser_threshold = list(DEFAULT_LASER_THRESHOLD)
        self.ui_target = "B"
        self.ui_selected = 1
        self.ui_step = 2
        self.ui_enabled = True
        self.last_frame_center = None
        self.last_laser_center = None
        self.last_diag_ms = 0
        self.last_heart_ms = 0
        self.frame_index = 0
        self.tune_message = ""
        self.tune_message_until = 0


def threshold_values_for_ui(state):
    if state.ui_target == "B":
        return [state.black_threshold[0][0], state.black_threshold[0][1]]
    return list(state.laser_threshold[0])


def set_threshold_values_from_ui(state, values):
    if state.ui_target == "B":
        state.black_threshold = [(int(values[0]), int(values[1]))]
    else:
        state.laser_threshold = [tuple(int(v) for v in values[:6])]


def set_tune_message(state, message, duration_ms=900):
    state.tune_message = message
    state.tune_message_until = ticks_ms() + duration_ms


def enter_tune_mode(state, target=None):
    state.mode = "TUNE"
    state.view = "BLACK" if target != "L" else "LASER"
    state.ui_target = "B" if target != "L" else "L"
    state.ui_selected = clamp(state.ui_selected, 0, 1 if state.ui_target == "B" else 5)
    set_tune_message(state, "TUNE %s" % ("BLACK" if state.ui_target == "B" else "LASER"))


def load_thresholds(state, path=THRESHOLD_CONFIG_PATH):
    if not json:
        return False
    try:
        with open(path, "r") as f:
            data = json.load(f)
        black = data.get("black")
        laser = data.get("laser")
        if black and len(black) >= 2:
            state.black_threshold = [(int(black[0]), int(black[1]))]
        if laser and len(laser) >= 6:
            state.laser_threshold = [tuple(int(v) for v in laser[:6])]
        return True
    except Exception:
        return False


def save_thresholds(state, path=THRESHOLD_CONFIG_PATH):
    if not json:
        return False
    try:
        data = {
            "black": list(state.black_threshold[0]),
            "laser": list(state.laser_threshold[0]),
        }
        with open(path, "w") as f:
            json.dump(data, f)
        return True
    except Exception:
        return False


def adjust_selected_threshold(state, direction):
    values = threshold_values_for_ui(state)
    idx = clamp(int(state.ui_selected), 0, len(values) - 1)
    min_v = 0 if (state.ui_target == "B" or idx < 2) else -128
    max_v = 255 if state.ui_target == "B" else 127
    values[idx] = clamp(values[idx] + direction * state.ui_step, min_v, max_v)
    if idx % 2 == 0 and idx + 1 < len(values) and values[idx] > values[idx + 1]:
        values[idx] = values[idx + 1]
    if idx % 2 == 1 and values[idx] < values[idx - 1]:
        values[idx] = values[idx - 1]
    set_threshold_values_from_ui(state, values)


def handle_command(command, state):
    if not command:
        return False
    command = command.strip()
    if not command:
        return False
    parts = [part.strip() for part in command.split(",")]
    head = parts[0].upper()

    try:
        if head == "MODE" and len(parts) >= 2:
            mode = parts[1].upper()
            if mode in ("AIM", "TUNE", "STOP", "ECG"):
                if mode == "TUNE":
                    enter_tune_mode(state)
                else:
                    state.mode = mode
                return True
        if head in ("AIM", "STOP", "TUNE", "ECG"):
            if head == "TUNE":
                enter_tune_mode(state)
            else:
                state.mode = head
            return True
        if head == "BTH" and len(parts) in (3, 7):
            nums = [int(v) for v in parts[1:]]
            if len(nums) == 2:
                state.black_threshold = [(clamp(nums[0], 0, 255), clamp(nums[1], 0, 255))]
            else:
                state.black_threshold = [(clamp(nums[0], 0, 255), clamp(nums[1], 0, 255))]
            return True
        if head in ("LTH", "RTH") and len(parts) == 7:
            nums = [int(v) for v in parts[1:]]
            nums[0] = clamp(nums[0], 0, 127)
            nums[1] = clamp(nums[1], 0, 127)
            for i in range(2, 6):
                nums[i] = clamp(nums[i], -128, 127)
            state.laser_threshold = [tuple(nums)]
            return True
        if head == "UI" and len(parts) >= 2:
            arg = parts[1].upper()
            if arg == "ON":
                state.ui_enabled = True
                return True
            if arg == "OFF":
                state.ui_enabled = False
                return True
            if arg in ("B", "BLACK"):
                state.ui_target = "B"
                state.ui_selected = clamp(state.ui_selected, 0, 1)
                return True
            if arg in ("L", "R", "LASER", "RED"):
                state.ui_target = "L"
                return True
            if arg == "NEXT":
                limit = 2 if state.ui_target == "B" else 6
                state.ui_selected = (state.ui_selected + 1) % limit
                return True
            if arg == "PREV":
                limit = 2 if state.ui_target == "B" else 6
                state.ui_selected = (state.ui_selected - 1) % limit
                return True
            if arg == "+":
                adjust_selected_threshold(state, 1)
                return True
            if arg == "-":
                adjust_selected_threshold(state, -1)
                return True
            if arg == "STEP" and len(parts) >= 3:
                state.ui_step = clamp(int(parts[2]), 1, 20)
                return True
            if arg == "VIEW" and len(parts) >= 3:
                view = parts[2].upper()
                if view in ("AIM", "BLACK", "LASER", "RAW"):
                    state.view = view
                    return True
        if head in ("RAW", "BLACK", "LASER"):
            state.view = head
            return True
    except Exception:
        return False
    return False


def send_packet(uart, packet):
    line = packet if isinstance(packet, str) else packet.decode("utf-8", "ignore")
    line = line.strip()
    try:
        print(line)
    except Exception:
        pass
    if uart:
        try:
            uart.send((line + "\r\n").encode("utf-8"))
        except Exception:
            try:
                uart.send(line + "\r\n")
            except Exception as exc:
                print("UART_SEND_ERR,%s" % exc)


def read_uart_commands(uart, state):
    if not uart:
        return
    while True:
        try:
            if not uart.any():
                return
            raw = uart.readline()
        except Exception:
            return
        if not raw:
            return
        try:
            line = raw.decode("utf-8", "ignore").strip()
        except Exception:
            line = str(raw).strip()
        if handle_command(line, state):
            send_packet(uart, "ACK,%s" % line)
            send_thresholds(uart, state)
        else:
            send_packet(uart, "NACK,%s" % line)


def send_thresholds(uart, state):
    b = state.black_threshold[0]
    l = state.laser_threshold[0]
    send_packet(uart, "VTH,BTH,%d,%d" % (b[0], b[1]))
    send_packet(uart, "VTH,LTH,%d,%d,%d,%d,%d,%d" % l)


def draw_text(img, x, y, text, color=(255, 255, 255), size=18):
    try:
        img.draw_string_advanced(int(x), int(y), int(size), text, color=color)
    except Exception:
        try:
            img.draw_string(int(x), int(y), text, color=color)
        except Exception:
            pass


def draw_frame(img, candidate, color=(0, 255, 0)):
    if not candidate:
        return
    try:
        img.draw_rectangle(candidate["rect"], color=color, thickness=2)
    except Exception:
        pass
    try:
        corners = candidate["corners"]
        for i in range(4):
            p1 = corners[i]
            p2 = corners[(i + 1) % 4]
            img.draw_line(p1[0], p1[1], p2[0], p2[1], color=color, thickness=2)
    except Exception:
        pass


def draw_cross(img, point, color=(255, 0, 0)):
    if not point:
        return
    x, y = int(point[0]), int(point[1])
    try:
        img.draw_cross(x, y, color=color, size=16, thickness=2)
    except Exception:
        try:
            img.draw_line(x - 12, y, x + 12, y, color=color, thickness=2)
            img.draw_line(x, y - 12, x, y + 12, color=color, thickness=2)
        except Exception:
            pass


def draw_ui_overlay(img, state, fps, status):
    if not state.ui_enabled:
        return
    draw_text(img, 6, 4, "K230 %s %s fps:%.1f" % (state.mode, status, fps), color=(255, 255, 0), size=18)
    values = threshold_values_for_ui(state)
    names = ("Lmin", "Lmax") if state.ui_target == "B" else ("Lmin", "Lmax", "Amin", "Amax", "Bmin", "Bmax")
    idx = clamp(state.ui_selected, 0, len(values) - 1)
    draw_text(img, 6, 28, "UI %s step:%d >%s=%d" % (state.ui_target, state.ui_step, names[idx], values[idx]), color=(255, 255, 255), size=16)
    draw_text(img, 6, 50, "B:%s L:%s" % (state.black_threshold[0], state.laser_threshold[0]), color=(180, 255, 180), size=14)

    y = DISPLAY_HEIGHT - 34
    labels = ("AIM", "BLACK", "LASER", "RAW")
    for i, label in enumerate(labels):
        x = i * (IMAGE_WIDTH // 4)
        try:
            img.draw_rectangle(x, y, IMAGE_WIDTH // 4, 34, color=(60, 60, 60), thickness=1, fill=True)
        except Exception:
            pass
        draw_text(img, x + 18, y + 7, label, color=(255, 255, 255), size=16)


def tune_button_name(x, y):
    if x < TUNE_PANEL_WIDTH:
        if y < 40:
            return "return"
        if y > DISPLAY_HEIGHT - 40:
            return "reset"
        if y >= 60 and (y - 60) % 60 < 40:
            idx = (y - 60) // 60
            if 0 <= idx < 6:
                return "dec%d" % idx
    elif x > DISPLAY_WIDTH - TUNE_PANEL_WIDTH:
        if y < 40:
            return "change"
        if y > DISPLAY_HEIGHT - 40:
            return "save"
        if y >= 60 and (y - 60) % 60 < 40:
            idx = (y - 60) // 60
            if 0 <= idx < 6:
                return "inc%d" % idx
    return None


def handle_tune_button(button, state):
    if not button:
        return False
    if button == "return":
        state.mode = "AIM"
        state.view = "AIM"
        set_tune_message(state, "RETURN AIM")
        return True
    if button == "change":
        state.ui_target = "L" if state.ui_target == "B" else "B"
        state.view = "LASER" if state.ui_target == "L" else "BLACK"
        state.ui_selected = 0
        set_tune_message(state, "TUNE %s" % ("LASER" if state.ui_target == "L" else "BLACK"))
        return True
    if button == "reset":
        if state.ui_target == "B":
            state.black_threshold = [(0, 255)]
        else:
            state.laser_threshold = [(0, 127, -128, 127, -128, 127)]
        state.ui_selected = 0
        set_tune_message(state, "RESET")
        return True
    if button == "save":
        set_tune_message(state, "SAVED" if save_thresholds(state) else "SAVE FAIL")
        return True
    if button.startswith("dec") or button.startswith("inc"):
        idx = int(button[3:])
        limit = 2 if state.ui_target == "B" else 6
        if idx >= limit:
            return False
        state.ui_selected = idx
        adjust_selected_threshold(state, -1 if button.startswith("dec") else 1)
        set_tune_message(state, "%s %s" % ("BTH" if state.ui_target == "B" else "LTH", threshold_values_for_ui(state)[idx]), 400)
        return True
    return False


def handle_touch(tp, state, last_touch_ms):
    if not tp:
        return last_touch_ms
    now = ticks_ms()
    if ticks_diff(now, last_touch_ms) < 280:
        return last_touch_ms
    try:
        points = tp.read(1)
    except TypeError:
        points = tp.read()
    except Exception:
        return last_touch_ms
    if not points:
        return last_touch_ms

    point = points[0]
    x = int(point.x)
    y = int(point.y)
    if state.mode == "TUNE":
        if handle_tune_button(tune_button_name(x, y), state):
            return now
        return last_touch_ms
    if y >= DISPLAY_HEIGHT - 44:
        local_x = x - DISPLAY_X_OFFSET
        if local_x < 0 or local_x >= IMAGE_WIDTH:
            return last_touch_ms
        zone = int(local_x * 4 // IMAGE_WIDTH)
        state.view = ("AIM", "BLACK", "LASER", "RAW")[clamp(zone, 0, 3)]
        if state.view in ("BLACK", "LASER"):
            enter_tune_mode(state, "B" if state.view == "BLACK" else "L")
        return now
    if y < 80 and x < 170:
        adjust_selected_threshold(state, -1)
        return now
    if y < 80 and x > DISPLAY_WIDTH - 170:
        adjust_selected_threshold(state, 1)
        return now
    if 230 < x < 410 and y < 90:
        state.ui_target = "L" if state.ui_target == "B" else "B"
        state.ui_selected = 0
        return now
    if x < 130 and 100 < y < DISPLAY_HEIGHT - 70:
        handle_command("UI,PREV", state)
        return now
    if x > DISPLAY_WIDTH - 130 and 100 < y < DISPLAY_HEIGHT - 70:
        handle_command("UI,NEXT", state)
        return now
    return now


def init_sensor():
    try:
        sensor = Sensor(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    except TypeError:
        sensor = Sensor()
    sensor.reset()
    try:
        sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT, chn=CAMERA_CHN)
    except TypeError:
        sensor.set_framesize(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    try:
        sensor.set_pixformat(Sensor.RGB565, chn=CAMERA_CHN)
    except TypeError:
        sensor.set_pixformat(Sensor.RGB565)
    try:
        sensor.skip_frames(time=300)
    except Exception:
        pass
    return sensor


def find_frame(img, state):
    candidates = []
    raw_count = 0
    try:
        gray = img.to_grayscale(copy=True)
        binary = gray.binary(state.black_threshold)
    except Exception:
        binary = img
    for threshold in RECT_THRESHOLDS:
        try:
            rects = binary.find_rects(threshold=threshold) or []
        except TypeError:
            rects = binary.find_rects(threshold) or []
        except Exception:
            rects = []
        raw_count = len(rects)
        candidates = []
        for feature in rects:
            candidate = make_frame_candidate(feature)
            if candidate:
                candidates.append(candidate)
        selected = select_best_candidate(candidates, state.last_frame_center)
        if selected:
            state.last_frame_center = smooth_point(state.last_frame_center, selected["center"])
            selected["center"] = state.last_frame_center
            return selected, raw_count, binary
    return None, raw_count, binary


def find_laser(img, state, frame_candidate):
    roi = None
    if frame_candidate:
        roi = inflate_rect(frame_candidate["rect"], LASER_SEARCH_MARGIN)
    try:
        if roi:
            blobs = img.find_blobs(state.laser_threshold, False, roi, x_stride=2, y_stride=2, pixels_threshold=MIN_LASER_AREA, margin=False) or []
        else:
            blobs = img.find_blobs(state.laser_threshold, False, x_stride=3, y_stride=3, pixels_threshold=MIN_LASER_AREA, margin=False) or []
    except Exception:
        blobs = []
    target_center = frame_candidate["center"] if frame_candidate else None
    selected = select_laser_blob(blobs, target_center, state.last_laser_center)
    if selected:
        state.last_laser_center = smooth_point(state.last_laser_center, blob_center(selected))
    return selected, len(blobs)


def make_view_image(img, black_binary, state):
    if state.view == "BLACK" and black_binary is not None:
        try:
            return black_binary.to_rgb565()
        except Exception:
            return black_binary
    if state.view == "LASER":
        try:
            return img.binary(state.laser_threshold).to_rgb565()
        except Exception:
            return img
    return img


def tune_preview_image(raw_img, state):
    try:
        img_ = raw_img.copy(roi=TUNE_PREVIEW_ROI)
    except Exception:
        img_ = raw_img
    if state.ui_target == "B":
        try:
            img_ = img_.to_grayscale()
            img_ = img_.binary(state.black_threshold)
            return img_.to_rgb565()
        except Exception:
            return img_
    try:
        return img_.binary(state.laser_threshold).to_rgb565()
    except Exception:
        return img_


def draw_tune_screen(raw_img, state):
    canvas = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.RGB565)
    try:
        canvas.draw_rectangle(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, color=(255, 255, 255), thickness=1, fill=True)
    except Exception:
        pass

    preview = tune_preview_image(raw_img, state)
    try:
        canvas.draw_image(preview, TUNE_PANEL_WIDTH, 0)
    except Exception:
        pass

    button_color = (150, 150, 150)
    active_color = (90, 180, 255)
    text_color = (0, 0, 0)
    labels = ("Lmin", "Lmax") if state.ui_target == "B" else ("Lmin", "Lmax", "Amin", "Amax", "Bmin", "Bmax")
    values = threshold_values_for_ui(state)

    def button(x, y, w, h, text, active=False):
        color = active_color if active else button_color
        try:
            canvas.draw_rectangle(x, y, w, h, color=color, thickness=2, fill=True)
        except Exception:
            pass
        draw_text(canvas, x + 12, y + 8, text, color=text_color, size=18)

    button(0, 0, TUNE_PANEL_WIDTH, 40, "返回")
    button(DISPLAY_WIDTH - TUNE_PANEL_WIDTH, 0, TUNE_PANEL_WIDTH, 40, "切换")
    button(0, DISPLAY_HEIGHT - 40, TUNE_PANEL_WIDTH, 40, "归位")
    button(DISPLAY_WIDTH - TUNE_PANEL_WIDTH, DISPLAY_HEIGHT - 40, TUNE_PANEL_WIDTH, 40, "保存")

    for i in range(6):
        y = 60 + i * 60
        active = i == state.ui_selected
        if i < len(values):
            text = "%s-" % labels[i]
            text_r = "%s+" % labels[i]
        else:
            text = "--"
            text_r = "--"
        button(0, y, TUNE_PANEL_WIDTH, 40, text, active)
        button(DISPLAY_WIDTH - TUNE_PANEL_WIDTH, y, TUNE_PANEL_WIDTH, 40, text_r, active)

    mode_name = "BLACK" if state.ui_target == "B" else "LASER"
    draw_text(canvas, TUNE_PANEL_WIDTH + 8, 6, "TUNE %s step:%d" % (mode_name, state.ui_step), color=(255, 255, 0), size=18)
    draw_text(canvas, TUNE_PANEL_WIDTH + 8, 30, "B:%s" % (state.black_threshold[0],), color=(0, 255, 0), size=14)
    draw_text(canvas, TUNE_PANEL_WIDTH + 8, 50, "L:%s" % (state.laser_threshold[0],), color=(0, 255, 255), size=14)
    if state.tune_message and ticks_diff(state.tune_message_until, ticks_ms()) > 0:
        try:
            canvas.draw_rectangle(250, 210, 300, 44, color=(150, 150, 150), thickness=2, fill=True)
        except Exception:
            pass
        draw_text(canvas, 270, 222, state.tune_message, color=(0, 0, 0), size=20)
    return canvas


def main():
    if Sensor is None:
        raise RuntimeError("This file must run on CanMV K230 firmware.")

    sensor = None
    uart = None
    tp = None
    state = RuntimeState()
    load_thresholds(state)
    last_touch_ms = 0

    try:
        uart = YbUart(baudrate=UART_BAUDRATE) if YbUart else None
        tp = TOUCH(0) if TOUCH else None
    except Exception as exc:
        print("INIT_WARN,%s" % exc)

    try:
        sensor = init_sensor()
        Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)
        MediaManager.init()
        sensor.run()
        clock = time.clock()
        send_packet(uart, "K230AIM,START,VISION_UART")
        send_thresholds(uart, state)

        while True:
            if os:
                try:
                    os.exitpoint()
                except Exception:
                    pass
            clock.tick()
            state.frame_index += 1
            read_uart_commands(uart, state)
            last_touch_ms = handle_touch(tp, state, last_touch_ms)

            try:
                img = sensor.snapshot(chn=CAMERA_CHN)
            except TypeError:
                img = sensor.snapshot()

            if state.mode == "TUNE":
                Display.show_image(draw_tune_screen(img, state))
                now = ticks_ms()
                if ticks_diff(now, state.last_heart_ms) >= HEART_INTERVAL_MS:
                    send_packet(uart, "KHEART,%d,%s,%s" % (now, state.mode, state.view))
                    send_thresholds(uart, state)
                    state.last_heart_ms = now
                continue

            frame_candidate, black_count, black_binary = find_frame(img, state)
            laser_blob, laser_count = find_laser(img, state, frame_candidate) if state.mode != "STOP" else (None, 0)

            frame_ok = frame_candidate is not None
            laser_ok = laser_blob is not None
            if frame_ok and laser_ok:
                target = frame_candidate["center"]
                laser = state.last_laser_center or blob_center(laser_blob)
                err_x = int(laser[0] - target[0])
                err_y = int(laser[1] - target[1])
            else:
                err_x = 0
                err_y = 0

            vstat = format_vstat(frame_ok, laser_ok, err_x, err_y)
            status = vstat.split(",")[1]

            now = ticks_ms()
            if state.mode == "AIM" and frame_ok and laser_ok:
                send_packet(uart, format_gimbal_delta(err_x, err_y))
            if ticks_diff(now, state.last_diag_ms) >= DIAG_INTERVAL_MS:
                send_packet(uart, vstat)
                send_packet(uart, format_vdiag(frame_ok, laser_ok, black_count, laser_count, frame_candidate, laser_blob))
                state.last_diag_ms = now
            if ticks_diff(now, state.last_heart_ms) >= HEART_INTERVAL_MS:
                send_packet(uart, "KHEART,%d,%s,%s" % (now, state.mode, state.view))
                state.last_heart_ms = now

            if state.frame_index % DISPLAY_EVERY_N == 0:
                view_img = make_view_image(img, black_binary, state)
                if frame_candidate:
                    draw_frame(view_img, frame_candidate)
                    draw_cross(view_img, frame_candidate["center"], color=(255, 0, 0))
                if laser_blob:
                    try:
                        view_img.draw_rectangle(blob_rect(laser_blob), color=(255, 128, 0), thickness=2)
                    except Exception:
                        pass
                    draw_cross(view_img, state.last_laser_center or blob_center(laser_blob), color=(0, 255, 255))
                draw_ui_overlay(view_img, state, clock.fps(), status)
                Display.show_image(view_img, x=DISPLAY_X_OFFSET, y=0)

    except KeyboardInterrupt:
        print("K230AIM,STOP_BY_USER")
    except BaseException as exc:
        print("K230AIM,EXCEPTION,%s" % exc)
    finally:
        if sensor is not None:
            try:
                sensor.stop()
            except Exception:
                pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        try:
            if uart:
                uart.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
