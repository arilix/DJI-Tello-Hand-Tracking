#!/usr/bin/env python3
"""
hand_control_node — Kontrol DJI Tello via gesture tangan (MediaPipe + PnP).

Gesture:
  ✌️  (peace/V sign)   → TAKEOFF  — tahan 1 detik
  🤟  (ILY/love sign)  → LAND     — tahan 1 detik

Tracking:
  Drone mengikuti tangan agar selalu berada di dalam kotak tengah frame.
  PD controller: error X/Y → lr/ud | error Z (jarak PnP) → fb.

Topics subscribe:
  /image_raw                 sensor_msgs/Image    (kamera Tello)

Topics publish:
  /takeoff                   std_msgs/Empty
  /land                      std_msgs/Empty
  /control                   geometry_msgs/Twist
  /hand_vision/debug_image   sensor_msgs/Image
"""

import sys
import time
import threading
import json
from collections import deque
from enum import Enum, auto

import cv2
import numpy as np

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
except ImportError:
    print("ERROR: mediapipe tidak ditemukan. Install: pip install mediapipe 'numpy<2'")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from cv_bridge import CvBridge

# ─── Landmark indices ──────────────────────────────────────────────────────────
LM_WRIST      = 0
LM_THUMB_CMC  = 1;  LM_THUMB_MCP  = 2;  LM_THUMB_IP   = 3;  LM_THUMB_TIP  = 4
LM_INDEX_MCP  = 5;  LM_INDEX_PIP  = 6;  LM_INDEX_DIP  = 7;  LM_INDEX_TIP  = 8
LM_MIDDLE_MCP = 9;  LM_MIDDLE_PIP = 10; LM_MIDDLE_DIP = 11; LM_MIDDLE_TIP = 12
LM_RING_MCP   = 13; LM_RING_PIP   = 14; LM_RING_DIP   = 15; LM_RING_TIP   = 16
LM_PINKY_MCP  = 17; LM_PINKY_PIP  = 18; LM_PINKY_DIP  = 19; LM_PINKY_TIP  = 20

CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),
    (0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20),
    (5,9),(9,13),(13,17),
]

# Warna
C_GREEN  = (0, 220, 0)
C_YELLOW = (0, 220, 220)
C_RED    = (40, 40, 220)
C_WHITE  = (255, 255, 255)
C_GRAY   = (110, 110, 110)
C_BLACK  = (0, 0, 0)
C_CYAN   = (220, 200, 0)
C_ORANGE = (0, 165, 255)
C_BLUE   = (220, 100, 0)


# ─── Drone state ──────────────────────────────────────────────────────────────
class DroneState(Enum):
    IDLE            = auto()   # di tanah, tunggu ✌️
    CONFIRM_TAKEOFF = auto()   # tahan ✌️ sebelum takeoff
    TAKING_OFF      = auto()   # takeoff command dikirim, tunggu
    TRACKING        = auto()   # terbang + ikuti tangan, tunggu 🤟
    CONFIRM_LAND    = auto()   # tahan 🤟 sebelum landing
    LANDING         = auto()   # land command dikirim, tunggu
    EMERGENCY       = auto()   # emergency stop


# ─── Pose Estimator (solvePnP + EMA) ─────────────────────────────────────────
class PoseEstimator:
    _CORNER = [LM_INDEX_MCP, LM_MIDDLE_MCP, LM_RING_MCP, LM_PINKY_MCP]

    def __init__(self, K: np.ndarray, D: np.ndarray, alpha: float = 0.35):
        self._K     = K.astype(np.float64)
        self._D     = D.astype(np.float64)
        self._alpha = alpha
        self._prev_r: np.ndarray | None = None
        self._prev_t: np.ndarray | None = None

    def estimate(self, lm_px: list, lm_world: list) -> dict:
        if len(lm_px) < 21 or len(lm_world) < 21:
            return {"valid": False}
        obj = np.array([lm_world[i] for i in self._CORNER], np.float32)
        img = np.array([lm_px[i]    for i in self._CORNER], np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj, img, self._K, self._D,
                                       flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return {"valid": False}
        rvec = rvec.flatten()
        tvec = tvec.flatten()
        # EMA smoothing
        if self._prev_r is not None:
            rvec = self._alpha * rvec + (1 - self._alpha) * self._prev_r
            tvec = self._alpha * tvec + (1 - self._alpha) * self._prev_t
        self._prev_r = rvec.copy()
        self._prev_t = tvec.copy()
        dist = float(np.linalg.norm(tvec))
        return {
            "valid": True,
            "rvec":  rvec,
            "tvec":  tvec,
            "tx": float(tvec[0]),
            "ty": float(tvec[1]),
            "tz": float(tvec[2]),
            "distance": dist,
        }

    def reset(self):
        self._prev_r = None
        self._prev_t = None


# ─── PD Controller ────────────────────────────────────────────────────────────
class PDController:
    def __init__(self, kp: float, kd: float, max_out: float, deadband: float = 0.0):
        self._kp       = kp
        self._kd       = kd
        self._max      = max_out
        self._deadband = deadband
        self._prev_err = 0.0

    def compute(self, error: float, dt: float) -> float:
        if abs(error) < self._deadband:
            self._prev_err = 0.0
            return 0.0
        deriv = (error - self._prev_err) / max(dt, 1e-3)
        out   = self._kp * error + self._kd * deriv
        self._prev_err = error
        return float(np.clip(out, -self._max, self._max))

    def reset(self):
        self._prev_err = 0.0


# ─── Gesture classifier ───────────────────────────────────────────────────────
def _finger_up(lm_px: list, tip: int, pip: int) -> bool:
    return lm_px[tip][1] < lm_px[pip][1]

def classify_gesture(lm_px: list, handedness: str) -> str:
    """Klasifikasi: 'PEACE' | 'ILY' | 'NONE'"""
    if len(lm_px) < 21:
        return "NONE"
    index  = _finger_up(lm_px, LM_INDEX_TIP,  LM_INDEX_PIP)
    middle = _finger_up(lm_px, LM_MIDDLE_TIP, LM_MIDDLE_PIP)
    ring   = _finger_up(lm_px, LM_RING_TIP,   LM_RING_PIP)
    pinky  = _finger_up(lm_px, LM_PINKY_TIP,  LM_PINKY_PIP)
    right  = (handedness == "Right")
    thumb  = ((right     and lm_px[LM_THUMB_TIP][0] < lm_px[LM_THUMB_IP][0]) or
              (not right and lm_px[LM_THUMB_TIP][0] > lm_px[LM_THUMB_IP][0]))

    # ✌️ Peace: index + middle naik, ring + pinky turun
    if index and middle and not ring and not pinky:
        return "PEACE"

    # 🤟 ILY: thumb + index + pinky naik, middle + ring turun
    if thumb and index and not middle and not ring and pinky:
        return "ILY"

    return "NONE"


# ─── UI helpers ───────────────────────────────────────────────────────────────
def _label(frame: np.ndarray, text: str, pt: tuple, color: tuple, scale: float = 0.48):
    th = 1
    (tw, th2), bl = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, th)
    x, y = int(pt[0]), int(pt[1])
    cv2.rectangle(frame, (x-2, y-th2-2), (x+tw+4, y+bl+2), C_BLACK, -1)
    cv2.putText(frame, text, (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, th, cv2.LINE_AA)


def draw_hand(frame: np.ndarray, lm_px: list):
    tips = {LM_THUMB_TIP, LM_INDEX_TIP, LM_MIDDLE_TIP, LM_RING_TIP, LM_PINKY_TIP}
    for a, b in CONNECTIONS:
        cv2.line(frame,
                 (int(lm_px[a][0]), int(lm_px[a][1])),
                 (int(lm_px[b][0]), int(lm_px[b][1])),
                 C_GRAY, 2, cv2.LINE_AA)
    for i, (px, py) in enumerate(lm_px):
        col = C_YELLOW if i == LM_WRIST else (C_GREEN if i in tips else C_WHITE)
        r   = 7 if i == LM_WRIST else 4
        cv2.circle(frame, (int(px), int(py)), r, col, -1, cv2.LINE_AA)


def draw_tracking_box(frame: np.ndarray, box_px: int,
                      hand_center: tuple | None, locked: bool):
    H, W = frame.shape[:2]
    cx, cy = W // 2, H // 2
    col = C_GREEN if locked else C_RED
    arm = 24

    # Corner brackets
    for dx, dy in [(-box_px, -box_px), (box_px, -box_px),
                   (box_px, box_px),   (-box_px, box_px)]:
        c = (cx + dx, cy + dy)
        cv2.line(frame, c, (c[0] + (arm if dx < 0 else -arm), c[1]), col, 2, cv2.LINE_AA)
        cv2.line(frame, c, (c[0], c[1] + (arm if dy < 0 else -arm)), col, 2, cv2.LINE_AA)

    # Center dot
    cv2.circle(frame, (cx, cy), 4, col, -1, cv2.LINE_AA)

    # Error arrow (hand → center)
    if hand_center is not None:
        hx, hy = int(hand_center[0]), int(hand_center[1])
        cv2.circle(frame, (hx, hy), 8, C_YELLOW, 2, cv2.LINE_AA)
        if not locked:
            cv2.arrowedLine(frame, (hx, hy), (cx, cy), C_ORANGE, 2,
                            tipLength=0.3, line_type=cv2.LINE_AA)


def draw_hud(frame: np.ndarray, state: DroneState, gesture: str,
             confirm_pct: float, fps: float,
             rc: dict | None, pose: dict | None):
    H, W = frame.shape[:2]

    # State banner
    state_color = {
        DroneState.IDLE:            C_GRAY,
        DroneState.CONFIRM_TAKEOFF: C_YELLOW,
        DroneState.TAKING_OFF:      C_CYAN,
        DroneState.TRACKING:        C_GREEN,
        DroneState.CONFIRM_LAND:    C_ORANGE,
        DroneState.LANDING:         C_ORANGE,
        DroneState.EMERGENCY:       C_RED,
    }.get(state, C_WHITE)

    state_txt = {
        DroneState.IDLE:            "IDLE  — tunjukkan ✌ untuk takeoff",
        DroneState.CONFIRM_TAKEOFF: f"KONFIRMASI TAKEOFF  {int(confirm_pct*100)}%",
        DroneState.TAKING_OFF:      "TAKEOFF ...",
        DroneState.TRACKING:        "TRACKING  — tunjukkan 🤟 untuk landing",
        DroneState.CONFIRM_LAND:    f"KONFIRMASI LANDING  {int(confirm_pct*100)}%",
        DroneState.LANDING:         "LANDING ...",
        DroneState.EMERGENCY:       "EMERGENCY STOP",
    }.get(state, "")

    _label(frame, state_txt, (8, 28), state_color, 0.60)
    _label(frame, f"FPS:{fps:.1f}", (W - 75, 20), C_WHITE, 0.45)

    # Gesture
    g_col = {
        "PEACE": C_GREEN, "ILY": C_ORANGE, "NONE": C_GRAY
    }.get(gesture, C_WHITE)
    g_sym = {"PEACE": "✌", "ILY": "🤟"}.get(gesture, "-")
    _label(frame, f"Gesture: {gesture} {g_sym}", (8, H - 36), g_col, 0.50)

    # RC commands
    if rc:
        rc_txt = (f"LR:{rc['lr']:+.0f}  FB:{rc['fb']:+.0f}  "
                  f"UD:{rc['ud']:+.0f}  YAW:{rc['yaw']:+.0f}")
        _label(frame, rc_txt, (8, H - 14), C_CYAN, 0.42)

    # Pose info
    if pose and pose.get("valid"):
        _label(frame, f"Dist: {pose['distance']:.2f}m  "
               f"XYZ=({pose['tx']:.2f},{pose['ty']:.2f},{pose['tz']:.2f})",
               (8, 52), C_WHITE, 0.42)

    # Confirm progress bar
    if confirm_pct > 0:
        bar_w = int((W - 20) * confirm_pct)
        cv2.rectangle(frame, (10, H - 58), (10 + bar_w, H - 48),
                      C_YELLOW if state == DroneState.CONFIRM_TAKEOFF else C_ORANGE, -1)
        cv2.rectangle(frame, (10, H - 58), (W - 10, H - 48), C_WHITE, 1)


# ─── Main Node ────────────────────────────────────────────────────────────────
class HandControlNode(Node):

    def __init__(self):
        super().__init__("hand_control_node")
        self._declare_params()
        self._load_params()
        self._init_mediapipe()
        self._init_controllers()
        self._bridge   = CvBridge()
        self._lock     = threading.Lock()
        self._frame_n  = 0
        self._t_deque  = deque(maxlen=60)
        self._prev_t   = time.monotonic()

        # State machine
        self._state          = DroneState.IDLE
        self._confirm_frames = 0
        self._wait_frames    = 0
        self._last_gesture   = "NONE"
        self._gesture_stable = 0   # consecutive frames same gesture

        # Debug
        self._last_rc   = None
        self._last_pose = None

        self._init_publishers()
        self._init_subscribers()

        self.get_logger().info("═══════════════════════════════════════════════")
        self.get_logger().info("  Tello Hand Control — PnP + Gesture")
        self.get_logger().info(f"  model       : {self._model_path}")
        self.get_logger().info(f"  target_dist : {self._target_z:.2f}m")
        self.get_logger().info(f"  box_px      : {self._box_px}px")
        self.get_logger().info(f"  confirm_frm : {self._confirm_frames_need}")
        self.get_logger().info("  Gesture: ✌ = TAKEOFF  |  🤟 = LAND")
        self.get_logger().info("═══════════════════════════════════════════════")

    # ── Parameter ─────────────────────────────────────────────────────────────
    def _declare_params(self):
        self.declare_parameter("hand_model_path",    "")
        self.declare_parameter("camera_topic",       "/image_raw")
        self.declare_parameter("target_distance",    0.60)    # meter
        self.declare_parameter("box_px",             130)     # pixel half-size tracking box
        self.declare_parameter("kp_lateral",         0.50)    # P gain lateral (normalized)
        self.declare_parameter("kd_lateral",         0.08)    # D gain lateral
        self.declare_parameter("kp_depth",           55.0)    # P gain depth (m → %)
        self.declare_parameter("kd_depth",           8.0)     # D gain depth
        self.declare_parameter("max_speed_lateral",  35.0)    # % max lr/ud
        self.declare_parameter("max_speed_depth",    28.0)    # % max fb
        self.declare_parameter("deadband_px",        18.0)    # pixel deadband X/Y
        self.declare_parameter("deadband_z",         0.04)    # meter deadband Z
        self.declare_parameter("confirm_seconds",    1.0)     # detik tahan gesture
        self.declare_parameter("takeoff_wait_sec",   4.5)     # detik tunggu setelah takeoff
        self.declare_parameter("land_wait_sec",      3.0)     # detik tunggu setelah land
        self.declare_parameter("max_hands",          1)
        self.declare_parameter("min_detect_conf",    0.6)
        self.declare_parameter("min_track_conf",     0.5)
        self.declare_parameter("smoothing_alpha",    0.35)
        self.declare_parameter("show_window",        True)
        self.declare_parameter("camera_matrix",
            [924.0, 0.0, 480.0, 0.0, 924.0, 360.0, 0.0, 0.0, 1.0])
        self.declare_parameter("dist_coeffs",
            [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("capture_internal",   False)
        self.declare_parameter("capture_device_id",  0)
        self.declare_parameter("capture_width",      960)
        self.declare_parameter("capture_height",     720)
        self.declare_parameter("capture_fps",        30.0)

    def _load_params(self):
        g = self.get_parameter
        self._model_path         = g("hand_model_path").value
        self._camera_topic       = g("camera_topic").value
        self._target_z           = g("target_distance").value
        self._box_px             = g("box_px").value
        self._kp_lat             = g("kp_lateral").value
        self._kd_lat             = g("kd_lateral").value
        self._kp_dep             = g("kp_depth").value
        self._kd_dep             = g("kd_depth").value
        self._max_lat            = g("max_speed_lateral").value
        self._max_dep            = g("max_speed_depth").value
        self._db_px              = g("deadband_px").value
        self._db_z               = g("deadband_z").value
        self._confirm_sec        = g("confirm_seconds").value
        self._takeoff_wait_sec   = g("takeoff_wait_sec").value
        self._land_wait_sec      = g("land_wait_sec").value
        self._max_hands          = g("max_hands").value
        self._min_det            = g("min_detect_conf").value
        self._min_trk            = g("min_track_conf").value
        self._alpha              = g("smoothing_alpha").value
        self._show_win           = g("show_window").value
        cam  = g("camera_matrix").value
        dist = g("dist_coeffs").value
        self._K = (np.array(cam, np.float64).reshape(3, 3)
                   if len(cam) == 9
                   else np.diag([924., 924., 1.]))
        self._D = np.array(dist, np.float64)
        self._cap_internal = g("capture_internal").value
        self._cap_dev_id   = g("capture_device_id").value
        self._cap_w        = g("capture_width").value
        self._cap_h        = g("capture_height").value
        self._cap_fps      = g("capture_fps").value
        self._confirm_frames_need = int(self._confirm_sec * 25)  # @25fps

    # ── MediaPipe ──────────────────────────────────────────────────────────────
    def _init_mediapipe(self):
        if not self._model_path:
            self.get_logger().fatal(
                "hand_model_path belum diset!\n"
                "Download: wget -O hand_landmarker.task \\\n"
                "  https://storage.googleapis.com/mediapipe-models/"
                "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")
            rclpy.shutdown()
            sys.exit(1)
        base = mp_python.BaseOptions(model_asset_path=self._model_path)
        opts = mp_vision.HandLandmarkerOptions(
            base_options=base,
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=self._max_hands,
            min_hand_detection_confidence=self._min_det,
            min_hand_presence_confidence=self._min_det,
            min_tracking_confidence=self._min_trk,
        )
        self._detector = mp_vision.HandLandmarker.create_from_options(opts)
        self.get_logger().info(f"MediaPipe OK — max_hands={self._max_hands}")

    # ── Controllers ────────────────────────────────────────────────────────────
    def _init_controllers(self):
        self._pose_est = PoseEstimator(self._K, self._D, self._alpha)
        # X: kanan positif, Y: atas positif (image Y dibalik), Z: maju positif
        # Normalisasi: error_x_px / (W/2) → [-1, 1] → dikali max_lat
        # Untuk Z: error meter langsung dikali kp_dep
        self._ctrl_x = PDController(self._kp_lat * self._max_lat,
                                    self._kd_lat * self._max_lat,
                                    self._max_lat, 0.0)
        self._ctrl_y = PDController(self._kp_lat * self._max_lat,
                                    self._kd_lat * self._max_lat,
                                    self._max_lat, 0.0)
        self._ctrl_z = PDController(self._kp_dep, self._kd_dep,
                                    self._max_dep, 0.0)
        # EMA untuk output kontrol (meredam noise)
        self._rc_ema = {"lr": 0.0, "fb": 0.0, "ud": 0.0}
        self._rc_alpha = 0.45

    # ── Publishers / Subscribers ───────────────────────────────────────────────
    def _init_publishers(self):
        self._pub_takeoff = self.create_publisher(Empty,  "takeoff",  1)
        self._pub_land    = self.create_publisher(Empty,  "land",     1)
        self._pub_ctrl    = self.create_publisher(Twist,  "control",  1)
        self._pub_img     = self.create_publisher(Image,
                            "/hand_vision/debug_image", 5)
        self.get_logger().info("Publishers: /takeoff /land /control /hand_vision/debug_image")

    def _init_subscribers(self):
        if self._cap_internal:
            self._init_capture()
            return
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(
            Image, self._camera_topic, self._image_cb, qos)
        self.get_logger().info(f"Subscribe: '{self._camera_topic}'")

    def _init_capture(self):
        self._vcap = cv2.VideoCapture(self._cap_dev_id, cv2.CAP_V4L2)
        if not self._vcap.isOpened():
            self.get_logger().error("capture_internal gagal buka kamera")
            self._cap_internal = False
            self._init_subscribers()
            return
        self._vcap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._cap_w)
        self._vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._cap_h)
        self._vcap.set(cv2.CAP_PROP_FPS,          self._cap_fps)
        threading.Thread(target=self._cap_loop, daemon=True).start()
        self.get_logger().info(f"capture_internal: dev={self._cap_dev_id}")

    def _cap_loop(self):
        interval = 1.0 / max(self._cap_fps, 1)
        while rclpy.ok():
            t0 = time.monotonic()
            ret, frame = self._vcap.read()
            if ret and frame is not None:
                self._process(frame, self.get_clock().now())
            rem = interval - (time.monotonic() - t0)
            if rem > 0:
                time.sleep(rem)

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return
        self._process(frame, msg.header.stamp)

    # ── Main processing ────────────────────────────────────────────────────────
    def _process(self, frame: np.ndarray, stamp):
        with self._lock:
            now = time.monotonic()
            dt  = max(now - self._prev_t, 1e-3)
            self._prev_t = now
            self._frame_n += 1
            self._t_deque.append(now)
            H, W = frame.shape[:2]

            # ── Deteksi tangan ────────────────────────────────────────────────
            rgb    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = self._detector.detect(mp_img)

            hand_detected = False
            lm_px   = []
            gesture = "NONE"
            center  = None
            pose    = {"valid": False}

            if result.hand_landmarks:
                norm_lm  = result.hand_landmarks[0]
                world_lm = result.hand_world_landmarks[0]
                side     = result.handedness[0][0].category_name

                lm_px    = [(lm.x * W, lm.y * H) for lm in norm_lm]
                lm_world = [(wlm.x, wlm.y, wlm.z) for wlm in world_lm]
                gesture  = classify_gesture(lm_px, side)
                palm_idx = [LM_WRIST, LM_INDEX_MCP, LM_MIDDLE_MCP,
                            LM_RING_MCP, LM_PINKY_MCP]
                center       = np.mean([lm_px[i] for i in palm_idx], axis=0)
                pose         = self._pose_est.estimate(lm_px, lm_world)
                hand_detected = True

            # ── State machine + kontrol ───────────────────────────────────────
            rc, confirm_pct = self._update_state(
                gesture, hand_detected, center, pose, (W, H), dt)

            # ── Gambar UI ─────────────────────────────────────────────────────
            out   = frame.copy()
            if lm_px:
                draw_hand(out, lm_px)
            locked = (hand_detected and center is not None and
                      self._state == DroneState.TRACKING and
                      abs(center[0] - W/2) < self._box_px and
                      abs(center[1] - H/2) < self._box_px)
            draw_tracking_box(out, self._box_px, center, locked)
            fps = self._fps()
            draw_hud(out, self._state, gesture, confirm_pct, fps, rc, pose)

            if self._show_win:
                cv2.imshow("Tello Hand Control", out)
                if cv2.waitKey(1) == 27:          # ESC = emergency
                    self._emergency()

            # ── Publish debug image ───────────────────────────────────────────
            try:
                img_msg = self._bridge.cv2_to_imgmsg(out, "bgr8")
                self._pub_img.publish(img_msg)
            except Exception:
                pass

            self._last_rc   = rc
            self._last_pose = pose

    # ── State machine ──────────────────────────────────────────────────────────
    def _update_state(self, gesture: str, detected: bool,
                      center, pose: dict, frame_wh: tuple,
                      dt: float) -> tuple[dict | None, float]:
        """Returns (rc_dict | None, confirm_progress 0-1)."""
        W, H = frame_wh
        rc = None
        pct = 0.0

        # Gesture stability counter (debounce)
        if gesture == self._last_gesture and gesture != "NONE":
            self._gesture_stable = min(self._gesture_stable + 1, 120)
        else:
            self._gesture_stable = 0
        self._last_gesture = gesture

        # ── IDLE: tunggu ✌️ ───────────────────────────────────────────────────
        if self._state == DroneState.IDLE:
            if gesture == "PEACE":
                self._confirm_frames += 1
                pct = min(self._confirm_frames / self._confirm_frames_need, 1.0)
                if self._confirm_frames >= self._confirm_frames_need:
                    self._state          = DroneState.TAKING_OFF
                    self._confirm_frames = 0
                    self._wait_frames    = 0
                    self._pose_est.reset()
                    self._reset_controllers()
                    self.get_logger().info("✌ TAKEOFF!")
                    self._pub_takeoff.publish(Empty())
            else:
                self._confirm_frames = max(0, self._confirm_frames - 2)
                pct = self._confirm_frames / self._confirm_frames_need

        # ── TAKING_OFF: tunggu drone stabil ───────────────────────────────────
        elif self._state == DroneState.TAKING_OFF:
            self._wait_frames += 1
            wait_need = int(self._takeoff_wait_sec * self._fps())
            if wait_need < 1:
                wait_need = int(self._takeoff_wait_sec * 25)
            if self._wait_frames >= wait_need:
                self._state       = DroneState.TRACKING
                self._wait_frames = 0
                self.get_logger().info("TRACKING dimulai")

        # ── TRACKING: ikuti tangan ────────────────────────────────────────────
        elif self._state == DroneState.TRACKING:
            if detected and center is not None:
                rc = self._compute_rc(center, pose, W, H, dt)
                self._publish_rc(rc)
            else:
                # Tangan hilang → hover (kirim RC nol)
                self._stop_rc()
                self._reset_controllers()

            # Cek gesture ILY untuk landing
            if gesture == "ILY":
                self._confirm_frames += 1
                pct = min(self._confirm_frames / self._confirm_frames_need, 1.0)
                if self._confirm_frames >= self._confirm_frames_need:
                    self._state          = DroneState.LANDING
                    self._confirm_frames = 0
                    self._wait_frames    = 0
                    self.get_logger().info("🤟 LANDING!")
                    self._stop_rc()
                    self._pub_land.publish(Empty())
            else:
                self._confirm_frames = max(0, self._confirm_frames - 2)
                pct = self._confirm_frames / self._confirm_frames_need

        # ── LANDING: tunggu drone turun ───────────────────────────────────────
        elif self._state == DroneState.LANDING:
            self._wait_frames += 1
            wait_need = int(self._land_wait_sec * 25)
            if self._wait_frames >= wait_need:
                self._state       = DroneState.IDLE
                self._wait_frames = 0
                self._pose_est.reset()
                self.get_logger().info("Landed — kembali ke IDLE")

        return rc, pct

    # ── PD tracking ───────────────────────────────────────────────────────────
    def _compute_rc(self, center: np.ndarray, pose: dict,
                    W: int, H: int, dt: float) -> dict:
        """Hitung perintah RC dari error posisi tangan."""
        cx, cy = W / 2, H / 2
        ex = (center[0] - cx) / (W / 2)   # normalized [-1, 1], positif = tangan ke kanan
        ey = (center[1] - cy) / (H / 2)   # positif = tangan ke bawah (image Y)

        # Deadband pixel (konversi ke normalized)
        db_norm = self._db_px / (W / 2)
        if abs(ex) < db_norm:
            ex = 0.0
            self._ctrl_x.reset()
        if abs(ey) < db_norm:
            ey = 0.0
            self._ctrl_y.reset()

        # Drone bergerak SAMA ARAH dengan error untuk mengejar tangan
        lr =  self._ctrl_x.compute(ex, dt)
        ud = -self._ctrl_y.compute(ey, dt)   # Y image terbalik dengan ud

        # Depth control dari PnP
        fb = 0.0
        if pose.get("valid"):
            ez = self._target_z - pose["tz"]   # positif = tangan lebih jauh dari target
            if abs(ez) >= self._db_z:
                fb = self._ctrl_z.compute(ez, dt)
            else:
                self._ctrl_z.reset()

        # EMA smoothing pada output akhir
        a = self._rc_alpha
        self._rc_ema["lr"] = a * lr + (1 - a) * self._rc_ema["lr"]
        self._rc_ema["fb"] = a * fb + (1 - a) * self._rc_ema["fb"]
        self._rc_ema["ud"] = a * ud + (1 - a) * self._rc_ema["ud"]

        return {
            "lr":  float(np.clip(self._rc_ema["lr"], -self._max_lat, self._max_lat)),
            "fb":  float(np.clip(self._rc_ema["fb"], -self._max_dep, self._max_dep)),
            "ud":  float(np.clip(self._rc_ema["ud"], -self._max_lat, self._max_lat)),
            "yaw": 0.0,
        }

    def _publish_rc(self, rc: dict):
        msg = Twist()
        msg.linear.x  = rc["lr"]
        msg.linear.y  = rc["fb"]
        msg.linear.z  = rc["ud"]
        msg.angular.z = rc["yaw"]
        self._pub_ctrl.publish(msg)

    def _stop_rc(self):
        self._pub_ctrl.publish(Twist())
        self._rc_ema = {"lr": 0.0, "fb": 0.0, "ud": 0.0}

    def _reset_controllers(self):
        self._ctrl_x.reset()
        self._ctrl_y.reset()
        self._ctrl_z.reset()
        self._rc_ema = {"lr": 0.0, "fb": 0.0, "ud": 0.0}

    def _emergency(self):
        self._state = DroneState.EMERGENCY
        self._stop_rc()
        self._pub_land.publish(Empty())
        self.get_logger().error("EMERGENCY STOP!")

    # ── FPS ───────────────────────────────────────────────────────────────────
    def _fps(self) -> float:
        now = time.monotonic()
        return float(sum(1 for t in self._t_deque if now - t <= 1.0))


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = HandControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._show_win:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
