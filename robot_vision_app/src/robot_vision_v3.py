#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
INDUSTRIAL ROBOT VISION & COLLISION AVOIDANCE DASHBOARD v3.0
================================================================
Production-ready система трекинга рук для коллаборативных роботов.

Все 10 улучшений:
1. Predictive Collision Detection (3D-экстраполяция)
2. Safety-rated E-Stop (Modbus TCP)
3. Автоматическая калибровка камеры
4. YAML-конфигурация (Pydantic)
5. Multiprocessing для YOLO
6. Structured JSON Logging
7. Web Dashboard (FastAPI + WebSocket)
8. Anomaly Detection (Isolation Forest)
9. Multi-Camera Fusion
10. Black Box Recorder (видео + телеметрия)
"""

import os
import sys
import time
import csv
import math
import json
import socket
import argparse
import threading
import queue
import urllib.request
import urllib.error
import logging
import logging.handlers
import asyncio
import warnings
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import List, Tuple, Optional, Dict, Any, Union
from pathlib import Path
from datetime import datetime, timezone
from contextlib import asynccontextmanager

import numpy as np
import cv2
from scipy.fft import fft, fftfreq
from scipy.signal.windows import hann

# =============================================================================
# CONSTANTS
# =============================================================================

WINDOW_NAME = "Industrial Robot Vision — q:Quit r:Reset s:Snapshot a:FFT 1-4:Toggles"

RING_BUFFER_SIZE = 300
VELOCITY_WINDOW = 5
JITTER_WINDOW = 10
FFT_WINDOW = 128
HEATMAP_BINS = 32

# Project root: this file lives in <root>/src/, while models/, config/ and the
# runtime output dirs live in <root>/. Resolving from __file__ keeps paths
# correct regardless of the current working directory.
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "config.yaml"

SNAPSHOT_DIR = os.environ.get("SNAPSHOT_DIR", str(PROJECT_ROOT / "snapshots"))
CSV_DIR = os.environ.get("CSV_DIR", str(PROJECT_ROOT / "telemetry"))

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15),
    (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
]

HAND_MODEL_URLS = [
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task",
]

KP = {
    "left_shoulder": 5, "right_shoulder": 6,
    "left_elbow": 7, "right_elbow": 8,
    "left_wrist": 9, "right_wrist": 10
}
SIDES = ("left", "right")
JOINTS = ("shoulder", "elbow", "wrist")
ARM_COLORS = {"left": (0, 200, 255), "right": (255, 120, 0)}


# =============================================================================
# FEATURE 4: YAML CONFIGURATION (Pydantic)
# =============================================================================

try:
    from pydantic import BaseModel, Field, validator, ConfigDict
    from pydantic import ValidationError
    HAS_PYDANTIC = True
except ImportError:
    HAS_PYDANTIC = False
    print("[Warning] pydantic not installed, using dict config fallback")

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False
    print("[Warning] pyyaml not installed, using defaults")

if HAS_PYDANTIC:
    class CameraConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        id: Union[str, int] = "0"
        width: int = Field(default=1280, ge=640, le=3840)
        height: int = Field(default=720, ge=480, le=2160)
        fps: int = Field(default=30, ge=1, le=120)
        calibration_file: Optional[str] = None
        mirror: bool = True

    class SafetyConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        emergency_zone_mm: float = Field(default=200, gt=50, lt=2000)
        warning_zone_mm: float = Field(default=500, gt=100, lt=5000)
        predictive_horizon_ms: int = Field(default=500, ge=100, le=2000)
        plc_ip: Optional[str] = None
        plc_port: int = Field(default=502, ge=1, le=65535)
        enable_hardware_estop: bool = False

    class YOLOConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        path: str = "yolo11n-pose.pt"
        device: str = Field(default="auto", pattern=r"^(auto|cpu|cuda)$")
        imgsz: int = Field(default=480, ge=160, le=1280)
        conf: float = Field(default=0.5, ge=0.1, le=1.0)
        skip_frames: int = Field(default=2, ge=1, le=10)
        use_multiprocessing: bool = False

    class MediaPipeConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        max_hands: int = Field(default=2, ge=1, le=4)
        min_conf: float = Field(default=0.5, ge=0.1, le=1.0)

    class TelemetryConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        udp_target: str = "127.0.0.1:9090"
        csv_dir: str = "./telemetry"
        rate_limit_hz: int = Field(default=10, ge=1, le=60)
        enable_blackbox: bool = True
        blackbox_dir: str = "./blackbox"
        blackbox_max_gb: float = Field(default=10.0, ge=1.0, le=100.0)

    class WebDashboardConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        enabled: bool = False
        host: str = "0.0.0.0"
        port: int = Field(default=8080, ge=1024, le=65535)

    class AnomalyConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        enabled: bool = False
        contamination: float = Field(default=0.05, ge=0.01, le=0.5)
        window_size: int = Field(default=100, ge=50, le=1000)

    class MultiCameraConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        enabled: bool = False
        cameras: List[CameraConfig] = []
        extrinsics_files: List[str] = []

    class AppConfig(BaseModel):
        model_config = ConfigDict(extra="ignore")
        camera: CameraConfig = CameraConfig()
        safety: SafetyConfig = SafetyConfig()
        models: Dict[str, Any] = {"yolo": YOLOConfig(), "mediapipe": MediaPipeConfig()}
        telemetry: TelemetryConfig = TelemetryConfig()
        web_dashboard: WebDashboardConfig = WebDashboardConfig()
        anomaly: AnomalyConfig = AnomalyConfig()
        multicamera: MultiCameraConfig = MultiCameraConfig()
        snapshot_dir: str = "./snapshots"
        no_show: bool = False
        log_level: str = Field(default="INFO", pattern=r"^(DEBUG|INFO|WARNING|ERROR|CRITICAL)$")

else:
    AppConfig = dict


def load_config(config_path: Optional[str] = None) -> Union[AppConfig, dict]:
    """Load configuration from YAML or use defaults."""
    if not HAS_PYDANTIC or not HAS_YAML:
        print("[Config] Using default configuration")
        return AppConfig() if HAS_PYDANTIC else {}

    if config_path and os.path.exists(config_path):
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        print(f"[Config] Loaded from {config_path}")
        return AppConfig(**data)

    default_paths = [
        str(DEFAULT_CONFIG_PATH),
        "config.yaml",
        os.path.join(os.path.dirname(__file__), "config.yaml"),
        "/etc/robot_vision/config.yaml"
    ]
    for path in default_paths:
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            print(f"[Config] Loaded from {path}")
            return AppConfig(**data)

    print("[Config] No config file found, using defaults")
    return AppConfig()


# =============================================================================
# FEATURE 6: STRUCTURED JSON LOGGING
# =============================================================================

class JSONFormatter(logging.Formatter):
    """Formatter for ELK/Grafana Loki integration."""

    def format(self, record):
        log_obj = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "function": record.funcName,
            "line": record.lineno,
            "thread": record.thread,
        }
        if hasattr(record, "extra"):
            log_obj.update(record.extra)
        if record.exc_info:
            log_obj["exception"] = self.formatException(record.exc_info)
        return json.dumps(log_obj, ensure_ascii=False, default=str)


def setup_logging(log_level: str = "INFO", log_dir: Optional[str] = None):
    """Setup logging: stdout + rotating file."""
    if log_dir is None:
        log_dir = str(PROJECT_ROOT / "logs")
    os.makedirs(log_dir, exist_ok=True)

    logger = logging.getLogger("robot_vision")
    logger.setLevel(getattr(logging, log_level))
    logger.handlers = []

    json_handler = logging.handlers.RotatingFileHandler(
        os.path.join(log_dir, "robot_vision.json"),
        maxBytes=10*1024*1024,
        backupCount=5
    )
    json_handler.setFormatter(JSONFormatter())
    logger.addHandler(json_handler)

    console = logging.StreamHandler(sys.stdout)
    console.setFormatter(logging.Formatter(
        "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
    ))
    logger.addHandler(console)

    return logger


logger = logging.getLogger("robot_vision")


# =============================================================================
# CAMERA
# =============================================================================

def open_camera(camera_id="0"):
    """Open camera with specified settings."""
    cap = cv2.VideoCapture(int(camera_id))
    if not cap.isOpened():
        logger.warning(f"Could not open camera {camera_id}, trying default")
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Failed to open any camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    logger.info(f"Camera opened: {actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.1f}fps")

    return cap


# =============================================================================
# FEATURE 3: CAMERA CALIBRATION
# =============================================================================

class CameraCalibration:
    """Camera calibration via chessboard + undistort."""

    def __init__(self, chessboard_size=(9, 6), square_size=25.0):
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.new_camera_matrix: Optional[np.ndarray] = None
        self.roi: Optional[Tuple] = None
        self.is_calibrated = False

    def load(self, filepath: str) -> bool:
        if not os.path.exists(filepath):
            logger.warning(f"Calibration file not found: {filepath}")
            return False
        try:
            data = np.load(filepath)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            self.new_camera_matrix = data.get('new_camera_matrix', self.camera_matrix)
            self.roi = tuple(data['roi']) if 'roi' in data else None
            self.is_calibrated = True
            logger.info(f"Calibration loaded from {filepath}")
            return True
        except Exception as e:
            logger.error(f"Failed to load calibration: {e}")
            return False

    def save(self, filepath: str):
        if not self.is_calibrated:
            return
        np.savez(filepath,
                 camera_matrix=self.camera_matrix,
                 dist_coeffs=self.dist_coeffs,
                 new_camera_matrix=self.new_camera_matrix,
                 roi=np.array(self.roi) if self.roi else np.array([]))
        logger.info(f"Calibration saved to {filepath}")

    def calibrate_from_images(self, image_paths: List[str]) -> bool:
        objp = np.zeros((self.chessboard_size[0]*self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        objpoints, imgpoints = [], []

        for path in image_paths:
            img = cv2.imread(path)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                imgpoints.append(corners2)

        if len(objpoints) < 3:
            logger.error("Not enough calibration images found")
            return False

        h, w = gray.shape[:2]
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, (w, h), None, None
        )
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        self.is_calibrated = True
        logger.info(f"Calibration complete. Reprojection error: {ret:.4f}")
        return True

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        if not self.is_calibrated:
            return frame
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs,
                            None, self.new_camera_matrix)

    def get_focal_length(self) -> float:
        if self.is_calibrated and self.camera_matrix is not None:
            return float(self.camera_matrix[0, 0])
        return 1000.0


# =============================================================================
# FEATURE 1: PREDICTIVE COLLISION DETECTION
# =============================================================================

class TrajectoryPredictor:
    """3D trajectory extrapolation via least squares."""

    def __init__(self, horizon_ms: int = 500):
        self.horizon = horizon_ms / 1000.0
        self._buffer: Dict[str, deque] = {}
        self._lock = threading.Lock()

    def add_point(self, label: str, x: float, y: float, z: float, t: float):
        with self._lock:
            if label not in self._buffer:
                self._buffer[label] = deque(maxlen=20)
            self._buffer[label].append((x, y, z, t))

    def predict_position(self, label: str) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            buf = self._buffer.get(label)
            if not buf or len(buf) < 5:
                return None
            data = np.array(list(buf))

        t = data[:, 3] - data[-1, 3]
        if np.max(t) - np.min(t) < 0.05:
            return None

        def fit_velocity(coord_idx):
            coeffs = np.polyfit(t, data[:, coord_idx], 1)
            return coeffs[0]

        vx = fit_velocity(0)
        vy = fit_velocity(1)
        vz = fit_velocity(2)

        x_pred = data[-1, 0] + vx * self.horizon
        y_pred = data[-1, 1] + vy * self.horizon
        z_pred = data[-1, 2] + vz * self.horizon

        return x_pred, y_pred, z_pred

    def predict_all(self) -> Dict[str, Tuple[float, float, float]]:
        results = {}
        with self._lock:
            labels = list(self._buffer.keys())
        for label in labels:
            pred = self.predict_position(label)
            if pred:
                results[label] = pred
        return results

    def reset(self):
        with self._lock:
            self._buffer.clear()


# =============================================================================
# FEATURE 2: SAFETY-RATED E-STOP (Modbus TCP)
# =============================================================================

try:
    from pymodbus.client import ModbusTcpClient
    HAS_MODBUS = True
except ImportError:
    HAS_MODBUS = False
    logger.warning("pymodbus not installed, hardware E-Stop disabled")


class SafetyController:
    """Two-level safety: WARNING -> SLOW -> STOP via Modbus TCP."""

    STATES = {0b00: "RUN", 0b01: "STOP", 0b10: "SLOW", 0b11: "WARNING"}

    def __init__(self, plc_ip: Optional[str] = None, plc_port: int = 502,
                 emergency_zone_mm: float = 200, warning_zone_mm: float = 500,
                 enable_hardware: bool = False):
        self.plc_ip = plc_ip
        self.plc_port = plc_port
        self.emergency_zone = emergency_zone_mm
        self.warning_zone = warning_zone_mm
        self.enable_hardware = enable_hardware and HAS_MODBUS and plc_ip
        self._last_state = "RUN"
        self._state_lock = threading.Lock()
        self._client: Optional[Any] = None
        self._consecutive_detections = 0
        self._detection_threshold = 3

        if self.enable_hardware:
            try:
                self._client = ModbusTcpClient(plc_ip, port=plc_port, timeout=1)
                self._client.connect()
                logger.info(f"Safety PLC connected: {plc_ip}:{plc_port}")
            except Exception as e:
                logger.error(f"Failed to connect to Safety PLC: {e}")
                self.enable_hardware = False

    def evaluate(self, hand_data: Dict[str, dict], predictions: Dict[str, Tuple[float, float, float]]) -> str:
        min_z = float('inf')
        min_pred_z = float('inf')
        min_ttc = float('inf')

        for label, data in hand_data.items():
            z = data.get("z_mm", 9999)
            ttc = data.get("ttc_sec", float('inf'))
            min_z = min(min_z, z)
            min_ttc = min(min_ttc, ttc)

            if label in predictions:
                _, _, pred_z = predictions[label]
                min_pred_z = min(min_pred_z, pred_z)

        new_state = "RUN"
        if min_z < self.emergency_zone or min_pred_z < self.emergency_zone or min_ttc < 0.2:
            new_state = "STOP"
        elif min_z < self.warning_zone or min_pred_z < self.warning_zone or min_ttc < 0.6:
            new_state = "SLOW"

        if new_state == "STOP":
            self._consecutive_detections += 1
            if self._consecutive_detections < self._detection_threshold:
                new_state = "SLOW"
        else:
            self._consecutive_detections = max(0, self._consecutive_detections - 1)

        with self._state_lock:
            if new_state != self._last_state:
                self._send_safety_signal(new_state)
                self._last_state = new_state
                logger.warning("Safety state changed", extra={
                    "old_state": self._last_state,
                    "new_state": new_state,
                    "min_z": min_z,
                    "min_pred_z": min_pred_z,
                    "min_ttc": min_ttc
                })

        return new_state

    def _send_safety_signal(self, state: str):
        state_code = {"RUN": 0b00, "STOP": 0b01, "SLOW": 0b10, "WARNING": 0b11}.get(state, 0b00)

        if self.enable_hardware and self._client and self._client.connected:
            try:
                self._client.write_register(0, state_code, slave=1)
                logger.info(f"Hardware safety signal sent: {state} ({state_code})")
            except Exception as e:
                logger.error(f"Failed to write to PLC: {e}")
        else:
            logger.info(f"Software safety state: {state} (no hardware connection)")

    def get_state(self) -> str:
        with self._state_lock:
            return self._last_state

    def close(self):
        if self._client:
            try:
                self._client.close()
                logger.info("Safety PLC connection closed")
            except:
                pass


# =============================================================================
# FEATURE 8: ANOMALY DETECTION (Isolation Forest)
# =============================================================================

try:
    from sklearn.ensemble import IsolationForest
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False
    logger.warning("scikit-learn not installed, anomaly detection disabled")


class AnomalyDetector:
    """Detect anomalous behavior (seizures, attacks)."""

    FEATURES = ["speed_px_s", "accel_px_s2", "jitter", "depth_variance"]

    def __init__(self, contamination: float = 0.05, window_size: int = 100):
        self.contamination = contamination
        self.window_size = window_size
        self.model: Optional[Any] = None
        self._buffer: deque = deque(maxlen=window_size)
        self._is_trained = False
        self._lock = threading.Lock()

        if HAS_SKLEARN:
            self.model = IsolationForest(
                contamination=contamination,
                random_state=42,
                n_estimators=100
            )

    def add_observation(self, speed: float, accel: float, jitter: float, depth_var: float):
        with self._lock:
            self._buffer.append([speed, accel, jitter, depth_var])

    def fit(self):
        if not HAS_SKLEARN or len(self._buffer) < 50:
            return False
        with self._lock:
            X = np.array(self._buffer)
        self.model.fit(X)
        self._is_trained = True
        logger.info(f"Anomaly model trained on {len(X)} samples")
        return True

    def predict(self, speed: float, accel: float, jitter: float, depth_var: float) -> Tuple[bool, float]:
        if not self._is_trained or not HAS_SKLEARN:
            return False, 0.0

        features = np.array([[speed, accel, jitter, depth_var]])
        prediction = self.model.predict(features)[0]
        score = self.model.score_samples(features)[0]

        is_anomaly = (prediction == -1)
        if is_anomaly:
            logger.warning("Anomaly detected", extra={
                "speed": speed,
                "accel": accel,
                "jitter": jitter,
                "depth_var": depth_var,
                "anomaly_score": score
            })

        return is_anomaly, score

    def reset(self):
        with self._lock:
            self._buffer.clear()
            self._is_trained = False


# =============================================================================
# FEATURE 10: BLACK BOX RECORDER
# =============================================================================

class BlackBoxRecorder:
    """Video + synchronized telemetry recording for incident investigation."""

    def __init__(self, output_dir: str = "./blackbox", max_gb: float = 10.0,
                 fps: int = 30, resolution: Tuple[int, int] = (1280, 720)):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.max_bytes = max_gb * 1024**3
        self.fps = fps
        self.resolution = resolution

        self._video_writer: Optional[cv2.VideoWriter] = None
        self._csv_file: Optional[Any] = None
        self._csv_writer: Optional[Any] = None
        self._session_start: Optional[float] = None
        self._lock = threading.Lock()
        self._is_recording = False
        self._frame_queue: queue.Queue = queue.Queue(maxsize=30)
        self._writer_thread: Optional[threading.Thread] = None

    def start_session(self):
        with self._lock:
            self._cleanup_old_files()

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_path = self.output_dir / f"session_{timestamp}.mp4"
            csv_path = self.output_dir / f"session_{timestamp}.csv"

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self._video_writer = cv2.VideoWriter(
                str(video_path), fourcc, self.fps, self.resolution
            )

            self._csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=[
                "frame_id", "timestamp", "hand", "x_mm", "y_mm", "z_mm",
                "speed_px_s", "ttc_sec", "gesture", "safety_state", "anomaly_score"
            ])
            self._csv_writer.writeheader()

            self._session_start = time.time()
            self._is_recording = True

            self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
            self._writer_thread.start()

            logger.info(f"BlackBox session started: {video_path}")

    def _writer_loop(self):
        while self._is_recording:
            try:
                item = self._frame_queue.get(timeout=0.5)
                if item is None:
                    break
                frame, telemetry = item
                if self._video_writer:
                    self._video_writer.write(frame)
                if telemetry and self._csv_writer:
                    # Flatten telemetry for CSV - remove nested dicts
                    flat_telemetry = {}
                    for k, v in telemetry.items():
                        if isinstance(v, dict):
                            # Skip nested dicts or flatten them
                            continue
                        flat_telemetry[k] = v
                    self._csv_writer.writerow(flat_telemetry)
            except queue.Empty:
                continue

    def write_frame(self, frame: np.ndarray, telemetry: Optional[dict] = None):
        if not self._is_recording:
            return

        if frame.shape[:2] != (self.resolution[1], self.resolution[0]):
            frame = cv2.resize(frame, self.resolution)

        try:
            self._frame_queue.put_nowait((frame, telemetry))
        except queue.Full:
            pass

    def _cleanup_old_files(self):
        files = sorted(self.output_dir.glob("session_*"), key=lambda p: p.stat().st_mtime)
        total_size = sum(f.stat().st_size for f in files)

        while total_size > self.max_bytes * 0.8 and len(files) > 1:
            oldest = files.pop(0)
            total_size -= oldest.stat().st_size
            oldest.unlink()
            logger.info(f"Removed old blackbox file: {oldest}")

    def stop(self):
        with self._lock:
            self._is_recording = False
            try:
                self._frame_queue.put(None, timeout=1.0)
            except:
                pass

            if self._writer_thread:
                self._writer_thread.join(timeout=2.0)

            if self._video_writer:
                self._video_writer.release()
            if self._csv_file:
                self._csv_file.close()

            logger.info("BlackBox session stopped")


# =============================================================================
# FEATURE 9: MULTI-CAMERA FUSION
# =============================================================================

class MultiCameraFusion:
    """Triangulate 3D coordinates from multiple cameras."""

    def __init__(self, calibrations: List[CameraCalibration], extrinsics: List[np.ndarray]):
        self.calibrations = calibrations
        self.extrinsics = extrinsics
        self.projection_matrices = self._build_projection_matrices()

    def _build_projection_matrices(self) -> List[np.ndarray]:
        P_list = []
        for calib, ext in zip(self.calibrations, self.extrinsics):
            if not calib.is_calibrated:
                continue
            K = calib.camera_matrix
            RT = ext[:3, :]
            P = K @ RT
            P_list.append(P)
        return P_list

    def triangulate(self, uv_points: List[Tuple[float, float]]) -> Optional[Tuple[float, float, float]]:
        if len(uv_points) < 2 or len(uv_points) != len(self.projection_matrices):
            return None

        A = []
        for (u, v), P in zip(uv_points, self.projection_matrices):
            A.append(u * P[2, :] - P[0, :])
            A.append(v * P[2, :] - P[1, :])

        A = np.array(A)
        _, _, Vt = np.linalg.svd(A)
        X = Vt[-1]
        X = X / X[3]

        return float(X[0]), float(X[1]), float(X[2])

    def fuse_detections(self, detections_per_camera: List[Dict[str, Tuple[float, float]]]) -> Dict[str, Tuple[float, float, float]]:
        labels = set()
        for dets in detections_per_camera:
            labels.update(dets.keys())

        fused = {}
        for label in labels:
            uv_points = []
            for dets in detections_per_camera:
                if label in dets:
                    uv_points.append(dets[label])

            if len(uv_points) >= 2:
                xyz = self.triangulate(uv_points)
                if xyz:
                    fused[label] = xyz

        return fused


# =============================================================================
# CORE: KALMAN, FILTERS, MATH, RING BUFFERS
# =============================================================================

class KalmanFilter1D:
    """1D Kalman Filter with adaptive Q based on dt."""

    def __init__(self, dt: float = 1/30.0, q_scale: float = 1.0):
        self.x = np.array([[0.0], [0.0]])
        self.P = np.eye(2) * 1000.0
        self.F = np.array([[1.0, dt], [0.0, 1.0]])
        self.H = np.array([[1.0, 0.0]])
        self.R = np.array([[50.0]])
        self.Q_base = np.array([[1.0, 0.0], [0.0, 50.0]])
        self.q_scale = q_scale
        self.last_t = 0.0

    def update(self, t: float, z_meas: float) -> Tuple[float, float]:
        if self.last_t == 0.0:
            self.x[0, 0] = z_meas
            self.last_t = t
            return z_meas, 0.0
        dt = t - self.last_t
        if dt <= 0:
            dt = 1e-6
        self.F[0, 1] = dt
        self.Q = self.Q_base * self.q_scale * dt
        self.last_t = t
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q
        y = z_meas - (self.H @ x_pred)[0, 0]
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        self.x = x_pred + K * y
        self.P = (np.eye(2) - K @ self.H) @ P_pred
        return float(self.x[0, 0]), float(self.x[1, 0])


class OneEuroFilter:
    def __init__(self, t0: float, x0: np.ndarray, min_cutoff: float = 1.0, beta: float = 0.007, d_cutoff: float = 1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = np.array(x0, dtype=np.float64)
        self.dx_prev = np.zeros_like(x0, dtype=np.float64)
        self.t_prev = t0

    def __call__(self, t: float, x: np.ndarray) -> np.ndarray:
        dt = t - self.t_prev
        if dt <= 0:
            return self.x_prev
        x = np.array(x, dtype=np.float64)
        dx = (x - self.x_prev) / dt
        alpha_d = 1.0 / (1.0 + self.d_cutoff / (2 * math.pi * dt))
        dx_hat = alpha_d * dx + (1.0 - alpha_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        alpha = 1.0 / (1.0 + cutoff / (2 * math.pi * dt))
        x_hat = alpha * x + (1.0 - alpha) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


class RobotVisionMath:
    @staticmethod
    def estimate_raw_depth(landmarks, W: int, H: int, focal_length_px: float = 1000.0) -> float:
        if len(landmarks) < 18:
            return 0.0
        p5 = np.array([landmarks[5].x * W, landmarks[5].y * H])
        p17 = np.array([landmarks[17].x * W, landmarks[17].y * H])
        w_pixel = np.linalg.norm(p5 - p17)
        if w_pixel == 0:
            return 0.0
        w_real_mm = 80.0
        return (focal_length_px * w_real_mm) / w_pixel

    @staticmethod
    def pixel_to_metric(px: float, py: float, Z_mm: float, W: int, H: int, focal_length_px: float = 1000.0) -> Tuple[float, float]:
        cx, cy = W / 2.0, H / 2.0
        X_mm = (px - cx) * Z_mm / focal_length_px
        Y_mm = (py - cy) * Z_mm / focal_length_px
        return X_mm, Y_mm

    @staticmethod
    def create_safety_envelope(points: np.ndarray, current_speed_px_s: float, base_margin: int = 40) -> np.ndarray:
        if len(points) < 3:
            x, y, w, h = cv2.boundingRect(points)
            margin = min(int(base_margin + 0.04 * current_speed_px_s), 150)
            return np.array([
                [x - margin, y - margin],
                [x + w + margin, y - margin],
                [x + w + margin, y + h + margin],
                [x - margin, y + h + margin]
            ], dtype=np.int32).reshape((-1, 1, 2))

        dynamic_margin = int(base_margin + 0.04 * current_speed_px_s)
        dynamic_margin = min(dynamic_margin, 150)
        hull = cv2.convexHull(points)
        M = cv2.moments(hull)
        if M["m00"] == 0:
            return hull
        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
        expanded_hull = []
        for pt in hull:
            x, y = pt[0]
            dx, dy = x - cx, y - cy
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            nx, ny = dx / length, dy / length
            expanded_hull.append([int(x + nx * dynamic_margin), int(y + ny * dynamic_margin)])
        return np.array(expanded_hull, dtype=np.int32).reshape((-1, 1, 2))

    @staticmethod
    def calculate_elbow_angle(shoulder: Tuple[float, float], elbow: Tuple[float, float], wrist: Tuple[float, float]) -> float:
        ba = np.array(shoulder) - np.array(elbow)
        bc = np.array(wrist) - np.array(elbow)
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6)
        return float(np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0))))

    @staticmethod
    def detect_gesture(landmarks) -> str:
        if len(landmarks) < 21:
            return "Unknown"
        p4 = np.array([landmarks[4].x, landmarks[4].y, landmarks[4].z])
        p8 = np.array([landmarks[8].x, landmarks[8].y, landmarks[8].z])
        pinch_dist = np.linalg.norm(p4 - p8)
        p0 = np.array([landmarks[0].x, landmarks[0].y, landmarks[0].z])
        tips = [4, 8, 12, 16, 20]
        avg_tip_dist = np.mean([np.linalg.norm(p0 - np.array([landmarks[t].x, landmarks[t].y, landmarks[t].z])) for t in tips])
        if pinch_dist < 0.04:
            return "Pinch Trigger"
        if avg_tip_dist < 0.25:
            return "Fist (Stop)"
        return "Open Hand"

    # MediaPipe landmark indices
    WRIST = 0
    INDEX_MCP = 5
    PINKY_MCP = 17
    FINGERTIPS = (4, 8, 12, 16, 20)

    @staticmethod
    def compute_hand_metrics(landmarks, W: int, H: int, z_mm: float,
                             focal_length_px: float = 1000.0) -> Dict[str, Any]:
        """Center of mass, occupied area/volume and pointing direction of a hand.

        Designed for the ROS2 bridge: returns the centroid of the yellow safety
        zone (3D, mm, camera frame), the area/volume it occupies, and a unit
        pointing vector (wrist -> fingertips, i.e. "fingers forward") plus its
        yaw angle. Also exposes the palm normal as an orientation hint.
        """
        if landmarks is None or len(landmarks) < 21 or z_mm <= 0:
            return {}

        cx_img, cy_img = W / 2.0, H / 2.0
        scale = z_mm / focal_length_px  # mm per pixel at this depth

        # 3D point (mm, camera frame) for every landmark.
        # MediaPipe's relative z is ~same scale as normalised x, wrist ~ 0.
        pts2d = np.empty((len(landmarks), 2), dtype=np.float32)
        pts3d = np.empty((len(landmarks), 3), dtype=np.float64)
        for i, lm in enumerate(landmarks):
            px, py = lm.x * W, lm.y * H
            pts2d[i] = (px, py)
            pts3d[i] = ((px - cx_img) * scale, (py - cy_img) * scale,
                        z_mm + (lm.z * W) * scale)

        # --- Center of mass of the yellow zone (convex hull of the hand) ---
        hull = cv2.convexHull(pts2d)
        M = cv2.moments(hull)
        if M["m00"] != 0:
            cx_px = M["m10"] / M["m00"]
            cy_px = M["m01"] / M["m00"]
        else:
            cx_px, cy_px = float(pts2d[:, 0].mean()), float(pts2d[:, 1].mean())

        centroid_x_mm = (cx_px - cx_img) * scale
        centroid_y_mm = (cy_px - cy_img) * scale
        centroid_z_mm = float(pts3d[:, 2].mean())

        # --- Area / volume of the occupied zone ---
        area_px = float(cv2.contourArea(hull))
        area_mm2 = area_px * scale * scale
        thickness_mm = max(float(pts3d[:, 2].max() - pts3d[:, 2].min()), 30.0)
        volume_mm3 = area_mm2 * thickness_mm

        # --- Pointing / reach direction: wrist -> mean of fingertips ---
        wrist = pts3d[RobotVisionMath.WRIST]
        tips = pts3d[list(RobotVisionMath.FINGERTIPS)].mean(axis=0)
        direction = tips - wrist
        norm = np.linalg.norm(direction)
        direction = direction / norm if norm > 1e-6 else np.array([0.0, 0.0, 1.0])
        angle_deg = float(np.degrees(np.arctan2(direction[1], direction[0])))

        # --- Palm normal (cross product) as an orientation hint ---
        v1 = pts3d[RobotVisionMath.INDEX_MCP] - wrist
        v2 = pts3d[RobotVisionMath.PINKY_MCP] - wrist
        normal = np.cross(v1, v2)
        n_norm = np.linalg.norm(normal)
        normal = normal / n_norm if n_norm > 1e-6 else np.array([0.0, 0.0, -1.0])

        return {
            "centroid_x_mm": float(centroid_x_mm),
            "centroid_y_mm": float(centroid_y_mm),
            "centroid_z_mm": float(centroid_z_mm),
            "centroid_px": (float(cx_px), float(cy_px)),
            "area_mm2": float(area_mm2),
            "volume_mm3": float(volume_mm3),
            "dir_x": float(direction[0]),
            "dir_y": float(direction[1]),
            "dir_z": float(direction[2]),
            "angle_deg": angle_deg,
            "normal_x": float(normal[0]),
            "normal_y": float(normal[1]),
            "normal_z": float(normal[2]),
        }


class NumpyRingBuffer:
    """Thread-safe ring buffer with vectorized get_latest."""

    def __init__(self, capacity: int, dtype=np.float64, shape=()):
        self.capacity = capacity
        if shape == ():
            self._buf = np.zeros(capacity, dtype=dtype)
        else:
            self._buf = np.zeros((capacity,) + tuple(shape), dtype=dtype)
        self._head, self._size, self._full = 0, 0, False
        self._lock = threading.Lock()

    def append(self, value):
        with self._lock:
            self._buf[self._head] = value
            self._head = (self._head + 1) % self.capacity
            if not self._full:
                self._size += 1
                if self._size >= self.capacity:
                    self._full = True

    @property
    def size(self) -> int:
        with self._lock:
            return self._size

    def get_latest(self, n: int) -> np.ndarray:
        with self._lock:
            if n >= self._size:
                n = self._size
            if n == 0:
                return np.empty((0,) + self._buf.shape[1:], dtype=self._buf.dtype)
            indices = np.arange(self._head - n, self._head) % self.capacity
            return self._buf[indices].copy()

    def clear(self):
        with self._lock:
            self._head, self._size, self._full = 0, 0, False


class MultiChannelRingBuffer:
    def __init__(self, capacity: int, channels: int, dtype=np.float64):
        self.capacity = capacity
        self.channels = channels
        self._buf = np.zeros((capacity, channels), dtype=dtype)
        self._head, self._size, self._full = 0, 0, False
        self._lock = threading.Lock()

    def append(self, values: np.ndarray):
        with self._lock:
            self._buf[self._head] = values
            self._head = (self._head + 1) % self.capacity
            if not self._full:
                self._size += 1
                if self._size >= self.capacity:
                    self._full = True

    @property
    def size(self) -> int:
        with self._lock:
            return self._size

    def get_latest(self, n: int) -> np.ndarray:
        with self._lock:
            if n >= self._size:
                n = self._size
            if n == 0:
                return np.empty((0, self.channels), dtype=self._buf.dtype)
            indices = np.arange(self._head - n, self._head) % self.capacity
            return self._buf[indices].copy()

    def clear(self):
        with self._lock:
            self._head, self._size, self._full = 0, 0, False


@dataclass
class TimingStats:
    window: int = 60
    _times: deque = field(default_factory=lambda: deque(maxlen=60))
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def add(self, ms: float):
        with self._lock:
            self._times.append(ms)

    def report(self) -> Dict[str, float]:
        with self._lock:
            if not self._times:
                return {"avg": 0.0, "p50": 0.0, "p99": 0.0, "n": 0}
            arr = np.array(self._times)
            n = len(arr)
            return {
                "avg": float(np.mean(arr)),
                "p50": float(np.median(arr)),
                "p99": float(np.percentile(arr, 99)),
                "n": n
            }


class StageTimer:
    def __init__(self, stats: TimingStats):
        self.stats = stats
    def __enter__(self):
        self.start = time.perf_counter()
        return self
    def __exit__(self, *args):
        self.stats.add((time.perf_counter() - self.start) * 1000.0)


class ThreadedCamera:
    """Thread-safe camera with atomic latest frame replacement."""

    def __init__(self, cap: cv2.VideoCapture, drop_frames: bool = True):
        self.cap = cap
        self.drop_frames = drop_frames
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        self._stopped = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="CameraThread")
        self._thread.start()

    def _loop(self):
        while not self._stopped.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.001)
                continue
            with self._frame_lock:
                self._latest_frame = frame

    def read(self, timeout: float = 1.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._frame_lock:
                if self._latest_frame is not None:
                    return True, self._latest_frame.copy()
            time.sleep(0.001)
        return False, None

    def release(self):
        self._stopped.set()
        self._thread.join(timeout=2.0)
        self.cap.release()


class RobotTelemetryStreamer:
    """UDP streamer with error logging and rate limiting."""

    def __init__(self, ip: str = "127.0.0.1", port: int = 9090):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self._error_count = 0
        self._last_error_log = 0.0
        self._lock = threading.Lock()
        logger.info(f"UDP Streamer initialized on {ip}:{port}")

    def send_packet(self, data: dict):
        try:
            payload = json.dumps(data).encode('utf-8')
            if len(payload) > 65507:
                payload = json.dumps({"timestamp": data.get("timestamp"), "error": "payload_too_large"}).encode('utf-8')
            self.sock.sendto(payload, (self.ip, self.port))
            with self._lock:
                self._error_count = max(0, self._error_count - 1)
        except BlockingIOError:
            with self._lock:
                self._error_count += 1
                now = time.time()
                if now - self._last_error_log > 5.0:
                    logger.warning(f"UDP buffer full, dropped packets: {self._error_count}")
                    self._last_error_log = now
        except (OSError, socket.error) as e:
            with self._lock:
                self._error_count += 1
                now = time.time()
                if now - self._last_error_log > 5.0:
                    logger.error(f"Network error: {e}")
                    self._last_error_log = now
        except Exception as e:
            with self._lock:
                now = time.time()
                if now - self._last_error_log > 5.0:
                    logger.error(f"Unexpected telemetry error: {e}")
                    self._last_error_log = now


class AsyncCSVWriter:
    """Thread-safe async CSV writer with graceful shutdown."""

    def __init__(self, filepath, fieldnames):
        os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
        self.filepath = filepath
        self.fieldnames = fieldnames
        self.queue: queue.Queue = queue.Queue(maxsize=2000)
        self._stopped = threading.Event()
        self._flush_event = threading.Event()
        self.file = open(filepath, 'w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=fieldnames)
        self.writer.writeheader()
        self.thread = threading.Thread(target=self._writer_loop, daemon=True, name="CSVWriter")
        self.thread.start()

    def _writer_loop(self):
        while not self._stopped.is_set():
            try:
                row = self.queue.get(timeout=0.1)
                if row is None:
                    break
                self.writer.writerow(row)
                if self._flush_event.is_set():
                    self.file.flush()
                    self._flush_event.clear()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"CSV Writer Error: {e}")

    def write(self, row: dict):
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            logger.warning("CSV queue full, dropping row")

    def flush(self):
        self._flush_event.set()

    def close(self):
        self._stopped.set()
        try:
            self.queue.put(None, timeout=1.0)
        except:
            pass
        self.thread.join(timeout=3.0)
        self.file.flush()
        self.file.close()
        logger.info(f"CSV Writer closed: {self.filepath}")


# =============================================================================
# DETECTORS
# =============================================================================

@dataclass
class ArmPose:
    person_id: int
    points: Dict[Tuple[str, str], Tuple[float, float]]
    timestamp: float = 0.0

    def get_chain(self, side: str) -> List[Tuple[float, float]]:
        return [self.points[(side, joint)] for joint in ("shoulder", "elbow", "wrist") if (side, joint) in self.points]


# =============================================================================
# FEATURE 5: MULTIPROCESSING FOR YOLO
# =============================================================================

class YOLOProcessWorker:
    """YOLO inference in separate process."""

    @staticmethod
    def worker_loop(config: dict, input_q, output_q):
        """Runs in separate process."""
        try:
            from ultralytics import YOLO
            import torch

            device = config.get("device", "auto")
            device = "cuda" if torch.cuda.is_available() and device in ("auto", "cuda") else "cpu"

            model = YOLO(config["path"])
            if device == "cuda":
                model.to(device)
                half = True
                model.half()
            else:
                half = False

            logger.info(f"YOLO process started on {device}")

            while True:
                try:
                    item = input_q.get(timeout=1.0)
                    if item is None:
                        break
                    frame, timestamp = item
                    results = model.predict(
                        frame,
                        device=device,
                        imgsz=config["imgsz"],
                        verbose=False,
                        half=half,
                        conf=config["conf"]
                    )
                    output_q.put((results, timestamp), timeout=2.0)
                except queue.Empty:
                    continue
                except Exception as e:
                    logger.error(f"YOLO process error: {e}")
                    output_q.put((None, 0))
        except Exception as e:
            logger.error(f"YOLO process init failed: {e}")


class YOLODetector:
    """YOLO detector with optional multiprocessing."""

    def __init__(self, model_path: str, device: str = "auto", imgsz: int = 480,
                 conf: float = 0.5, skip_frames: int = 2, use_multiprocessing: bool = False):
        self.use_mp = use_multiprocessing
        self.skip_frames = max(1, skip_frames)
        self._cached_poses: List[ArmPose] = []
        self._cache_lock = threading.Lock()
        self._frame_counter = 0
        self._counter_lock = threading.Lock()
        self.stats = TimingStats()
        self._model_path = model_path
        self._device = device
        self._imgsz = imgsz
        self._conf = conf

        try:
            import torch
            from ultralytics import YOLO
        except ImportError:
            logger.error("ultralytics not installed. YOLO disabled.")
            self.model = None
            return

        if self.use_mp:
            from multiprocessing import Process, Queue as MPQueue
            self._input_q = MPQueue(maxsize=2)
            self._output_q = MPQueue(maxsize=2)
            config = {
                "path": model_path,
                "device": device,
                "imgsz": imgsz,
                "conf": conf
            }
            self._proc = Process(
                target=YOLOProcessWorker.worker_loop,
                args=(config, self._input_q, self._output_q),
                daemon=True
            )
            self._proc.start()
            self.model = "multiprocess"
            logger.info("YOLO multiprocessing mode enabled")
        else:
            self._init_single_process()

    def _init_single_process(self):
        try:
            from ultralytics import YOLO
            import torch

            if not os.path.exists(self._model_path):
                logger.warning(f"Model not found: {self._model_path}, attempting download...")

            self.model = YOLO(self._model_path)
            self.device = "cuda" if torch.cuda.is_available() and self._device in ("auto", "cuda") else "cpu"
            self.imgsz = self._imgsz
            self.conf = self._conf

            if self.device == "cuda":
                self.model.to(self.device)
                self.half = True
                self.model.half()
            else:
                self.half = False

            logger.info(f"YOLO initialized on {self.device}")
        except Exception as e:
            logger.error(f"YOLO init failed: {e}")
            self.model = None

    def detect(self, frame: np.ndarray, timestamp: float = 0.0) -> List[ArmPose]:
        if self.model is None:
            return []

        with self._counter_lock:
            self._frame_counter += 1
            should_process = (self._frame_counter % self.skip_frames == 0)

        if not should_process:
            with self._cache_lock:
                return list(self._cached_poses)

        if self.use_mp:
            return self._detect_multiprocess(frame, timestamp)
        else:
            return self._detect_single(frame, timestamp)

    def _detect_single(self, frame: np.ndarray, timestamp: float) -> List[ArmPose]:
        with StageTimer(self.stats):
            try:
                results = self.model.predict(
                    frame, device=self.device, imgsz=self.imgsz,
                    verbose=False, half=self.half, conf=self.conf
                )
            except Exception as e:
                logger.error(f"YOLO prediction error: {e}")
                with self._cache_lock:
                    return list(self._cached_poses)

        poses = self._parse_results(results[0] if results else None, timestamp)
        with self._cache_lock:
            self._cached_poses = poses
        return poses

    def _detect_multiprocess(self, frame: np.ndarray, timestamp: float) -> List[ArmPose]:
        try:
            if self._input_q.full():
                try:
                    self._input_q.get_nowait()
                except:
                    pass
            self._input_q.put_nowait((frame, timestamp))

            results, ts = self._output_q.get(timeout=0.5)
            if results is None:
                with self._cache_lock:
                    return list(self._cached_poses)

            poses = self._parse_results(results[0] if results else None, timestamp)
            with self._cache_lock:
                self._cached_poses = poses
            return poses
        except queue.Empty:
            with self._cache_lock:
                return list(self._cached_poses)
        except Exception as e:
            logger.error(f"Multiprocess YOLO error: {e}")
            with self._cache_lock:
                return list(self._cached_poses)

    def _parse_results(self, result, timestamp: float = 0.0):
        if result is None or not hasattr(result, 'keypoints') or result.keypoints is None:
            return []
        poses = []
        try:
            kp = result.keypoints.xy.cpu().numpy()
            for pid, person in enumerate(kp):
                pts = {}
                try:
                    pts[("left", "shoulder")] = tuple(person[5])
                    pts[("right", "shoulder")] = tuple(person[6])
                    pts[("left", "elbow")] = tuple(person[7])
                    pts[("right", "elbow")] = tuple(person[8])
                    pts[("left", "wrist")] = tuple(person[9])
                    pts[("right", "wrist")] = tuple(person[10])
                    poses.append(ArmPose(pid, pts, timestamp))
                except Exception:
                    pass
        except Exception:
            pass
        return poses

    def get_stats(self) -> str:
        return f"YOLO avg: {self.stats.report()['avg']:.1f}ms"

    def close(self):
        if self.use_mp and hasattr(self, '_proc'):
            try:
                self._input_q.put(None, timeout=1.0)
                self._proc.join(timeout=3.0)
                if self._proc.is_alive():
                    self._proc.terminate()
            except:
                pass


class MediaPipeHandDetector:
    """MediaPipe hand detector with safe model download."""

    HAND_MODEL_URLS = [
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task",
    ]

    def __init__(self, max_hands: int = 2, min_conf: float = 0.5):
        self.max_hands = max_hands
        self.min_conf = min_conf
        self._model_path = str(MODELS_DIR / self.HAND_MODEL_URLS[0].split('/')[-1])
        self._download_model()
        self._result_lock = threading.Lock()
        self._latest_result = None
        self._latest_timestamp = 0
        self.stats = TimingStats()
        self._init_landmarker()

    def _download_model(self):
        if os.path.exists(self._model_path):
            logger.info(f"MediaPipe model found: {self._model_path}")
            return
        os.makedirs(os.path.dirname(self._model_path) or ".", exist_ok=True)
        logger.info("Downloading MediaPipe hand model...")
        try:
            urllib.request.urlretrieve(self.HAND_MODEL_URLS[0], self._model_path)
            logger.info(f"Model downloaded: {self._model_path}")
        except urllib.error.URLError as e:
            logger.error(f"Failed to download model: {e}")
            raise RuntimeError(f"Cannot download MediaPipe model: {e}")
        except Exception as e:
            logger.error(f"Unexpected download error: {e}")
            raise

    def _init_landmarker(self):
        import mediapipe as mp
        from mediapipe.tasks.python.vision import HandLandmarkerOptions, HandLandmarker, RunningMode
        BaseOptions = mp.tasks.BaseOptions

        def result_callback(result, output_image, timestamp_ms: int):
            with self._result_lock:
                self._latest_result = result
                self._latest_timestamp = timestamp_ms

        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=self._model_path),
            running_mode=RunningMode.LIVE_STREAM,
            num_hands=self.max_hands,
            min_hand_detection_confidence=self.min_conf,
            min_hand_presence_confidence=self.min_conf,
            min_tracking_confidence=self.min_conf,
            result_callback=result_callback,
        )
        self.landmarker = HandLandmarker.create_from_options(options)
        logger.info("MediaPipe HandLandmarker initialized")

    def detect_async(self, frame: np.ndarray, timestamp_ms: int):
        import mediapipe as mp
        with StageTimer(self.stats):
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            self.landmarker.detect_async(mp_image, timestamp_ms)

    def get_latest_result(self):
        with self._result_lock:
            return self._latest_result, self._latest_timestamp

    def close(self):
        if hasattr(self, 'landmarker'):
            self.landmarker.close()

    def get_stats(self) -> str:
        return f"MP avg: {self.stats.report()['avg']:.1f}ms"


# =============================================================================
# BACKGROUND ANALYSIS (FFT)
# =============================================================================

class BackgroundAnalysis(threading.Thread):
    """Background FFT analysis with Hann window."""

    FFT_WINDOW = 128

    def __init__(self, analytics):
        super().__init__(daemon=True, name="BackgroundAnalysis")
        self.analytics = analytics
        self._stopped = threading.Event()
        self.fft_results = {}
        self._result_lock = threading.Lock()
        self._window = hann(self.FFT_WINDOW)

    def run(self):
        while not self._stopped.is_set():
            time.sleep(3.0)
            try:
                self.perform_fft_analysis()
            except Exception as e:
                logger.error(f"Background FFT Error: {e}")

    def perform_fft_analysis(self):
        for label, vbuf in self.analytics._velocity_buffers.items():
            if vbuf.size < self.FFT_WINDOW:
                continue
            data = vbuf.get_latest(self.FFT_WINDOW)
            if len(data) < self.FFT_WINDOW:
                continue
            windowed = data * self._window
            N = len(windowed)
            yf = fft(windowed)
            xf = fftfreq(N, 1.0/30.0)[:N//2]
            magnitude = 2.0/N * np.abs(yf[0:N//2])
            if len(magnitude) > 1:
                dominant_idx = np.argmax(magnitude[1:]) + 1
                dominant_freq = xf[dominant_idx]
            else:
                dominant_freq = 0.0
            with self._result_lock:
                self.fft_results[label] = {
                    "dominant_freq": float(dominant_freq),
                    "max_magnitude": float(np.max(magnitude)),
                    "timestamp": time.time()
                }

    def get_results(self):
        with self._result_lock:
            return dict(self.fft_results)

    def stop(self):
        self._stopped.set()


# =============================================================================
# ANALYTICS
# =============================================================================

class HandAnalytics:
    """Hand analytics with per-hand confidence tracking."""

    RING_BUFFER_SIZE = 300
    VELOCITY_WINDOW = 5

    def __init__(self, buffer_size: int = RING_BUFFER_SIZE):
        self.buffer_size = buffer_size
        self._hand_buffers: Dict[str, MultiChannelRingBuffer] = {}
        self._velocity_buffers: Dict[str, NumpyRingBuffer] = {}
        self._jitter_buffers: Dict[str, NumpyRingBuffer] = {}
        self._hand_lock = threading.Lock()
        self._conf_buffers: Dict[str, NumpyRingBuffer] = {}
        self._filters: Dict[str, OneEuroFilter] = {}
        self._kalman_filters: Dict[str, KalmanFilter1D] = {}
        self._frame_count = 0
        self._detection_count = 0
        self._stats_lock = threading.Lock()
        self.bg_analysis: Optional[BackgroundAnalysis] = None

    def update(self, hand_result, timestamp: float, frame_shape: Tuple[int, int],
               calibration: Optional[CameraCalibration] = None) -> Dict[str, dict]:
        enriched_data = {}
        with self._stats_lock:
            self._frame_count += 1

        focal_length = calibration.get_focal_length() if calibration else 1000.0

        if hand_result is None or not hand_result.hand_landmarks:
            with self._hand_lock:
                for label in self._conf_buffers:
                    self._conf_buffers[label].append(0.0)
            return enriched_data

        self._detection_count += 1
        H, W = frame_shape[:2]
        detected_labels = set()

        for hidx, landmarks in enumerate(hand_result.hand_landmarks):
            label = hand_result.handedness[hidx][0].category_name if hand_result.handedness else f"Hand_{hidx}"
            conf = hand_result.handedness[hidx][0].score if hand_result.handedness else 0.0
            detected_labels.add(label)

            if len(landmarks) == 0:
                continue

            with self._hand_lock:
                if label not in self._hand_buffers:
                    self._hand_buffers[label] = MultiChannelRingBuffer(self.buffer_size, channels=5)
                    self._velocity_buffers[label] = NumpyRingBuffer(self.buffer_size)
                    self._jitter_buffers[label] = NumpyRingBuffer(self.buffer_size)
                    self._conf_buffers[label] = NumpyRingBuffer(self.buffer_size)

            lm0 = landmarks[0]
            raw_coords = np.array([lm0.x * W, lm0.y * H, lm0.z])
            if label not in self._filters:
                self._filters[label] = OneEuroFilter(timestamp, raw_coords)
            filtered_coords = self._filters[label](timestamp, raw_coords)

            raw_z_mm = RobotVisionMath.estimate_raw_depth(landmarks, W, H, focal_length)
            if label not in self._kalman_filters:
                self._kalman_filters[label] = KalmanFilter1D()
            smooth_z_mm, velocity_z_mms = self._kalman_filters[label].update(timestamp, raw_z_mm)

            buf = self._hand_buffers[label]
            buf.append(np.array([filtered_coords[0], filtered_coords[1], filtered_coords[2], conf, timestamp]))

            if buf.size >= 2:
                latest = buf.get_latest(min(self.VELOCITY_WINDOW + 1, buf.size))
                dt = np.diff(latest[:, 4])
                dt[dt <= 0] = 1e-6
                speeds = np.sqrt(np.diff(latest[:, 0])**2 + np.diff(latest[:, 1])**2) / dt
                self._velocity_buffers[label].append(float(np.mean(speeds)) if len(speeds) > 0 else 0.0)

            x_mm, y_mm = RobotVisionMath.pixel_to_metric(
                filtered_coords[0], filtered_coords[1], smooth_z_mm, W, H, focal_length
            )

            ttc = float('inf')
            if velocity_z_mms < -50.0:
                ttc = abs(smooth_z_mm / velocity_z_mms)

            enriched_data[label] = {
                "filtered_x_px": filtered_coords[0],
                "filtered_y_px": filtered_coords[1],
                "z_mm": smooth_z_mm,
                "x_mm": x_mm,
                "y_mm": y_mm,
                "velocity_z_mms": velocity_z_mms,
                "ttc_sec": ttc,
                "speed_px_s": self.get_current_speed(label),
                "gesture": RobotVisionMath.detect_gesture(landmarks),
                "confidence": conf
            }
            enriched_data[label].update(
                RobotVisionMath.compute_hand_metrics(landmarks, W, H, smooth_z_mm, focal_length)
            )

            with self._hand_lock:
                self._conf_buffers[label].append(conf)

        with self._hand_lock:
            for label in self._conf_buffers:
                if label not in detected_labels:
                    self._conf_buffers[label].append(0.0)

        return enriched_data

    def get_current_speed(self, hand_label: str) -> float:
        if hand_label not in self._velocity_buffers:
            return 0.0
        v = self._velocity_buffers[hand_label].get_latest(1)
        return float(v[0]) if len(v) > 0 else 0.0

    def get_confidence_data(self, label: str, n: int = 60) -> np.ndarray:
        with self._hand_lock:
            if label not in self._conf_buffers:
                return np.array([])
            return self._conf_buffers[label].get_latest(n)

    def get_depth_variance(self, label: str, n: int = 20) -> float:
        if label not in self._hand_buffers:
            return 0.0
        data = self._hand_buffers[label].get_latest(n)
        if len(data) < 2:
            return 0.0
        return float(np.var(data[:, 2]))

    def reset(self):
        with self._hand_lock:
            for b in self._hand_buffers.values():
                b.clear()
            for b in self._velocity_buffers.values():
                b.clear()
            for b in self._jitter_buffers.values():
                b.clear()
            for b in self._conf_buffers.values():
                b.clear()
        self._filters.clear()
        self._kalman_filters.clear()
        logger.info("All analytics buffers reset")


# =============================================================================
# RENDERER
# =============================================================================

class AnalyticsRenderer:
    def __init__(self, analytics: HandAnalytics):
        self.analytics = analytics
        self.stats = TimingStats()
        self.show_yolo = True
        self.show_mp = True
        self.show_envelope = True
        self.show_hud = True

    def draw_sparkline(self, frame, data: np.ndarray, x: int, y: int, w: int, h: int, color=(0, 255, 0)):
        if len(data) < 2:
            return
        d_min, d_max = np.min(data), np.max(data)
        d_range = (d_max - d_min) if (d_max - d_min) > 0 else 1.0
        pts = []
        for i, val in enumerate(data):
            px = x + int(i * w / (len(data) - 1))
            py = y + h - int((val - d_min) * h / d_range)
            pts.append((px, py))
        for i in range(len(pts) - 1):
            cv2.line(frame, pts[i], pts[i+1], color, 1)

    def render(self, frame: np.ndarray, poses: List[ArmPose], hand_result, enriched_data: dict,
               fps: float, safety_state: str = "RUN", anomaly_data: Dict[str, Any] = None,
               predictions: Dict[str, Tuple[float, float, float]] = None,
               frame_timestamp: float = 0.0) -> np.ndarray:
        with StageTimer(self.stats):
            H, W = frame.shape[:2]
            anomaly_data = anomaly_data or {}
            predictions = predictions or {}

            safety_colors = {"RUN": (0, 255, 0), "SLOW": (0, 200, 255), "STOP": (0, 0, 255), "WARNING": (0, 165, 255)}
            safety_color = safety_colors.get(safety_state, (128, 128, 128))

            # YOLO skeleton
            if self.show_yolo and poses:
                for pose in poses:
                    if frame_timestamp > 0 and pose.timestamp > 0 and abs(frame_timestamp - pose.timestamp) > 0.1:
                        continue
                    for side in ("left", "right"):
                        chain = pose.get_chain(side)
                        for i in range(len(chain) - 1):
                            p1 = tuple(map(int, chain[i]))
                            p2 = tuple(map(int, chain[i+1]))
                            cv2.line(frame, p1, p2, {"left": (0, 200, 255), "right": (255, 120, 0)}[side], 2)
                        for pt in chain:
                            cv2.circle(frame, tuple(map(int, pt)), 5, {"left": (0, 200, 255), "right": (255, 120, 0)}[side], -1)

            # MediaPipe hands
            if hand_result and hand_result.hand_landmarks and self.show_mp:
                for idx, landmarks in enumerate(hand_result.hand_landmarks):
                    label = hand_result.handedness[idx][0].category_name if hand_result.handedness else "Unknown"
                    pix = [(int(lm.x * W), int(lm.y * H)) for lm in landmarks]
                    data = enriched_data.get(label, {})

                    for a, b in HAND_CONNECTIONS:
                        if a < len(pix) and b < len(pix):
                            cv2.line(frame, pix[a], pix[b], (220, 220, 220), 1)
                    for x, y in pix:
                        cv2.circle(frame, (x, y), 2, (0, 0, 255), -1)

                    # Predicted position visualization
                    if label in predictions:
                        px_pred, py_pred, pz_pred = predictions[label]
                        px_pred_px = int((px_pred / data.get("z_mm", 1000)) * 1000 + W/2)
                        py_pred_px = int((py_pred / data.get("z_mm", 1000)) * 1000 + H/2)
                        cv2.circle(frame, (px_pred_px, py_pred_px), 8, (255, 0, 255), 2)
                        cv2.putText(frame, "PRED", (px_pred_px + 10, py_pred_px), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

                    # Center of mass + pointing vector (for ROS2 / KUKA)
                    if "centroid_px" in data:
                        cmx, cmy = int(data["centroid_px"][0]), int(data["centroid_px"][1])
                        ex = int(cmx + data.get("dir_x", 0.0) * 90)
                        ey = int(cmy + data.get("dir_y", 0.0) * 90)
                        cv2.arrowedLine(frame, (cmx, cmy), (ex, ey), (0, 255, 255), 2, tipLength=0.3)
                        cv2.circle(frame, (cmx, cmy), 6, (0, 255, 255), -1)
                        cv2.circle(frame, (cmx, cmy), 9, (0, 0, 0), 1)
                        cv2.putText(frame,
                            f"COM {data.get('centroid_x_mm',0):.0f},{data.get('centroid_y_mm',0):.0f},{data.get('centroid_z_mm',0):.0f}mm",
                            (cmx + 12, cmy + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                        cv2.putText(frame,
                            f"Area {data.get('area_mm2',0)/100.0:.0f}cm2  Ang {data.get('angle_deg',0):.0f}",
                            (cmx + 12, cmy + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                    if self.show_envelope:
                        arr = np.array(pix, np.int32).reshape((-1, 1, 2))
                        speed = data.get("speed_px_s", 0.0)
                        safety = RobotVisionMath.create_safety_envelope(arr, speed)
                        cv2.polylines(frame, [safety], True, (0, 255, 255), 2)
                        overlay = frame.copy()
                        cv2.fillPoly(overlay, [safety], (0, 64, 255))
                        cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)

                        bx, by, bw, bh = cv2.boundingRect(arr)
                        text_y = max(100, by - 20)
                        cv2.rectangle(frame, (bx, text_y - 100), (bx + 280, text_y), (30, 30, 30), -1)
                        cv2.rectangle(frame, (bx, text_y - 100), (bx + 280, text_y), safety_color, 1)

                        z_mm = data.get("z_mm", 0)
                        x_mm = data.get("x_mm", 0)
                        y_mm = data.get("y_mm", 0)
                        ttc = data.get("ttc_sec", float('inf'))
                        gesture = data.get("gesture", "")
                        conf = data.get("confidence", 0.0)
                        is_anomaly = anomaly_data.get(label, {}).get("is_anomaly", False)

                        cv2.putText(frame, f"{label} | {gesture}", (bx + 8, text_y - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                        cv2.putText(frame, f"3D: {x_mm:.0f} {y_mm:.0f} {z_mm:.0f}mm", (bx + 8, text_y - 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                        cv2.putText(frame, f"Conf: {conf:.2f}", (bx + 8, text_y - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

                        if is_anomaly:
                            cv2.putText(frame, "ANOMALY!", (bx + 8, text_y - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                        if ttc < 0.6:
                            cv2.putText(frame, f"TTC: {ttc:.2f}s COLLISION!", (bx + 8, text_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                            if int(time.time() * 10) % 2 == 0:
                                cv2.rectangle(frame, (0, 0), (W, H), (0, 0, 255), 10)
                        else:
                            cv2.putText(frame, f"TTC: Safe", (bx + 8, text_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

            # Safety state banner
            cv2.rectangle(frame, (0, H - 40), (W, H), safety_color, -1)
            cv2.putText(frame, f"SAFETY: {safety_state}", (20, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # HUD
            if self.show_hud:
                cv2.rectangle(frame, (0, 0), (320, 300), (25, 25, 25), -1)
                cv2.putText(frame, f"FPS: {fps:.1f}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                with self.analytics._hand_lock:
                    labels = list(self.analytics._conf_buffers.keys())
                if labels:
                    conf_data = self.analytics.get_confidence_data(labels[0], 60)
                    if len(conf_data) > 1:
                        self.draw_sparkline(frame, conf_data, 15, 45, 280, 45, (0, 200, 255))
                        cv2.putText(frame, f"Conf ({labels[0]}): {conf_data[-1]:.2f}", (15, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                fft_res = self.analytics.bg_analysis.get_results() if hasattr(self.analytics, 'bg_analysis') and self.analytics.bg_analysis else {}
                y_offset = 130
                for label, fft_data in list(fft_res.items())[:2]:
                    dom_freq = fft_data.get("dominant_freq", 0)
                    cv2.putText(frame, f"FFT {label}: {dom_freq:.1f}Hz", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)
                    y_offset += 20

                for label, adata in list(anomaly_data.items())[:2]:
                    score = adata.get("score", 0)
                    cv2.putText(frame, f"Anom {label}: {score:.3f}", (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
                    y_offset += 20

            return frame


# =============================================================================
# FEATURE 7: WEB DASHBOARD (FastAPI + WebSocket)
# =============================================================================

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False
    logger.warning("fastapi not installed, web dashboard disabled")


if HAS_FASTAPI:
    WEB_DASHBOARD_HTML = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Vision Dashboard</title>
        <style>
            body { font-family: monospace; background: #111; color: #0f0; margin: 0; padding: 20px; }
            .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
            .panel { background: #222; border: 1px solid #333; padding: 15px; border-radius: 8px; }
            .panel h3 { margin-top: 0; color: #0ff; }
            .metric { display: flex; justify-content: space-between; padding: 5px 0; border-bottom: 1px solid #333; }
            .metric span:first-child { color: #aaa; }
            .metric span:last-child { color: #0f0; font-weight: bold; }
            .alert { color: #f00; animation: blink 1s infinite; }
            @keyframes blink { 0%, 50% { opacity: 1; } 51%, 100% { opacity: 0.3; } }
            #video-feed { width: 100%; border: 2px solid #0f0; }
            canvas { width: 100%; height: 200px; background: #000; border: 1px solid #333; }
        </style>
    </head>
    <body>
        <h1>Industrial Robot Vision Dashboard</h1>
        <div class="grid">
            <div class="panel">
                <h3>Live Feed</h3>
                <img id="video-feed" src="/video_feed" alt="Camera Feed">
            </div>
            <div class="panel">
                <h3>Safety Status</h3>
                <div id="safety-state" class="metric"><span>State:</span><span>RUN</span></div>
                <div id="min-ttc" class="metric"><span>Min TTC:</span><span>--</span></div>
                <div id="active-hands" class="metric"><span>Active Hands:</span><span>0</span></div>
                <div id="fps" class="metric"><span>FPS:</span><span>0</span></div>
            </div>
            <div class="panel">
                <h3>Telemetry</h3>
                <div id="telemetry"></div>
            </div>
            <div class="panel">
                <h3>FFT Analysis</h3>
                <canvas id="fft-chart"></canvas>
            </div>
        </div>
        <script>
            const ws = new WebSocket(`ws://${window.location.host}/ws`);
            ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                document.getElementById('safety-state').innerHTML = 
                    `<span>State:</span><span class="${data.safety_state === 'STOP' ? 'alert' : ''}">${data.safety_state}</span>`;
                document.getElementById('min-ttc').innerHTML = `<span>Min TTC:</span><span>${data.min_ttc?.toFixed(2) || '--'}s</span>`;
                document.getElementById('active-hands').innerHTML = `<span>Active Hands:</span><span>${Object.keys(data.hands || {}).length}</span>`;
                document.getElementById('fps').innerHTML = `<span>FPS:</span><span>${data.fps?.toFixed(1) || 0}</span>`;

                let telemHTML = '';
                for (const [hand, info] of Object.entries(data.hands || {})) {
                    telemHTML += `<div class="metric"><span>${hand}:</span><span>${info.gesture} | ${info.z_mm?.toFixed(0)}mm | ${info.speed_px_s?.toFixed(0)}px/s</span></div>`;
                }
                document.getElementById('telemetry').innerHTML = telemHTML;
            };
            ws.onclose = () => { document.getElementById('safety-state').innerHTML = '<span>State:</span><span class="alert">DISCONNECTED</span>'; };
        </script>
    </body>
    </html>
    """


class WebDashboard:
    """FastAPI web dashboard with WebSocket streaming."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self.host = host
        self.port = port
        self.app: Optional[Any] = None
        self._clients: List[Any] = []
        self._latest_data: dict = {}
        self._data_lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        if HAS_FASTAPI:
            self._setup_app()

    def _setup_app(self):
        self.app = FastAPI(title="Robot Vision Dashboard")

        @self.app.get("/", response_class=HTMLResponse)
        async def root():
            return WEB_DASHBOARD_HTML

        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self._clients.append(websocket)
            try:
                while True:
                    with self._data_lock:
                        data = dict(self._latest_data)
                    await websocket.send_json(data)
                    await asyncio.sleep(0.1)
            except WebSocketDisconnect:
                if websocket in self._clients:
                    self._clients.remove(websocket)
            except Exception:
                if websocket in self._clients:
                    self._clients.remove(websocket)

        @self.app.get("/health")
        async def health():
            return {"status": "ok", "timestamp": time.time()}

    def update_data(self, data: dict):
        with self._data_lock:
            self._latest_data = data

    def start(self):
        if not HAS_FASTAPI or self.app is None:
            logger.warning("Web dashboard disabled (fastapi not installed)")
            return

        self._running = True
        self._thread = threading.Thread(target=self._run_server, daemon=True, name="WebDashboard")
        self._thread.start()
        logger.info(f"Web dashboard starting on http://{self.host}:{self.port}")

    def _run_server(self):
        import uvicorn
        uvicorn.run(self.app, host=self.host, port=self.port, log_level="warning")

    def stop(self):
        self._running = False


# =============================================================================
# MAIN APPLICATION
# =============================================================================

class HandTrackerApp:
    def __init__(self, args):
        self.args = args
        self.running = True
        self.fps_stats = TimingStats()
        self.cap = None
        self.camera = None
        self.yolo = None
        self.mp_detector = None
        self.renderer = None
        self.analytics = None
        self.bg_analysis = None
        self.csv_writer = None
        self.telemetry = None
        self.safety_controller = None
        self.trajectory_predictor = None
        self.anomaly_detector = None
        self.blackbox = None
        self.web_dashboard = None
        self.calibration = None
        self._yolo_poses: List[ArmPose] = []
        self._yolo_lock = threading.Lock()
        self._current_frame = None
        self._frame_lock = threading.Lock()
        self._frame_timestamp = 0.0
        self._last_telemetry_time = 0.0
        self._telemetry_interval = 0.1

        self.config = load_config(getattr(args, 'config', None))
        if HAS_PYDANTIC and isinstance(self.config, AppConfig):
            self._apply_pydantic_config()
        else:
            self._apply_dict_config()

    def _apply_pydantic_config(self):
        cfg = self.config
        self.snapshot_dir = cfg.snapshot_dir
        self.no_show = cfg.no_show
        self.camera_config = cfg.camera
        self.safety_config = cfg.safety
        self.yolo_config = cfg.models.get("yolo", {})
        self.mp_config = cfg.models.get("mediapipe", {})
        self.telemetry_config = cfg.telemetry
        self.web_config = cfg.web_dashboard
        self.anomaly_config = cfg.anomaly

        setup_logging(cfg.log_level)

        os.makedirs(self.snapshot_dir, exist_ok=True)
        os.makedirs(self.telemetry_config.csv_dir, exist_ok=True)
        if self.telemetry_config.enable_blackbox:
            os.makedirs(self.telemetry_config.blackbox_dir, exist_ok=True)

    def _apply_dict_config(self):
        self.snapshot_dir = getattr(self.args, 'snapshot_dir', "./snapshots")
        self.no_show = getattr(self.args, 'no_show', False)
        self.camera_config = {"id": "0", "mirror": True}
        self.safety_config = {"emergency_zone_mm": 200, "warning_zone_mm": 500}
        self.yolo_config = {}
        self.telemetry_config = {"udp_target": "127.0.0.1:9090", "csv_dir": "./telemetry"}
        self.web_config = {"enabled": False}
        self.anomaly_config = {"enabled": False}
        setup_logging("INFO")
        os.makedirs(self.snapshot_dir, exist_ok=True)

    def _init_camera(self):
        cam_id = self.camera_config.id if hasattr(self.camera_config, 'id') else self.camera_config.get("id", "0")
        self.cap = open_camera(str(cam_id))
        self.camera = ThreadedCamera(self.cap)
        logger.info("Camera initialized")

    def _init_calibration(self):
        calib_file = self.camera_config.calibration_file if hasattr(self.camera_config, 'calibration_file') else None
        if calib_file:
            self.calibration = CameraCalibration()
            if self.calibration.load(calib_file):
                logger.info("Camera calibration loaded")
            else:
                logger.warning("Using default focal length (1000px)")
        else:
            self.calibration = None

    def _init_yolo(self):
        if getattr(self.args, 'no_arm', False):
            return
        try:
            yolo_cfg = self.yolo_config
            if hasattr(yolo_cfg, 'model_dump'):
                yolo_cfg = yolo_cfg.model_dump()
            elif hasattr(yolo_cfg, 'dict'):
                yolo_cfg = yolo_cfg.dict()

            model_path = yolo_cfg.get('path', 'models/yolo11n-pose.pt')
            if not os.path.isabs(model_path):
                model_path = str(PROJECT_ROOT / model_path)

            self.yolo = YOLODetector(
                model_path=model_path,
                device=yolo_cfg.get('device', 'auto'),
                imgsz=yolo_cfg.get('imgsz', 480),
                conf=yolo_cfg.get('conf', 0.5),
                skip_frames=yolo_cfg.get('skip_frames', 2),
                use_multiprocessing=yolo_cfg.get('use_multiprocessing', False)
            )
            logger.info("YOLO initialized")
        except Exception as e:
            logger.error(f"YOLO initialization failed: {e}")
            self.yolo = None

    def _init_mediapipe(self):
        if getattr(self.args, 'no_hands', False):
            return
        try:
            mp_cfg = self.mp_config
            if hasattr(mp_cfg, 'model_dump'):
                mp_cfg = mp_cfg.model_dump()
            elif hasattr(mp_cfg, 'dict'):
                mp_cfg = mp_cfg.dict()

            self.mp_detector = MediaPipeHandDetector(
                max_hands=mp_cfg.get('max_hands', 2),
                min_conf=mp_cfg.get('min_conf', 0.5)
            )
            logger.info("MediaPipe initialized")
        except Exception as e:
            logger.error(f"MediaPipe initialization failed: {e}")
            self.mp_detector = None

    def _init_analytics(self):
        self.analytics = HandAnalytics()
        self.bg_analysis = BackgroundAnalysis(self.analytics)
        self.analytics.bg_analysis = self.bg_analysis
        self.bg_analysis.start()

        csv_path = os.path.join(
            self.telemetry_config.csv_dir if hasattr(self.telemetry_config, 'csv_dir') else self.telemetry_config.get('csv_dir', './telemetry'),
            "hand_telemetry.csv"
        )
        self.csv_writer = AsyncCSVWriter(csv_path, [
            "timestamp", "hand", "x_mm", "y_mm", "z_mm",
            "centroid_x_mm", "centroid_y_mm", "centroid_z_mm",
            "area_mm2", "volume_mm3", "dir_x", "dir_y", "dir_z", "angle_deg",
            "speed_px_s", "velocity_z_mms", "ttc_sec", "gesture", "confidence",
            "safety_state", "anomaly_score"
        ])
        logger.info("Analytics & Background FFT started")

    def _init_safety(self):
        sc = self.safety_config
        self.safety_controller = SafetyController(
            plc_ip=sc.plc_ip if hasattr(sc, 'plc_ip') else sc.get('plc_ip'),
            plc_port=sc.plc_port if hasattr(sc, 'plc_port') else sc.get('plc_port', 502),
            emergency_zone_mm=sc.emergency_zone_mm if hasattr(sc, 'emergency_zone_mm') else sc.get('emergency_zone_mm', 200),
            warning_zone_mm=sc.warning_zone_mm if hasattr(sc, 'warning_zone_mm') else sc.get('warning_zone_mm', 500),
            enable_hardware=sc.enable_hardware_estop if hasattr(sc, 'enable_hardware_estop') else sc.get('enable_hardware_estop', False)
        )
        self.trajectory_predictor = TrajectoryPredictor(
            horizon_ms=sc.predictive_horizon_ms if hasattr(sc, 'predictive_horizon_ms') else sc.get('predictive_horizon_ms', 500)
        )
        logger.info("Safety controller initialized")

    def _init_anomaly(self):
        cli_enabled = getattr(self.args, 'enable_anomaly', False)
        ac = self.anomaly_config
        config_enabled = ac.enabled if hasattr(ac, 'enabled') else ac.get('enabled', False)
        enabled = cli_enabled or config_enabled
        if enabled:
            self.anomaly_detector = AnomalyDetector(
                contamination=ac.contamination if hasattr(ac, 'contamination') else ac.get('contamination', 0.05),
                window_size=ac.window_size if hasattr(ac, 'window_size') else ac.get('window_size', 100)
            )
            logger.info("Anomaly detector initialized")
        else:
            self.anomaly_detector = None

    def _init_blackbox(self):
        cli_enabled = getattr(self.args, 'enable_blackbox', False)
        tc = self.telemetry_config
        config_enabled = tc.enable_blackbox if hasattr(tc, 'enable_blackbox') else tc.get('enable_blackbox', True)
        enabled = cli_enabled or config_enabled
        if enabled:
            self.blackbox = BlackBoxRecorder(
                output_dir=tc.blackbox_dir if hasattr(tc, 'blackbox_dir') else tc.get('blackbox_dir', './blackbox'),
                max_gb=tc.blackbox_max_gb if hasattr(tc, 'blackbox_max_gb') else tc.get('blackbox_max_gb', 10.0)
            )
            self.blackbox.start_session()
            logger.info("BlackBox recorder started")

    def _init_web_dashboard(self):
        # Check CLI arg first, then config
        cli_enabled = getattr(self.args, 'enable_web', False)
        wc = self.web_config
        config_enabled = wc.enabled if hasattr(wc, 'enabled') else wc.get('enabled', False)
        enabled = cli_enabled or config_enabled

        if enabled:
            # CLI port overrides config
            cli_port = getattr(self.args, 'web_port', None)
            port = cli_port if cli_port else (wc.port if hasattr(wc, 'port') else wc.get('port', 8080))
            host = wc.host if hasattr(wc, 'host') else wc.get('host', '0.0.0.0')

            self.web_dashboard = WebDashboard(host=host, port=port)
            self.web_dashboard.start()

    def _init_telemetry(self):
        tc = self.telemetry_config
        target = tc.udp_target if hasattr(tc, 'udp_target') else tc.get('udp_target', '127.0.0.1:9090')
        ip, port = target.split(':')
        self.telemetry = RobotTelemetryStreamer(ip, int(port))
        rate = tc.rate_limit_hz if hasattr(tc, 'rate_limit_hz') else tc.get('rate_limit_hz', 10)
        self._telemetry_interval = 1.0 / rate

    def run(self):
        self._init_camera()
        self._init_calibration()
        self._init_analytics()
        self._init_telemetry()
        self._init_safety()
        self._init_anomaly()
        self._init_blackbox()
        self._init_web_dashboard()

        self.renderer = AnalyticsRenderer(self.analytics)
        self._init_yolo()
        self._init_mediapipe()

        if not self.no_show:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, 1280, 720)

        t0 = time.time()
        last_fps_time = t0
        frame_counter = 0

        try:
            while self.running:
                loop_start = time.perf_counter()
                ok, frame = self.camera.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    continue

                mirror = self.camera_config.mirror if hasattr(self.camera_config, 'mirror') else self.camera_config.get("mirror", True)
                if mirror:
                    frame = cv2.flip(frame, 1)

                if self.calibration:
                    frame = self.calibration.undistort(frame)

                H, W = frame.shape[:2]
                ts = time.time() - t0
                ts_ms = int(ts * 1000)
                self._frame_timestamp = ts

                with self._frame_lock:
                    self._current_frame = frame.copy()

                if self.mp_detector:
                    self.mp_detector.detect_async(frame, ts_ms)

                yolo_poses = self.yolo.detect(frame, timestamp=ts) if self.yolo else []
                mp_result, mp_ts = self.mp_detector.get_latest_result() if self.mp_detector else (None, 0)

                enriched_data = self.analytics.update(mp_result, ts, (H, W), self.calibration)

                predictions = {}
                for label, data in enriched_data.items():
                    self.trajectory_predictor.add_point(
                        label, data["x_mm"], data["y_mm"], data["z_mm"], ts
                    )
                predictions = self.trajectory_predictor.predict_all()

                safety_state = "RUN"
                if self.safety_controller:
                    safety_state = self.safety_controller.evaluate(enriched_data, predictions)

                anomaly_data = {}
                if self.anomaly_detector:
                    for label, data in enriched_data.items():
                        speed = data.get("speed_px_s", 0)
                        accel = 0.0
                        if label in self.analytics._velocity_buffers:
                            vbuf = self.analytics._velocity_buffers[label]
                            if vbuf.size >= 3:
                                v = vbuf.get_latest(3)
                                accel = (v[-1] - v[0]) / 0.1

                        depth_var = self.analytics.get_depth_variance(label)
                        jitter = 0.0

                        self.anomaly_detector.add_observation(speed, accel, jitter, depth_var)
                        is_anom, score = self.anomaly_detector.predict(speed, accel, jitter, depth_var)
                        anomaly_data[label] = {"is_anomaly": is_anom, "score": score}

                    if not self.anomaly_detector._is_trained and self.anomaly_detector._buffer and len(self.anomaly_detector._buffer) >= 50:
                        self.anomaly_detector.fit()

                now = time.time()
                if now - self._last_telemetry_time >= self._telemetry_interval:
                    try:
                        self.telemetry.send_packet({
                            "timestamp": ts,
                            "frame_counter": frame_counter,
                            "hands": enriched_data,
                            "safety_state": safety_state,
                            "predictions": {k: list(v) for k, v in predictions.items()},
                            "anomaly": anomaly_data
                        })
                    except Exception:
                        pass
                    self._last_telemetry_time = now

                frame_counter += 1
                now = time.time()
                if now - last_fps_time >= 1.0:
                    fps = frame_counter / (now - last_fps_time)
                    frame_counter = 0
                    last_fps_time = now
                else:
                    fps = self.fps_stats.report().get("avg", 0.0)
                    fps = (1000.0 / fps) if fps > 0 else 0.0

                for hname, hdata in enriched_data.items():
                    try:
                        adata = anomaly_data.get(hname, {})
                        self.csv_writer.write({
                            "timestamp": ts,
                            "hand": hname,
                            "x_mm": hdata.get("x_mm"),
                            "y_mm": hdata.get("y_mm"),
                            "z_mm": hdata.get("z_mm"),
                            "centroid_x_mm": hdata.get("centroid_x_mm"),
                            "centroid_y_mm": hdata.get("centroid_y_mm"),
                            "centroid_z_mm": hdata.get("centroid_z_mm"),
                            "area_mm2": hdata.get("area_mm2"),
                            "volume_mm3": hdata.get("volume_mm3"),
                            "dir_x": hdata.get("dir_x"),
                            "dir_y": hdata.get("dir_y"),
                            "dir_z": hdata.get("dir_z"),
                            "angle_deg": hdata.get("angle_deg"),
                            "speed_px_s": hdata.get("speed_px_s"),
                            "velocity_z_mms": hdata.get("velocity_z_mms"),
                            "ttc_sec": hdata.get("ttc_sec"),
                            "gesture": hdata.get("gesture"),
                            "confidence": hdata.get("confidence", 0.0),
                            "safety_state": safety_state,
                            "anomaly_score": adata.get("score", 0.0)
                        })
                    except Exception:
                        pass

                if self.blackbox:
                    # Write one row per hand for blackbox CSV
                    for hname, hdata in enriched_data.items():
                        bb_telemetry = {
                            "frame_id": frame_counter,
                            "timestamp": ts,
                            "hand": hname,
                            "x_mm": hdata.get("x_mm"),
                            "y_mm": hdata.get("y_mm"),
                            "z_mm": hdata.get("z_mm"),
                            "speed_px_s": hdata.get("speed_px_s"),
                            "ttc_sec": hdata.get("ttc_sec"),
                            "gesture": hdata.get("gesture"),
                            "safety_state": safety_state,
                            "anomaly_score": anomaly_data.get(hname, {}).get("score", 0.0)
                        }
                        self.blackbox.write_frame(frame, bb_telemetry)
                    # Also write frame without telemetry for video
                    if not enriched_data:
                        self.blackbox.write_frame(frame, None)

                if self.web_dashboard:
                    min_ttc = min((d.get("ttc_sec", float('inf')) for d in enriched_data.values()), default=float('inf'))
                    self.web_dashboard.update_data({
                        "timestamp": ts,
                        "fps": fps,
                        "safety_state": safety_state,
                        "hands": enriched_data,
                        "predictions": {k: list(v) for k, v in predictions.items()},
                        "anomaly": anomaly_data,
                        "min_ttc": min_ttc if min_ttc != float('inf') else None
                    })

                display = self.renderer.render(
                    frame.copy(), yolo_poses, mp_result, enriched_data, fps,
                    safety_state=safety_state, anomaly_data=anomaly_data,
                    predictions=predictions, frame_timestamp=ts
                )

                if not self.no_show:
                    cv2.imshow(WINDOW_NAME, display)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('r'):
                        self.analytics.reset()
                        self.trajectory_predictor.reset()
                        if self.anomaly_detector:
                            self.anomaly_detector.reset()
                    elif key == ord('s'):
                        fname = os.path.join(self.snapshot_dir, f"snapshot_{int(time.time())}.jpg")
                        try:
                            cv2.imwrite(fname, display)
                            logger.info(f"Snapshot saved: {fname}")
                        except Exception as e:
                            logger.error(f"Snapshot error: {e}")
                    elif key == ord('a'):
                        fft_res = self.bg_analysis.get_results()
                        logger.info(f"FFT Results: {json.dumps(fft_res, indent=2)}")
                    elif key == ord('1'):
                        self.renderer.show_hud = not self.renderer.show_hud
                    elif key == ord('2'):
                        self.renderer.show_yolo = not self.renderer.show_yolo
                    elif key == ord('3'):
                        self.renderer.show_mp = not self.renderer.show_mp
                    elif key == ord('4'):
                        self.renderer.show_envelope = not self.renderer.show_envelope

                self.fps_stats.add((time.perf_counter() - loop_start) * 1000)

        finally:
            self.shutdown()

    def shutdown(self):
        self.running = False
        logger.info("Shutting down...")
        if self.bg_analysis:
            self.bg_analysis.stop()
        if self.camera:
            self.camera.release()
        if self.mp_detector:
            self.mp_detector.close()
        if self.yolo:
            self.yolo.close()
        if self.csv_writer:
            self.csv_writer.close()
        if self.safety_controller:
            self.safety_controller.close()
        if self.blackbox:
            self.blackbox.stop()
        if self.web_dashboard:
            self.web_dashboard.stop()
        cv2.destroyAllWindows()
        logger.info("Application terminated cleanly.")


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Industrial Robot Hand Tracker v3.0")
    ap.add_argument("--config", default=None, help="Path to YAML config file")
    ap.add_argument("--camera", default="0", help="Camera index")
    ap.add_argument("--model", default="yolo11n-pose.pt", help="YOLO pose model")
    ap.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    ap.add_argument("--imgsz", type=int, default=480)
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--skip", type=int, default=2)
    ap.add_argument("--max_hands", type=int, default=2)
    ap.add_argument("--no_arm", action="store_true")
    ap.add_argument("--no_hands", action="store_true")
    ap.add_argument("--no_mirror", action="store_true")
    ap.add_argument("--no_show", action="store_true")
    ap.add_argument("--snapshot_dir", default="./snapshots", help="Directory for snapshots")
    ap.add_argument("--csv_dir", default="./telemetry", help="Directory for CSV telemetry")
    ap.add_argument("--enable_web", action="store_true", help="Enable web dashboard")
    ap.add_argument("--web_port", type=int, default=8080, help="Web dashboard port")
    ap.add_argument("--enable_anomaly", action="store_true", help="Enable anomaly detection")
    ap.add_argument("--enable_blackbox", action="store_true", help="Enable black box recording")
    ap.add_argument("--enable_mp_yolo", action="store_true", help="Use multiprocessing for YOLO")
    ap.add_argument("--plc_ip", default=None, help="Safety PLC IP address")
    ap.add_argument("--calibration", default=None, help="Camera calibration .npz file")
    args = ap.parse_args()

    app = HandTrackerApp(args)
    app.run()
