# ================================
# file: code/appio/bluetooth.py
# ================================
from __future__ import annotations
from typing import Tuple, Optional
import json, threading, time
try:
    import serial  # pyserial
except Exception:
    serial = None

from core import Pose2D, LaserScan

class BleInterface:
    """BLE (serial over BT) interface for real-robot I/O.
    This class mirrors RobotSim's method names so callers can swap seamlessly.
    Thread-safety: reader thread fills latest data; main thread polls get_latest.
    """
    def __init__(self, port: Optional[str] = None, baud: int = 115200) -> None:
        self.ser = None
        self._rx_thread = None
        self._running = False
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Pose2D] = None  # absolute odom pose if provided
        self._seq = 0
        if port is not None:
            self.connect(port, baud)

    def connect(self, port: str, baud: int = 115200) -> None:
        if serial is None:
            raise RuntimeError("pyserial not available")
        self.ser = serial.Serial(port, baud, timeout=1)
        self._running = True
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()

    def _reader(self) -> None:
        buf = bytearray()
        while self._running and self.ser and self.ser.is_open:
            ch = self.ser.read(1)
            if not ch:
                continue
            if ch == b"\n":
                line = buf.decode("utf-8", errors="ignore").strip()
                buf.clear()
                if line:
                    self._handle_line(line)
            else:
                buf.extend(ch)

    def _handle_line(self, line: str) -> None:
        try:
            msg = json.loads(line)
        except Exception:
            return
        typ = msg.get("type")
        if typ == "L":
            amin = float(msg.get("angle_min", -3.14159))
            ainc = float(msg.get("angle_inc", 0.01745))
            ranges = msg.get("ranges", [])
            self.latest_scan = LaserScan(amin, ainc, ranges, robot_pose=None, t=msg.get("t"))
        elif typ == "O":
            x = float(msg.get("x", 0.0))
            y = float(msg.get("y", 0.0))
            th = float(msg.get("theta", 0.0))
            self.latest_odom = Pose2D(x, y, th)
        # H (heartbeat) and others ignored here.

    def get_latest_data(self) -> Tuple[Optional[LaserScan], Optional[Pose2D]]:
        return (self.latest_scan, self.latest_odom)

    # --- Methods mirroring RobotSim API ---
    def apply_control(self, v: float, w: float) -> None:
        if not self.ser or not self.ser.is_open:
            return
        self._seq += 1
        msg = {"t": round(time.time(),3), "type":"C", "v": float(v), "w": float(w), "seq": self._seq}
        data = (json.dumps(msg) + "\n").encode("utf-8")
        self.ser.write(data)

    # Real robot provides odom via get_latest_data(); keep API parity
    def get_odometry(self):  # type: ignore[no-redef]
        # Not used; main loop pulls latest_odom instead
        return (0.0, 0.0, 0.0)

    def get_lidar_scan(self):  # type: ignore[no-redef]
        # Not used; main loop pulls latest_scan instead
        return None

    def update(self, dt: float) -> None:
        # Real robot motion is internal; nothing to do here.
        pass

    def disconnect(self) -> None:
        self._running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
