# ================================
# file: code/appio/logger.py
# ================================
from __future__ import annotations
from typing import Sequence, Tuple
import time
import numpy as np
from core import Pose2D, LaserScan

class DataLogger:
    """Simple NPZ logger for scans, odometry, poses, commands and map snapshots."""
    def __init__(self) -> None:
        self.t0 = time.time()
        self.scans = []
        self.odoms = []
        self.poses = []
        self.cmds = []
        self.maps = []

    def log_scan(self, scan: LaserScan) -> None:
        self.scans.append((scan.t or (time.time()-self.t0), list(scan.ranges)))

    def log_odom(self, x: float, y: float, theta: float) -> None:
        self.odoms.append((time.time()-self.t0, float(x), float(y), float(theta)))

    def log_pose(self, pose: Pose2D) -> None:
        self.poses.append((time.time()-self.t0, pose.x, pose.y, pose.theta))

    def log_command(self, v: float, w: float) -> None:
        self.cmds.append((time.time()-self.t0, float(v), float(w)))

    def log_map(self, occ_grid) -> None:
        self.maps.append(np.asarray(occ_grid, dtype=np.uint8).copy())

    def save(self, path: str) -> None:
        # Convert scans to object array to handle variable-length data
        scans_array = np.array(self.scans, dtype=object)
        np.savez_compressed(path, scans=scans_array, odoms=self.odoms,
                            poses=self.poses, cmds=self.cmds, maps=self.maps)


