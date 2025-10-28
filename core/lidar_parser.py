# ================================
# file: core/lidar_parser.py
# ================================
"""
LiDAR data parsing utilities for navigation and mapping
Handles parsing of sparse LiDAR points from serial communication
"""

from __future__ import annotations
from typing import Optional, Dict, List
import math
import json

try:
    import numpy as np
except ImportError:
    np = None


def parse_lidar_line(line: str) -> Optional[Dict]:
    """Parse one lidar sparse point line - 与test_lidar_mapping.py完全一致
    Expected format compatible with your logger, e.g:
      'LIDAR Qxx A123.4 D0567'
    Returns dict {'angle_deg': float, 'distance_m': float, 'quality': int}
    or None if not a lidar line.
    """
    line = line.strip()
    if not line:
        return None
    
    # Single-point line format: "LIDAR Q15 A123.4 D0567" - 与test_lidar_mapping.py一致
    if line.startswith("LIDAR"):
        try:
            parts = line.split()
            if len(parts) >= 4:
                quality = int(parts[1][1:])      # Q15 -> 15
                angle_deg = float(parts[2][1:])  # A123.4 -> 123.4
                distance_mm = float(parts[3][1:]) # D0567 -> 567
                distance_m = distance_mm / 1000.0  # Convert mm to meters
                
                return {
                    "angle_deg": angle_deg,
                    "distance_m": distance_m,
                    "quality": quality
                }
        except (ValueError, IndexError):
            return None
    
    # JSON full frame (if needed in future) - 与test_lidar_mapping.py一致
    if line.startswith("{") and "\"type\"" in line:
        try:
            msg = json.loads(line)
            if msg.get("type") == "L" and "ranges" in msg:
                # Convert JSON format to sparse points (but we only handle first valid point)
                amin = float(msg.get("angle_min", -math.pi))
                ainc = float(msg.get("angle_increment", 2*math.pi/len(msg["ranges"])))
                ranges = [float(r) for r in msg["ranges"]]
                
                for i, range_val in enumerate(ranges):
                    if np is not None and np.isfinite(range_val):
                        angle_rad = amin + i * ainc
                        angle_deg = math.degrees(angle_rad)
                        return {
                            "angle_deg": angle_deg,
                            "distance_m": range_val,
                            "quality": 50  # Default quality for JSON data
                        }
        except Exception:
            return None
    
    return None

