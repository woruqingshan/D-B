# ================================
# file: explore/entry_detector.py
# ================================
from __future__ import annotations
"""Entry detection module for robot navigation system.
Handles real-time maze entry detection from LiDAR data and navigation state management.
"""
import math
from typing import Optional, Tuple, Dict, Any

from nav import NavState


class EntryDetector:
    """Handles real-time maze entry detection and navigation state management."""

    def __init__(self, logger_func=None):
        self.logger_func = logger_func
        self.log_file = None
        self.detection_enabled = True
        self.last_detection_time = 0.0
        self.detection_cooldown = 2.0  # 检测冷却时间（秒）
        self.detection_count = 0
        self.max_detections = 3  # 最大检测次数

    def _log(self, message: str, module: str = "ENTRY_DETECT") -> None:
        """Log message using the provided logger function"""
        if self.logger_func and self.log_file:
            self.logger_func(self.log_file, message, module)

    def detect_maze_entry_from_lidar(self, scan, robot_pose, maze_info) -> Optional[Tuple[float, float]]:
        """
        Detect maze entry from LiDAR scan data.
        Identifies entry by analyzing "channel" patterns in scan data.
        
        Parameters
        ----------
        scan : LaserScan
            LiDAR scan data
        robot_pose : Pose
            Current robot pose
        maze_info : Dict[str, Any]
            Maze information dictionary
            
        Returns
        -------
        Optional[Tuple[float, float]]
            Detected entry coordinates or None if no entry found
        """
        if not self.detection_enabled:
            return None
            
        if scan is None or len(scan.ranges) == 0:
            return None
        
        # Analyze scan data to find possible entry channels
        # Entry characteristics: far distance ahead, obstacles on both sides
        front_angles = range(len(scan.ranges) // 3, 2 * len(scan.ranges) // 3)  # Front 120 degrees
        front_ranges = [scan.ranges[i] for i in front_angles if i < len(scan.ranges)]
        
        if not front_ranges:
            return None
        
        # Find the farthest point (possible entry)
        max_range_idx = front_angles[front_ranges.index(max(front_ranges))]
        max_range = scan.ranges[max_range_idx]
        
        # Check if it meets entry conditions: moderate distance (not too far or too close)
        if 2.0 < max_range < 8.0:
            # Calculate position in robot coordinate system
            angle = scan.angle_min + max_range_idx * scan.angle_increment
            entry_x = robot_pose.x + max_range * math.cos(angle + robot_pose.theta)
            entry_y = robot_pose.y + max_range * math.sin(angle + robot_pose.theta)
            
            return (entry_x, entry_y)
        
        return None

    def analyze_lidar_for_entry_patterns(self, scan, robot_pose) -> Dict[str, Any]:
        """
        Analyze LiDAR data for entry patterns and return detailed analysis.
        
        Parameters
        ----------
        scan : LaserScan
            LiDAR scan data
        robot_pose : Pose
            Current robot pose
            
        Returns
        -------
        Dict[str, Any]
            Analysis results including patterns, potential entries, and confidence
        """
        if scan is None or len(scan.ranges) == 0:
            return {"valid": False, "reason": "No scan data"}
        
        analysis = {
            "valid": True,
            "total_points": len(scan.ranges),
            "angle_range": (scan.angle_min, scan.angle_max),
            "range_stats": {
                "min": min(scan.ranges),
                "max": max(scan.ranges),
                "mean": sum(scan.ranges) / len(scan.ranges)
            },
            "patterns": [],
            "potential_entries": [],
            "confidence": 0.0
        }
        
        # Analyze different angular sectors
        sectors = [
            ("front", range(len(scan.ranges) // 3, 2 * len(scan.ranges) // 3)),
            ("left", range(0, len(scan.ranges) // 3)),
            ("right", range(2 * len(scan.ranges) // 3, len(scan.ranges)))
        ]
        
        for sector_name, sector_indices in sectors:
            sector_ranges = [scan.ranges[i] for i in sector_indices if i < len(scan.ranges)]
            if sector_ranges:
                sector_analysis = {
                    "name": sector_name,
                    "min_range": min(sector_ranges),
                    "max_range": max(sector_ranges),
                    "avg_range": sum(sector_ranges) / len(sector_ranges),
                    "point_count": len(sector_ranges)
                }
                analysis["patterns"].append(sector_analysis)
                
                # Check for entry patterns in front sector
                if sector_name == "front":
                    # Look for "channel" pattern: medium distance with obstacles on sides
                    if 2.0 < sector_analysis["max_range"] < 8.0:
                        # Calculate potential entry point
                        max_range_idx = sector_indices[sector_ranges.index(sector_analysis["max_range"])]
                        angle = scan.angle_min + max_range_idx * scan.angle_increment
                        entry_x = robot_pose.x + sector_analysis["max_range"] * math.cos(angle + robot_pose.theta)
                        entry_y = robot_pose.y + sector_analysis["max_range"] * math.sin(angle + robot_pose.theta)
                        
                        entry_info = {
                            "position": (entry_x, entry_y),
                            "distance": sector_analysis["max_range"],
                            "angle": angle,
                            "confidence": self._calculate_entry_confidence(sector_analysis)
                        }
                        analysis["potential_entries"].append(entry_info)
        
        # Calculate overall confidence
        if analysis["potential_entries"]:
            analysis["confidence"] = max(entry["confidence"] for entry in analysis["potential_entries"])
        
        return analysis

    def _calculate_entry_confidence(self, sector_analysis: Dict[str, Any]) -> float:
        """
        Calculate confidence score for a potential entry detection.
        
        Parameters
        ----------
        sector_analysis : Dict[str, Any]
            Analysis of a sector
            
        Returns
        -------
        float
            Confidence score between 0.0 and 1.0
        """
        confidence = 0.0
        
        # Distance-based confidence (optimal range: 3-6 meters)
        distance = sector_analysis["max_range"]
        if 3.0 <= distance <= 6.0:
            confidence += 0.4
        elif 2.0 <= distance <= 8.0:
            confidence += 0.2
        
        # Range variation confidence (some variation indicates obstacles)
        range_variation = sector_analysis["max_range"] - sector_analysis["min_range"]
        if 1.0 <= range_variation <= 5.0:
            confidence += 0.3
        elif range_variation > 5.0:
            confidence += 0.1
        
        # Point density confidence (enough points for reliable detection)
        if sector_analysis["point_count"] >= 50:
            confidence += 0.3
        elif sector_analysis["point_count"] >= 30:
            confidence += 0.2
        
        return min(confidence, 1.0)

    def should_attempt_detection(self, current_time: float, nav_state: NavState) -> bool:
        """
        Determine if entry detection should be attempted.
        
        Parameters
        ----------
        current_time : float
            Current time in seconds
        nav_state : NavState
            Current navigation state
            
        Returns
        -------
        bool
            True if detection should be attempted
        """
        # Check if detection is enabled
        if not self.detection_enabled:
            return False
        
        # Check cooldown period
        if current_time - self.last_detection_time < self.detection_cooldown:
            return False
        
        # Check maximum detection attempts
        if self.detection_count >= self.max_detections:
            return False
        
        # Only attempt detection in certain navigation states
        allowed_states = [NavState.EXPLORING, NavState.IDLE]
        if nav_state not in allowed_states:
            return False
        
        return True

    def update_detection_state(self, detection_successful: bool, current_time: float) -> None:
        """
        Update detection state after an attempt.
        
        Parameters
        ----------
        detection_successful : bool
            Whether the detection was successful
        current_time : float
            Current time in seconds
        """
        self.last_detection_time = current_time
        
        if detection_successful:
            self.detection_count += 1
            self._log(f"Entry detection successful. Count: {self.detection_count}/{self.max_detections}")
        else:
            self._log(f"Entry detection failed. Count: {self.detection_count}/{self.max_detections}")

    def reset_detection_state(self) -> None:
        """Reset detection state (e.g., when entering a new phase)."""
        self.detection_count = 0
        self.last_detection_time = 0.0
        self._log("Entry detection state reset")

    def enable_detection(self) -> None:
        """Enable entry detection."""
        self.detection_enabled = True
        self._log("Entry detection enabled")

    def disable_detection(self) -> None:
        """Disable entry detection."""
        self.detection_enabled = False
        self._log("Entry detection disabled")

    def set_detection_parameters(self, cooldown: float = 2.0, max_detections: int = 3) -> None:
        """
        Set detection parameters.
        
        Parameters
        ----------
        cooldown : float
            Detection cooldown time in seconds
        max_detections : int
            Maximum number of detection attempts
        """
        self.detection_cooldown = cooldown
        self.max_detections = max_detections
        self._log(f"Detection parameters updated: cooldown={cooldown}s, max_detections={max_detections}")

    def get_detection_status(self) -> Dict[str, Any]:
        """
        Get current detection status.
        
        Returns
        -------
        Dict[str, Any]
            Current detection status
        """
        return {
            "enabled": self.detection_enabled,
            "count": self.detection_count,
            "max_detections": self.max_detections,
            "cooldown": self.detection_cooldown,
            "last_detection": self.last_detection_time
        } 