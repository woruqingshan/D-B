# ================================
# file: slam/__init__.py
# ================================
"""
SLAM Package with Decoupled Mapping and Localization

Exports:
- SlamSystem: Main SLAM controller (refactored with decoupled architecture)
- MapBuilder: Pure mapping module
- Localizer: Pure localization module
"""
from slam.slam_system import SlamSystem
from slam.map_builder import MapBuilder
from slam.localizer import Localizer

__all__ = [
    'SlamSystem',
    'MapBuilder',
    'Localizer',
]