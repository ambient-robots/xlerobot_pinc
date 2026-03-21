"""
Input providers for the teleoperation system.
Contains VR WebSocket server and keyboard listener implementations.
"""

from .base import ControlGoal, ControlMode

__all__ = [
    "ControlGoal",
    "ControlMode",
]

try:
    from .vr_ws_server import VRWebSocketServer
except ImportError:
    VRWebSocketServer = None
else:
    __all__.append("VRWebSocketServer")
