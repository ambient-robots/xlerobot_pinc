"""
Base classes and data structures for input providers.
"""

import asyncio
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any
from enum import Enum
from scipy.spatial.transform import Rotation

class ControlMode(Enum):
    """Control modes for the teleoperation system."""
    POSITION_CONTROL = "position"
    IDLE = "idle"
    RESET = "reset"

@dataclass
class ControlGoal:
    """High-level control goal message sent from input providers."""
    arm: Literal["left", "right"]
    mode: Optional[ControlMode] = None             # control mode (None = no mode change)
    relative_position: Optional[np.ndarray] = None # 3D relative position change in vr coordinates
    relative_rotvec: Optional[np.ndarray] = None   # 3D relative rotvec change in vr coordinates
    trigger: Optional[bool] = None                 # trigger status
    thumbstick: Optional[Dict[str, Any]] = None    # thumbstick info
    buttons: Optional[Dict[str, Any]] = None       # buttons info
    vr_ctrl_rotation: Optional[Rotation] = None    # vr ctrl original rotation when grip pressed
    
    # Additional data for debugging/monitoring
    metadata: Optional[Dict[str, Any]] = None

class BaseInputProvider(ABC):
    """Abstract base class for input providers."""
    
    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue
        self.is_running = False
    
    @abstractmethod
    async def start(self):
        """Start the input provider."""
        pass
    
    @abstractmethod
    async def stop(self):
        """Stop the input provider."""
        pass
    
    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue."""
        try:
            await self.command_queue.put(goal)
        except Exception as e:
            # Handle queue full or other errors
            pass 