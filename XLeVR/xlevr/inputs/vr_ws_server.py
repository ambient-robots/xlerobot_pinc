"""
VR WebSocket server for receiving controller data from web browsers.
Adapted from the original vr_robot_teleop.py script.
"""

import asyncio
import json
import ssl
import websockets
import numpy as np
import math
import logging
from typing import Dict, Optional, Set
from scipy.spatial.transform import Rotation

from .base import BaseInputProvider, ControlGoal, ControlMode
from ..config import XLeVRConfig

logger = logging.getLogger(__name__)


class VRControllerState:
    """State tracking for a VR controller."""
    
    def __init__(self, hand: str):
        self.hand = hand
        self.grip_active = False
        
        # Position tracking for relative movement
        self.origin_position = None
        
        # Quaternion-based rotation tracking (more stable than Euler)
        self.origin_quaternion = None
        self.origin_rotation = None
    
    def reset_grip(self):
        """Reset grip state but preserve trigger state."""
        self.grip_active = False
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None
    
    def reset_origin(self):
        """Reset origin position and rotation for auto-control mode."""
        self.origin_position = None
        self.origin_rotation = None
        self.origin_quaternion = None


class VRWebSocketServer(BaseInputProvider):
    """WebSocket server for VR controller input."""
    
    def __init__(self, command_queue: asyncio.Queue, config: XLeVRConfig, print_only: bool = False):
        super().__init__(command_queue)
        self.config = config
        self.clients: Set = set()
        self.server = None
        self.print_only = print_only  # New flag for print-only mode
        
        # Controller states
        self.left_controller = VRControllerState("left")
        self.right_controller = VRControllerState("right")
    
    def setup_ssl(self) -> Optional[ssl.SSLContext]:
        """Setup SSL context for WebSocket server."""
        # Automatically generate SSL certificates if they don't exist
        if not self.config.ssl_files_exist:
            logger.info("SSL certificates not found for WebSocket server, attempting to generate them...")
            if not self.config.ensure_ssl_certificates():
                logger.error("Failed to generate SSL certificates for WebSocket server")
                return None
        
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        try:
            ssl_context.load_cert_chain(certfile=self.config.certfile, keyfile=self.config.keyfile)
            logger.info("SSL certificate and key loaded successfully for WebSocket server")
            return ssl_context
        except ssl.SSLError as e:
            logger.error(f"Error loading SSL cert/key: {e}")
            return None
    
    async def start(self):
        """Start the WebSocket server."""
        if not self.config.enable_vr:
            logger.info("VR WebSocket server disabled in configuration")
            return
        
        ssl_context = self.setup_ssl()
        if ssl_context is None:
            logger.error("Failed to setup SSL for WebSocket server")
            return
        
        host = self.config.host_ip
        port = self.config.websocket_port
        
        try:
            self.server = await websockets.serve(
                self.websocket_handler, 
                host, 
                port, 
                ssl=ssl_context
            )
            self.is_running = True
            logger.info(f"VR WebSocket server running on wss://{host}:{port}")
        except Exception as e:
            logger.error(f"Failed to start WebSocket server: {e}")
    
    async def stop(self):
        """Stop the WebSocket server."""
        self.is_running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info("VR WebSocket server stopped")
    
    async def websocket_handler(self, websocket, path=None):
        """Handle WebSocket connections from VR controllers."""
        client_address = websocket.remote_address
        logger.info(f"VR client connected: {client_address}")
        self.clients.add(websocket)
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_controller_data(data)
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON message: {message}")
                except Exception as e:
                    logger.error(f"Error processing VR data: {e}")
                    # Add more context for debugging
                    logger.error(f"Data that caused error: {data}")
                    import traceback
                    logger.error(f"Traceback: {traceback.format_exc()}")
        
        except websockets.exceptions.ConnectionClosedOK:
            logger.info(f"VR client {client_address} disconnected normally")
        except websockets.exceptions.ConnectionClosedError as e:
            logger.warning(f"VR client {client_address} disconnected with error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error with VR client {client_address}: {e}")
        finally:
            self.clients.discard(websocket)
            # Handle grip releases when client disconnects
            await self.handle_grip_release('left')
            await self.handle_grip_release('right')
            logger.info(f"VR client {client_address} cleanup complete")
    
    async def process_controller_data(self, data: Dict):
        """Process incoming VR controller data."""
        
        # Process controller data
        if 'leftController' in data:
            await self.process_single_controller('left', data['leftController'])
        
        if 'rightController' in data:
            await self.process_single_controller('right', data['rightController'])
    
    async def process_single_controller(self, hand: str, data: Dict):
        """Process data for a single controller."""
        position = data.get('position', {})
        rotation = data.get('rotation', {})
        quaternion = data.get('quaternion', {})  # Get quaternion data directly
        grip_active = data.get('gripActive', False)
        trigger = data.get('trigger', 0)
        thumbstick = data.get('thumbstick', {})
        buttons = data.get('buttons', {})
        
        controller = self.left_controller if hand == 'left' else self.right_controller        
        if not grip_active:
            await self.handle_grip_release(hand, buttons)
        else:
            if not controller.grip_active:
                controller.grip_active = True

                # reset original position
                controller.origin_position = np.array([position.get('x', 0), position.get('y', 0), position.get('z', 0)])
                
                # reset original quaternion
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    controller.origin_quaternion = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                else:
                    controller.origin_quaternion = self.euler_to_quaternion(rotation) if rotation else None
                
                controller.origin_rotation = Rotation.from_quat(controller.origin_quaternion)

                # send reset goal
                reset_goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.RESET,
                    vr_ctrl_rotation=controller.origin_rotation,
                    metadata={
                        "source": f"vr_grip_reset_{hand}",
                    }
                )
                await self.send_goal(reset_goal)
                logger.info(f"🔒 {hand.upper()} grip activated - controlling {hand} arm (origin reset to current)")
                # IMPORTANT: do NOT also send POSITION_CONTROL in this same call
                return
            
            if controller.origin_position is not None:
                current_position = np.array([position.get('x', 0), position.get('y', 0), position.get('z', 0)])

                relative_position_unscaled = current_position - controller.origin_position
                relative_position = relative_position_unscaled * self.config.vr_to_robot_pos_scale
                controller.origin_position = current_position
                
                if quaternion and all(k in quaternion for k in ['x', 'y', 'z', 'w']):
                    current_quat = np.array([quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']])
                else:
                    current_quat = self.euler_to_quaternion(rotation)

                relative_rotvec_unscaled = self.compute_relative_rotvec(controller.origin_quaternion, current_quat)
                relative_rotvec = relative_rotvec_unscaled * self.config.vr_to_robot_ori_scale
                controller.origin_quaternion = current_quat
                
                goal = ControlGoal(
                    arm=hand,
                    mode=ControlMode.POSITION_CONTROL,
                    relative_position=relative_position,
                    relative_rotvec=relative_rotvec,
                    trigger=trigger,
                    thumbstick=thumbstick,
                    buttons=buttons,
                    metadata={
                        "source": f"vr_relative_pose_{hand}",
                    }
                )
                await self.send_goal(goal)
            
    
    async def handle_grip_release(self, hand: str, buttons):
        """Handle grip release for a controller."""
        if hand == 'left':
            controller = self.left_controller
        elif hand == 'right':
            controller = self.right_controller
        else:
            return
        
        # Only reset grip and send idle command when it is not reset
        # or when button pressed
        if controller.grip_active:
            controller.reset_grip() 
            # Send idle goal to stop arm control
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                buttons=buttons,
                metadata={
                    "source": "vr_grip_release",
                }
            )
            await self.send_goal(goal)
            logger.info(f"🔓 {hand.upper()} grip released - arm control stopped")
            return
        
        if any(buttons.values()):
            # Send idle goal with any button-pressed events
            goal = ControlGoal(
                arm=hand,
                mode=ControlMode.IDLE,
                buttons=buttons,
                metadata={
                    "source": "vr_button_pressed",
                }
            )
            await self.send_goal(goal)
            logger.info(f"🔓 {hand.upper()} button pressed")
        
    
    def euler_to_quaternion(self, euler_deg: Dict[str, float]) -> np.ndarray:
        """Convert Euler angles in degrees to quaternion [x, y, z, w]."""
        euler_rad = [math.radians(euler_deg['x']), math.radians(euler_deg['y']), math.radians(euler_deg['z'])]
        rotation = R.from_euler('xyz', euler_rad)
        return rotation.as_quat()

    def quat_normalize(self, quat):
        """Normalize quaternion quat = [x, y, z, w]."""
        quat = np.array(quat, dtype=float)
        return quat / np.linalg.norm(quat)

    def quat_conjugate(self, quat):
        """Conjugate of quaternion quat = [x, y, z, w]."""
        quat = np.asarray(quat, dtype=float)
        return np.array([-quat[0], -quat[1], -quat[2], quat[3]])
    
    def quat_pow(self, quat, scale):
        """
        Raise unit quaternion quat to real power scale, i.e. scale its rotation angle by s.
        q must be unit (will be renormalized here just in case).
        """
        quat = self.quat_normalize(quat)
        x, y, z, w = quat

        # Clamp w to valid range for acos, to avoid numerical issues
        w = np.clip(w, -1.0, 1.0)

        theta = 2.0 * np.arccos(w)  # rotation angle in [0, pi]
        if theta < 1e-8:
            # Very small rotation -> return identity (or q itself, they’re very close)
            return np.array([0.0, 0.0, 0.0, 1.0])

        sin_half = np.sin(theta / 2.0)
        # Rotation axis
        ux = x / sin_half
        uy = y / sin_half
        uz = z / sin_half

        theta_s = scale * theta
        w_s = np.cos(theta_s / 2.0)
        sin_half_s = np.sin(theta_s / 2.0)
        x_s = ux * sin_half_s
        y_s = uy * sin_half_s
        z_s = uz * sin_half_s

        return self.quat_normalize(np.array([x_s, y_s, z_s, w_s]))
    
    def quat_mul(self, q1, q2):
        """Quaternion product q = q1 * q2 (x, y, z, w convention)."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        ])
    
    def compute_relative_rotvec(self, origin_quat: np.ndarray, current_quat: np.ndarray):
        """
        Given start orientation q0 and target orientation q1 (both as [w,x,y,z]),
        return orientation q_s that is s-fraction of the way from q0 to q1.

        s = 0 -> q0
        s = 1 -> q1
        s between 0 and 1 -> intermediate orientation
        """

        if current_quat is None or origin_quat is None:
            return np.array([0.0, 0.0, 0.0, 1.0])
        
        origin_quat = self.quat_normalize(origin_quat)
        current_quat = self.quat_normalize(current_quat)

        # Ensure shortest path (handle q and -q equivalence)
        if np.dot(origin_quat, current_quat) < 0.0:
            current_quat = -current_quat

        # Relative rotation:  origin_quat * relative_quat = current_quat  => relative_quat = conj(origin_quat) * current_quat
        relative_quat = self.quat_mul(self.quat_conjugate(origin_quat), current_quat)
        R_rel = Rotation.from_quat(relative_quat)

        return R_rel.as_rotvec()  # vector direction = axis, norm = angle
    
    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue or print it if in print-only mode."""
        if self.print_only:
            # Print the ControlGoal in a formatted way
            print(f"\n🎮 ControlGoal:")
            print(f"   Arm: {goal.arm}")
            print(f"   Mode: {goal.mode}")
            if goal.relative_position is not None:
                print(f"   Relative Position: {goal.relative_position}")
            if goal.relative_rotvec is not None:
                print(f"   Relative Rotvec: {goal.relative_rotvec}")
            if goal.vr_ctrl_rotation is not None:
                print(f"   VR Frame: {goal.vr_ctrl_rotation}")
            if goal.trigger is not None:
                print(f"   Trigger: {goal.trigger}")
            if goal.thumbstick is not None:
                print(f"   Thumbstick: x={goal.thumbstick['x']:.2f}, y={goal.thumbstick['y']:.2f}")
            if goal.buttons is not None:
                print(f"   Buttons: {goal.buttons}")
            if goal.metadata:
                print(f"   Metadata: {goal.metadata}")
        else:
            # Use the parent class method to send to queue
            await super().send_goal(goal) 