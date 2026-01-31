#!/usr/bin/env python3
"""
VR control for XLerobot robot
Uses handle_vr_input with delta action control
"""

# Standard library imports
import time
import asyncio
import logging
import threading
import traceback
from scipy.spatial.transform import Rotation
import numpy as np

from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.robot_utils import precise_sleep
from lerobot.robots.xlerobot_pro import XLerobotProConfig, XLerobotPro
from lerobot.utils.visualization_utils import log_rerun_data, init_rerun
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so107_follower.robot_kinematic_processor import (
    EEReferenceAndDelta,
    EEBoundsAndSafety,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.datasets.utils import hw_to_dataset_features, build_dataset_frame

# Local imports
from XLeVR.vr_monitor import VRMonitor
from XLeVR.xlevr.inputs.base import ControlMode

# Setup logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    force=True)
logger = logging.getLogger(__name__)

# Joint mapping configurations
LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "elbow_roll": "left_arm_elbow_roll",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}
RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "elbow_roll": "right_arm_elbow_roll",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

FULL_START_POS = {
    "left_arm_shoulder_pan": 0.0,
    "left_arm_shoulder_lift": -90.0,
    "left_arm_elbow_flex": 45.0,
    "left_arm_elbow_roll": 0.0,
    "left_arm_wrist_flex": 75.0,
    "left_arm_wrist_roll": 0.0,
    "left_arm_gripper": 20.0,
    "right_arm_shoulder_pan": 0.0,
    "right_arm_shoulder_lift": -90.0,
    "right_arm_elbow_flex": 45.0,
    "right_arm_elbow_roll": 0.0,
    "right_arm_wrist_flex": 75.0,
    "right_arm_wrist_roll": 0.0,
    "right_arm_gripper": 20.0,
}

TASK_DESCRIPTION = "do something"
HF_REPO_ID = "xuweiwu/do-something"
NUM_EPISODES = 50
EPISODE_TIME_SEC = 180

FPS = 60
    
class LowPassEMA:
    def __init__(self, dim: int, alpha: float):
        self.y = np.zeros(dim, dtype=float)
        self.initialized = False
        self.alpha = alpha

    def reset(self, value: np.ndarray | None = None):
        if value is None:
            self.y[:] = 0.0
            self.initialized = False
        else:
            self.y[:] = np.asarray(value, dtype=float)
            self.initialized = True

    def step(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if not self.initialized:
            self.y[:] = x
            self.initialized = True
            return self.y.copy()
        self.y += self.alpha * (x - self.y)
        return self.y.copy()

def alpha_from_fc(fc_hz: float, dt: float) -> float:
    # 1st-order discrete LPF derived from continuous-time RC filter
    return float(1.0 - np.exp(-2.0 * np.pi * float(fc_hz) * float(dt)))

class SimpleTeleopArm:
    def __init__(self, joint_map, initial_obs, prefix="left", kp=0.7):
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp

        self.start_pos = {k: FULL_START_POS[v] for k, v in self.joint_map.items()}
        self.home_pos = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "elbow_roll": initial_obs[f"{prefix}_arm_elbow_roll.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": 20,
        }
        self.target_positions = self.start_pos.copy()

        # Delta scaling variables for VR input
        self.vr_relative_position_scaling = 1.0
        self.vr_relative_rotvec_scaling = 1.0
        
        self.max_pos_vel = 1.0 # m/s
        self.max_rad_vel = 180 * (np.pi/180) # rad/s
        self.gripper_vel_step = 1
        self.gripper_clip_min = 20
        self.gripper_clip_max = 100

        self.vr_calibrated = False
        self.vr_headset_to_base = Rotation.from_matrix(np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float))
        self.vr_ctrl_to_ee = None

        self.dt = 1.0 / FPS
        fc_pos_hz = 5.0 # Hz
        fc_rot_hz = 10.0 # Hz
        self.eps_pos = 2.5e-3 * self.dt  # 5 mm/s -> m/tick
        self.eps_rot = 1e-2 * self.dt # 0.05 rad/s (about 3 deg/s) -> rad/tick
        self.vr_stationary_pos = 1e-2 * self.dt  # 10 mm/s -> m/tick
        self.vr_stationary_rot = 4e-2 * self.dt # 0.075 rad/s (about 4.3 deg/s) -> rad/tick
        alpha_pos = alpha_from_fc(fc_pos_hz, self.dt)
        alpha_rot = alpha_from_fc(fc_rot_hz, self.dt)
        alpha_pos = np.clip(alpha_pos, 0.0, 1.0)
        alpha_rot = np.clip(alpha_rot, 0.0, 1.0)
        self.vr_pos_lpf = LowPassEMA(dim=3, alpha=alpha_pos)
        self.vr_rot_lpf = LowPassEMA(dim=3, alpha=alpha_rot)

        joint_names_wo_gripper = [j for j in self.target_positions if j != 'gripper']
        self.kinematics= RobotKinematics(
            urdf_path="/home/that/ambient_urdf/robot.urdf", 
            target_frame_name="gripper_frame_link",
            joint_names=joint_names_wo_gripper,
        )
        self.kinematics.solver.dt = self.dt

        self.ee_relative_to_robot_joints_processor = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
            [   
                EEReferenceAndDelta(
                    kinematics=self.kinematics,
                    end_effector_step_sizes={"x": 1.0, "y": 1.0, "z": 1.0},
                    motor_names=list(self.target_positions.keys()),
                    use_latched_reference=False,
                ),
                EEBoundsAndSafety(
                    end_effector_bounds={"min": [-0.5, -0.5, -0.5], "max": [0.5, 0.5, 0.5]},
                    max_ee_step_m=0.04,
                ),
                GripperVelocityToJoint(
                    speed_factor=50.0,
                    clip_min=self.gripper_clip_min,
                    clip_max=self.gripper_clip_max,
                ),
                InverseKinematicsEEToJoints(
                    kinematics=self.kinematics,
                    motor_names=list(self.target_positions.keys()),
                    weights={"position": 1.0, "orientation": 0.2},
                    initial_guess_current_joints=False,
                ),
            ],
            to_transition=robot_action_observation_to_transition,
            to_output=transition_to_robot_action,
        )
        self.ref_action_when_disabled = None

        self.last_obs = None

    def handle_vr_input(self, robot, vr_goal):
        """
        Handle VR input with delta action control - incremental position updates.
        
        Args:
            vr_goal: VR controller goal data containing target position and orientations
        """
        
        # VR goal contains: 
        # arm: Literal["left", "right"]
        # mode: Optional[ControlMode] = None
        # relative_position: Optional[np.ndarray] = None
        # relative_rotvec: Optional[np.ndarray] = None
        # trigger: Optional[bool] = None
        # thumbstick: Optional[Dict[str, Any]] = None
        # buttons: Optional[Dict[str, Any]] = None
        # metadata: Optional[Dict[str, Any]] = None

        if vr_goal is None:
            return

        mode = getattr(vr_goal, "mode", ControlMode.IDLE)
        mode_val = getattr(mode, "value", mode)
        
        if mode_val == ControlMode.IDLE.value:
            self.vr_calibrated = False
            self.vr_ctrl_to_ee = None
            self.vr_pos_lpf.reset()
            self.vr_rot_lpf.reset()
            return
        
        # If we are not calibrated yet, only accept RESET
        if not self.vr_calibrated:
            if mode_val != ControlMode.RESET.value:
                # Drop/ignore POSITION_CONTROL until RESET arrives
                return
        
        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        obs = {f"{j}.pos": obs_raw[self.joint_map[j]] for j in self.joint_map}
        logger.debug(f"[{self.prefix.capitalize()} ARM TELEOP] Current joint positions: {obs}")
        
        if mode_val == ControlMode.RESET.value:
            q_now = np.array([float(obs[n]) for n in obs if n != "gripper.pos"], dtype=float)
            ee_frame = self.kinematics.forward_kinematics(q_now)
            R_ee = Rotation.from_matrix(ee_frame[:3, :3])
            R_vr = vr_goal.vr_ctrl_rotation
            R_vr_to_base = self.vr_headset_to_base * R_vr
            self.vr_ctrl_to_ee =  R_ee.inv() * R_vr_to_base
            self.vr_calibrated = True
            self.vr_pos_lpf.reset()
            self.vr_rot_lpf.reset()
            logger.debug(f"[{self.prefix.capitalize()} ARM TELEOP] VR reset")
            return
            
        # Extract VR control data
        # Get VR relative pose changes
        vr_relative_position = np.asarray(getattr(vr_goal, "relative_position", np.zeros(3)), dtype=float)# [x, y, z] in meters in vr frame
        vr_relative_rotvec = np.asarray(getattr(vr_goal, "relative_rotvec", np.zeros(3)), dtype=float) # [wx, wy, wz] in radian in vr frame

        # Avoid commanding small motions
        is_stationary_vr_pos = np.linalg.norm(vr_relative_position) < self.vr_stationary_pos
        is_stationary_vr_rot = np.linalg.norm(vr_relative_rotvec) < self.vr_stationary_rot
        if is_stationary_vr_pos:
            vr_relative_position = np.zeros(3, dtype=float)
        if is_stationary_vr_rot:
            vr_relative_rotvec = np.zeros(3, dtype=float)
        
        # Transform vr relative position to robot frame
        # robot | vr
        # x     | -z
        # y     | -x
        # z     |  y
        vr_relative_position *= self.vr_relative_position_scaling
        delta_x = -vr_relative_position[2]
        delta_y = -vr_relative_position[0]
        delta_z = vr_relative_position[1]

        vr_relative_rotvec *= self.vr_relative_rotvec_scaling
        ee_relative_rotvec = self.vr_ctrl_to_ee.apply(vr_relative_rotvec)
        delta_wx = ee_relative_rotvec[0]
        delta_wy = ee_relative_rotvec[1]
        delta_wz = ee_relative_rotvec[2]
        
        delta_pos_limit = self.max_pos_vel/ FPS  # Maximum delta per update (meters)
        delta_rotvec_limit = self.max_rad_vel / FPS  # Maximum angle delta per update (radians)
        
        # Limit delta values to prevent sudden movements
        delta_x = max(-delta_pos_limit, min(delta_pos_limit, delta_x))
        delta_y = max(-delta_pos_limit, min(delta_pos_limit, delta_y))
        delta_z = max(-delta_pos_limit, min(delta_pos_limit, delta_z))
        delta_wx = max(-delta_rotvec_limit, min(delta_rotvec_limit, delta_wx))
        delta_wy = max(-delta_rotvec_limit, min(delta_rotvec_limit, delta_wy))
        delta_wz = max(-delta_rotvec_limit, min(delta_rotvec_limit, delta_wz))        

        raw_pos = np.array([delta_x, delta_y, delta_z], dtype=float)
        raw_rot = np.array([delta_wx, delta_wy, delta_wz], dtype=float)

        # Optional: if VR says "no motion", also reset filter to kill creep
        if is_stationary_vr_pos:
            self.vr_pos_lpf.reset(np.zeros(3))
            filt_pos = np.zeros(3)
        else:
            filt_pos = self.vr_pos_lpf.step(raw_pos)
        
        if is_stationary_vr_rot:
            self.vr_rot_lpf.reset(np.zeros(3))
            filt_rot = np.zeros(3)
        else:
            filt_rot = self.vr_rot_lpf.step(raw_rot)

        delta_x, delta_y, delta_z = filt_pos.tolist()
        delta_wx, delta_wy, delta_wz = filt_rot.tolist()

        target_action = {
            "enabled": True,
            "target_x": 0.0,
            "target_y": 0.0,
            "target_z": 0.0,
            "target_wx": 0.0,
            "target_wy": 0.0,
            "target_wz": 0.0,
            "gripper_vel": 0.0,
        }

        ee_pose_changed = False

        # deadzone on filtered deltas
        pos = np.array([delta_x, delta_y, delta_z])
        rot = np.array([delta_wx, delta_wy, delta_wz])
        if np.linalg.norm(pos) < self.eps_pos:
            pos[:] = 0.0
        if np.linalg.norm(rot) < self.eps_rot:
            rot[:] = 0.0

        if np.any(pos != 0.0):
            target_action["target_x"], target_action["target_y"], target_action["target_z"] = pos
            ee_pose_changed = True
        if np.any(rot != 0.0):
            target_action["target_wx"], target_action["target_wy"], target_action["target_wz"] = rot
            ee_pose_changed = True

        # Handle gripper state directly
        # trigger = getattr(vr_goal, "trigger", None)
        # if trigger is not None:
        #     gripper_pos_des = self.gripper_clip_min + (self.gripper_clip_max - self.gripper_clip_min) * trigger
        #     gripper_vel_des = (gripper_pos_des - obs["gripper.pos"]) / FPS # this means we expect to achieve gripper_pos_des in 1 sec.
        #     target_action["gripper_vel"] = gripper_vel_des

        thumb = getattr(vr_goal, "thumbstick", None)
        if thumb is not None:
            thumb_y = thumb.get('y', 0)
            if abs(thumb_y) > 0.25:
                if thumb_y > 0:
                    target_action["gripper_vel"] = -self.gripper_vel_step  # Move thumbstick backward to open gripper
                else:
                    target_action["gripper_vel"] = self.gripper_vel_step  # Move thumbstick forward to close gripper

        logger.debug(f"[{self.prefix.capitalize()} ARM TELEOP] Relative ee actions: {target_action}")
        desired_action = self.ee_relative_to_robot_joints_processor((target_action, obs))
        if not ee_pose_changed:
            if self.ref_action_when_disabled is None:
                self.ref_action_when_disabled = obs.copy()
            ref_action = self.ref_action_when_disabled.copy()
            ref_action['gripper.pos'] = desired_action['gripper.pos']   
        else:
            ref_action = desired_action.copy()
        self.ref_action_when_disabled = ref_action.copy()
        
        for key, ref_pos in ref_action.items():
            self.target_positions[key.removesuffix('.pos')] = ref_pos
        self.last_obs = obs.copy()
        logger.debug(f"[{self.prefix.capitalize()} ARM TELEOP] Ref joint positions: {self.target_positions}")
    
    def p_control_action(self, robot):

        if self.last_obs is None:
            if self.prefix=="left":
                obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
            else:
                obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
            obs = {}
            obs_no_prefix = {}
            for short_name, bus_name in self.joint_map.items():
                raw_value = obs_raw[bus_name]
                obs[f"{bus_name}.pos"] = raw_value
                obs_no_prefix[short_name] = raw_value
        else:
            obs_no_prefix = {k.removesuffix(".pos"): v for k, v in self.last_obs.items()}
            obs = {f"{self.prefix}_arm_{k}": v for k, v in self.last_obs.items()}
            self.last_obs = None


        # joint deadzone config
        tick_deg = 360.0 / 4096.0  # ≈ 0.0879 deg
        deadband_ticks = 1         # tune: 1..3
        deadband_deg = deadband_ticks * tick_deg

        action = {}
        for j in self.target_positions:
            q = float(obs_no_prefix[j])
            q_des = float(self.target_positions[j])
            error = q_des - q

            if j != "gripper":
                # joint deadzone (degrees)
                if abs(error) <= deadband_deg:
                    cmd = q  # hold position
                else:
                    cmd = q + self.kp * error
            else:
                # no deadzone for gripper (normalized [0,100])
                cmd = q + self.kp * error

            action[f"{self.joint_map[j]}.pos"] = cmd
        
        logger.debug(f"[{self.prefix.capitalize()} ARM CONTROL] Commanded actions: {action}")
        return action, obs

def move_to_target_full_body_with_ipol(
        robot, left_teleop, right_teleop,
        target_positions=None, duration=3.0, control_freq=200.0, kp=0.8,
        max_vel_per_joint=None, max_acc_per_joint=None, max_dev_per_joint=None):
        """
        Plan a quadratic-spline trajectory from current q to zero/init pos and execute it
        at fixed control frequency. Finishes exactly at `duration` if feasible.

        Raises:
            ValueError if requested duration is infeasible given the limits.
        """
        # 0) define target order explicitly via target_positions (canonical order)
        if target_positions is None:
            left_target_pos = {v: left_teleop.home_pos[k] for k, v in LEFT_JOINT_MAP.items()}
            right_target_pos = {v: right_teleop.home_pos[k] for k, v in RIGHT_JOINT_MAP.items()}
            target_positions = {**left_target_pos, **right_target_pos}
        
        # 1) Read current joint positions
        left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        right_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        obs = {**left_obs, **right_obs}
        logger.debug(f"[FULL-BODY IPOL] Current joint positions: {obs}")

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            logger.debug(f"[FULL-BODY IPOL] Warning: skipping joints missing in observation: {missing}")

        # 3) build current/goal vectors in that SAME order (no sorting)
        q_now = []
        q_goal = []
        for n in names:
            v = float(obs[n])
            q_now.append(v)
            q_goal.append(float(target_positions[n]))
        q_now  = np.array(q_now,  dtype=float)
        q_goal = np.array(q_goal, dtype=float)

        # 4) Limits (defaults if not provided)
        J = q_now.size
        if max_vel_per_joint is None:
            max_vel_per_joint = np.full(J, 30)   # deg/s (pick something reasonable)
        if max_acc_per_joint is None:
            max_acc_per_joint = np.full(J, 60)   # deg/s^2
        if max_dev_per_joint is None:
            # for a 2-via move, deviation isn't essential; keep tiny to retain quadratic plumbing
            max_dev_per_joint = np.full(J, 0.0)

        # 5) Build a 2-via path (current -> goal). You can insert mid vias if you want shaping.
        via = [
            Via(q=q_now,  max_dev=max_dev_per_joint),
            Via(q=q_goal, max_dev=max_dev_per_joint),
        ]
        lim = Limits(max_vel=np.asarray(max_vel_per_joint),
                    max_acc=np.asarray(max_acc_per_joint))

        ipol = QuadraticSplineInterpolator(via, lim)
        ipol.build()  # builds pieces, samples, ds envelope and forward/backward feasible ds(s)
        
        # 6) Slow-down scale so we finish exactly at 'duration'
        # Scaling ds(s) by k scales time as T = T_min / k -> choose k = T_min / duration
        ipol.scale_to_duration(duration)

        # 7) Generate time samples and joint references at controller rate
        dt = 1.0/ float(control_freq)
        t, q, qd, qdd = ipol.resample(dt)  # will end ~ at `duration`

        # 8) Stream to the robot
        logger.info(f"[FULL-BODY IPOL] Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
            f"planned duration ≈ {t[-1]:.3f}s (requested {duration:.3f}s)")
        
        t0 = time.perf_counter()
        next_tick = t0
        for k_step in range(len(t)):
            
            left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
            right_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
            obs = {**left_obs, **right_obs}

            q_meas = np.array([float(obs[n]) for n in names], dtype=float)
            q_ref = np.array([q[k_step, i] for i, n in enumerate(names)], dtype=float)
            q_cmd = q_meas + kp*(q_ref - q_meas)
            action = {f"{j}.pos": q_cmd[i] for i, j in enumerate(names)}
            robot.send_action(action)

            # sleep to maintain control_freq (best-effort wall clock pacing)
            next_tick += dt
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(next_tick - now)
        
        logger.debug("[FULL-BODY IPOL] Reached end of ipol trajectory.")
        for k, v in LEFT_JOINT_MAP.items():
            left_teleop.target_positions[k] = target_positions[v]
        logger.debug(f"[FULL-BODY IPOL] Reset left arm target positions to: {left_teleop.target_positions}")
        for k, v in RIGHT_JOINT_MAP.items():
            right_teleop.target_positions[k] = target_positions[v]
        logger.debug(f"[FULL-BODY IPOL] Reset right arm target positions to: {right_teleop.target_positions}")

        left_teleop.ee_relative_to_robot_joints_processor.reset()
        left_teleop.ref_action_when_disabled = None
        right_teleop.ee_relative_to_robot_joints_processor.reset()
        right_teleop.ref_action_when_disabled = None

def main():
    print("XLerobot Pro VR Control Example")
    print("="*50)

    shutdown_event = threading.Event()
    # saving_in_progress_event = threading.Event()

    robot, vr_monitor = None, None
    robot_name = "ambient_xlerobot_pro"
    try:
        robot_config = XLerobotProConfig(id=robot_name, use_degrees=True)
        robot = XLerobotPro(robot_config)
        robot.connect()
        print("[MAIN] Robot connection successful!")
        print(f"[MAIN] Motor bus_left_base info: {robot.bus_left_base.motors}")
        print(f"[MAIN] Motor bus_right_head info: {robot.bus_right_head.motors}") 
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(f"[MAIN] Robot config: {robot_config}")
        print(f"[MAIN] Robot: {robot}")
        traceback.print_exc()
        return
    
    continue_choice = input("Do you want to continue to teleop the robot? (y/n, [default y]): ").strip().lower()
    if continue_choice in ['n', 'no']:
        print(f"[MAIN] User decided not to continue")
        left_base_pos = robot.bus_left_base.sync_read("Present_Position", normalize=False)
        right_head_pos = robot.bus_right_head.sync_read("Present_Position", normalize=False)
        print(f"[MAIN] Left base states: {left_base_pos}")
        print(f"[MAIN] Right head states: {right_head_pos}")
        return
    
    try:
        # Init the arm instances
        print("🔧 Moving robot to start pose...")
        obs = robot.get_observation()
        left_arm_teleop = SimpleTeleopArm(LEFT_JOINT_MAP, obs, prefix="left")
        right_arm_teleop = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, prefix="right")

        # Move both arms to zero position at start
        move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, target_positions=FULL_START_POS)
        print("✅ Robot in start pose")
        
        # Initialize VR monitor
        print("🔧 Initializing VR monitor...")
        vr_monitor = VRMonitor()
        if not vr_monitor.initialize():
            print("❌ VR monitor initialization failed")
            return
        print("🚀 Starting VR monitoring...")
        vr_thread = threading.Thread(target=lambda: asyncio.run(vr_monitor.start_monitoring()), daemon=True)
        vr_thread.start()
        print("✅ VR system ready")

        print("Starting record loop...")
        while not shutdown_event.is_set():
            start = time.perf_counter()

            left_reset, left_motion = vr_monitor.pop_ordered_goals_for_arm("left")
            right_reset, right_motion = vr_monitor.pop_ordered_goals_for_arm("right")

            # For convenience, define "current" goals per arm (prefer motion, else reset)
            left_goal = left_motion or left_reset
            right_goal = right_motion or right_reset

            if left_goal is None and right_goal is None:
                # No new VR commands for either arm.
                # We can still keep sending whatever the teleop objects currently hold,
                # but we don't update them this cycle.
                
                # Optionally still send actions based on current teleop state:
                left_action, _ = left_arm_teleop.p_control_action(robot)
                right_action, _ = right_arm_teleop.p_control_action(robot)

                action = {**left_action, **right_action}
                robot.send_action(action)

                dt_s = time.perf_counter() - start
                precise_sleep(1 / FPS - dt_s)
                continue
            
            # Right B on right controller to stop
            for g in (right_motion, right_reset):
                if g and getattr(g, "buttons", None):
                    if g.buttons.get("B", 0):
                        print("User ended teleop by pressing right button B")
                        shutdown_event.set()
                        break
                    if g.buttons.get("A", 0):
                        move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, target_positions=FULL_START_POS)
                        print("User moved all joints to start pose by pressing right button X")
            if shutdown_event.is_set():
                print("Shutdown event triggered")
                break

            camera_obs = robot.get_camera_observation()

            # Handle VR input for both arms (RESET first, then motion)
            # LEFT ARM
            if left_reset is not None:
                left_arm_teleop.handle_vr_input(robot, left_reset)
            if left_motion is not None:
                left_arm_teleop.handle_vr_input(robot, left_motion)

            # RIGHT ARM
            if right_reset is not None:
                right_arm_teleop.handle_vr_input(robot, right_reset)
            if right_motion is not None:
                right_arm_teleop.handle_vr_input(robot, right_motion)

            # Compute actions from both arms
            left_action, left_obs = left_arm_teleop.p_control_action(robot)
            right_action, right_obs = right_arm_teleop.p_control_action(robot)

            # Merge all actions
            action = {**left_action, **right_action}
            robot.send_action(action)

            obs = {**left_obs, **right_obs, **camera_obs}

            dt_s = time.perf_counter() - start
            # dt_ms = dt_s * 1e3
            # print(f"Control delay: {dt_ms:.1f}ms")
            precise_sleep(1 / FPS - dt_s)
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        
    finally:
        # Cleanup
        shutdown_event.set()

        if robot:
            move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop)
            robot.disconnect()

if __name__ == "__main__":
    main()
