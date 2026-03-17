#!/usr/bin/env python3
"""
VR control for XLerobot robot
Uses handle_vr_input with delta action control
"""

# Standard library imports
import os
import time
import asyncio
import logging
import threading
import traceback
from scipy.spatial.transform import Rotation
import numpy as np

from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.robot_utils import precise_sleep
from lerobot.robots.xlerobot_pinc import XLerobotPincConfig, XLerobotPinc
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
URDF_PATH = os.getenv("XLEROBOT_URDF_PATH", "/home/that/ambient_urdf/robot.urdf")

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

HEAD_JOINT_MAP = {
    "head_pan": "head_pan",
    "head_tilt": "head_tilt",
}

FULL_START_POS = {
    "left_arm_shoulder_pan": 0.0,
    "left_arm_shoulder_lift": -90.0,
    "left_arm_elbow_flex": 45.0,
    "left_arm_elbow_roll": 0.0,
    "left_arm_wrist_flex": 45.0,
    "left_arm_wrist_roll": 0.0,
    "left_arm_gripper": 0.0,
    "right_arm_shoulder_pan": 0.0,
    "right_arm_shoulder_lift": -90.0,
    "right_arm_elbow_flex": 45.0,
    "right_arm_elbow_roll": 0.0,
    "right_arm_wrist_flex": 45.0,
    "right_arm_wrist_roll": 0.0,
    "right_arm_gripper": 0.0,
    "head_pan": 0.0,
    "head_tilt": 25.0,
}

# --- Minimal flags to disable one whole hand and head teleop ---
# Set to False to disable left-hand teleop and head teleop.
# Re-enable by setting to True.
ENABLE_HEAD = False
ENABLE_BASE = False

TASK_DESCRIPTION = "Do something"
HF_REPO_ID = "xuweiwu/do-something"
NUM_EPISODES = 50
EPISODE_TIME_SEC = 150

FPS = 50

class SimpleTeleopArm:
    def __init__(self, joint_map, initial_obs, prefix="left", kp=0.8):
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
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        self.target_positions = self.start_pos.copy()

        # Delta control state variables for VR input
        self.vr_relative_position_scaling = 1.2
        
        self.max_pos_vel = 0.4
        self.max_rad_vel = 150 * (np.pi/180)
        # self.gripper_vel_step = 1
        self.gripper_clip_min = 0
        self.gripper_clip_max = 100

        self.vr_calibrated = False
        self.vr_headset_to_base = Rotation.from_matrix(np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float))
        self.vr_ctrl_to_ee = None

        joint_names_wo_gripper = [j for j in self.target_positions if j != 'gripper']
        self.kinematics= RobotKinematics(
            urdf_path=URDF_PATH,
            target_frame_name="gripper_frame_link",
            joint_names=joint_names_wo_gripper,
        )
        self.kinematics.solver.dt = 1.0/FPS

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
                    max_ee_step_m=0.03,
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

    def move_to_target_with_ipol(self, robot, target_positions=None, duration=5.0, control_freq=200.0,
        max_vel_per_joint=None, max_acc_per_joint=None, max_dev_per_joint=None):
        """
        Plan a quadratic-spline trajectory from current q to zero/init pos and execute it
        at fixed control frequency. Finishes exactly at `duration` if feasible.

        Raises:
            ValueError if requested duration is infeasible given the limits.
        """
        # 0) define target order explicitly via target_positions (canonical order)
        if target_positions is None:
            target_positions = self.home_pos.copy()
        
        # 1) Read current joint positions (calibrated if provided)
        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        obs = {j: obs_raw[f"{self.joint_map[j]}"] for j in self.joint_map}
        print(f"[{self.prefix.capitalize()} ARM IPOL] Current joint positions: {obs}")

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            print(f"[{self.prefix.capitalize()} ARM IPOL] Warning: skipping joints missing in observation: {missing}")

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
            max_vel_per_joint = np.full(J, 15)   # deg/s (pick something reasonable)
        if max_acc_per_joint is None:
            max_acc_per_joint = np.full(J, 30)   # deg/s^2
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
        print(f"[{self.prefix.capitalize()} ARM IPOL] Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
            f"planned duration ≈ {t[-1]:.3f}s (requested {duration:.3f}s)")
        
        t0 = time.perf_counter()
        next_tick = t0
        for k_step in range(len(t)):
            
            if self.prefix=="left":
                obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
            else:
                obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
            obs = {j: obs_raw[f"{self.joint_map[j]}"] for j in self.joint_map}

            q_meas = np.array([float(obs[n]) for n in names], dtype=float)
            q_ref = np.array([q[k_step, i] for i, n in enumerate(names)], dtype=float)
            q_cmd = q_meas + self.kp*(q_ref - q_meas)
            action = {f"{self.joint_map[j]}.pos": q_cmd[i] for i, j in enumerate(names)}
            robot.send_action(action)

            # sleep to maintain control_freq (best-effort wall clock pacing)
            next_tick += dt
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(next_tick - now)
        
        print(f"[{self.prefix.capitalize()} ARM IPOL] Reached end of ipol trajectory.")
        self.target_positions = target_positions.copy()
        print(f"[{self.prefix.capitalize()} ARM IPOL] Reset target positions to: {self.target_positions}")
        self.ee_relative_to_robot_joints_processor.reset()
        self.ref_action_when_disabled = None

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
        print(f"[{self.prefix.capitalize()} ARM TELEOP] Current joint positions: {obs}")
        
        if mode_val == ControlMode.RESET.value:
            q_now = []
            for n in obs:
                if n != 'gripper.pos':
                    v = float(obs[n])
                    q_now.append(v)
            q_now  = np.array(q_now,  dtype=float)
            ee_frame = self.kinematics.forward_kinematics(q_now)
            R_ee = Rotation.from_matrix(ee_frame[:3, :3])
            R_vr = vr_goal.vr_ctrl_rotation
            R_vr_to_base = self.vr_headset_to_base * R_vr
            self.vr_ctrl_to_ee =  R_ee.inv() * R_vr_to_base
            self.vr_calibrated = True
            print(f"[{self.prefix.capitalize()} ARM TELEOP] VR reset")
            return
            
        # Extract VR control data
        # Get VR relative pose changes
        vr_relative_position = vr_goal.relative_position # [x, y, z] in meters in vr frame
        vr_relative_rotvec = vr_goal.relative_rotvec # [wx, wy, wz] in radian in vr frame

        # Avoid commanding small motions
        relative_position_norm = float(np.linalg.norm(vr_relative_position))
        relative_rotvec_norm = float(np.linalg.norm(vr_relative_rotvec))
        if relative_position_norm < 0.001 and relative_rotvec_norm < 0.005:
            vr_relative_position = np.array([0, 0, 0])
            vr_relative_rotvec = np.array([0, 0, 0])
        
        # Transform vr relative position to robot frame
        # robot | vr
        # x     | -z
        # y     | -x
        # z     |  y
        delta_x = -vr_relative_position[2] * self.vr_relative_position_scaling
        delta_y = -vr_relative_position[0] * self.vr_relative_position_scaling
        delta_z = vr_relative_position[1] * self.vr_relative_position_scaling

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
        threshold = 0.005
        if abs(delta_x) > threshold*delta_pos_limit:
            target_action["target_x"] = delta_x
            ee_pose_changed = True
        if abs(delta_y) > threshold*delta_pos_limit:
            target_action["target_y"] = delta_y
            ee_pose_changed = True
        if abs(delta_z) > threshold*delta_pos_limit:
            target_action["target_z"] = delta_z
            ee_pose_changed = True
        if abs(delta_wx) > threshold*delta_rotvec_limit:
            target_action["target_wx"] = delta_wx
            ee_pose_changed = True
        if abs(delta_wy) > threshold*delta_rotvec_limit:
            target_action["target_wy"] = delta_wy
            ee_pose_changed = True
        if abs(delta_wz) > threshold*delta_rotvec_limit:
            target_action["target_wz"] = delta_wz
            ee_pose_changed = True

        # Handle gripper state directly
        trigger = getattr(vr_goal, "trigger", None)
        if trigger is not None:
            gripper_pos_des = self.gripper_clip_min + (self.gripper_clip_max - self.gripper_clip_min) * trigger
            gripper_vel_des = (gripper_pos_des - obs["gripper.pos"]) / FPS # this means we expect to achieve gripper_pos_des in 1 sec.
            target_action["gripper_vel"] = gripper_vel_des

        print(f"[{self.prefix.capitalize()} ARM TELEOP] Relative ee actions: {target_action}")
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
        print(f"[{self.prefix.capitalize()} ARM TELEOP] Ref joint positions: {self.target_positions}")
    
    def p_control_action(self, robot):
        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)

        obs = {}
        obs_no_prefix = {}
        for j in self.joint_map:
            joint_name = self.joint_map[j]
            raw_value = obs_raw[joint_name]
            obs[f"{joint_name}.pos"] = raw_value
            obs_no_prefix[j] = raw_value
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - obs_no_prefix[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = obs_no_prefix[j] + control
        
        print(f"[{self.prefix.capitalize()} ARM CONTROL] Commanded actions: {action}")
        return action, obs

class SimpleHeadControl:
    def __init__(self, joint_map, kp=0.8):
        self.joint_map = joint_map
        self.kp = kp
        self.degree_step = 0.4
        self.start_pos = {k: FULL_START_POS[v] for k, v in self.joint_map.items()}
        self.home_pos = {"head_pan": 0.0, "head_tilt": 0.0}
        self.target_positions = self.start_pos.copy()

    def move_to_target_with_ipol(self, robot, target_positions=None, duration=5.0, control_freq=200.0,
        max_vel_per_joint=None, max_acc_per_joint=None, max_dev_per_joint=None):
        """
        Plan a quadratic-spline trajectory from current q to zero/init pos and execute it
        at fixed control frequency. Finishes exactly at `duration` if feasible.

        Raises:
            ValueError if requested duration is infeasible given the limits.
        """
        # 0) define target order explicitly via target_positions (canonical order)
        if target_positions is None:
            target_positions = self.home_pos.copy()
        
        # 1) Read current joint positions (calibrated if provided)
        obs = robot.bus_right_head.sync_read("Present_Position", robot.head_motors)
        print(f"[HEAD IPOL] Current joint positions: {obs}")

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            print(f"[HEAD IPOL] Warning: skipping joints missing in observation: {missing}")

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
            max_vel_per_joint = np.full(J, 15)   # deg/s (pick something reasonable)
        if max_acc_per_joint is None:
            max_acc_per_joint = np.full(J, 30)   # deg/s^2
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
        print(f"[HEAD IPOL] Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
            f"planned duration ≈ {t[-1]:.3f}s (requested {duration:.3f}s)")
        
        t0 = time.perf_counter()
        next_tick = t0
        for k_step in range(len(t)):
            
            obs = robot.bus_right_head.sync_read("Present_Position", robot.head_motors)
            q_meas = np.array([float(obs[n]) for n in names], dtype=float)
            q_ref = np.array([q[k_step, i] for i, n in enumerate(names)], dtype=float)
            q_cmd = q_meas + self.kp*(q_ref- q_meas)
            action = {f"{n}.pos": q_cmd[i] for i, n in enumerate(names)}
            robot.send_action(action)

            # sleep to maintain control_freq (best-effort wall clock pacing)
            next_tick += dt
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(next_tick - now)

        self.target_positions = target_positions.copy()
        print("[HEAD IPOL] Reached end of ipol trajectory.")
        print(f"[HEAD IPOL] Reset head target positions to: {self.target_positions}")

    def handle_vr_input(self, vr_goal):
        if vr_goal is None:
            return
        
        mode = getattr(vr_goal, "mode", ControlMode.IDLE)
        mode_val = getattr(mode, "value", mode)
        if mode_val == ControlMode.IDLE.value:
            return
        
        # Map VR input to head motor targets
        thumb = getattr(vr_goal, "thumbstick", None)
        if thumb is not None:
            thumb_x = thumb.get('x', 0)
            thumb_y = thumb.get('y', 0)
            if abs(thumb_x) > 0.5:
                if thumb_x > 0:
                    self.target_positions["head_pan"] += self.degree_step
                else:
                    self.target_positions["head_pan"] -= self.degree_step
            if abs(thumb_y) > 0.5:
                if thumb_y > 0:
                    self.target_positions["head_tilt"] += self.degree_step
                else:
                    self.target_positions["head_tilt"] -= self.degree_step
        print(f"[HEAD TELEOP] head_pan: {self.target_positions['head_pan']}")
        print(f"[HEAD TELEOP] head_tilt: {self.target_positions['head_tilt']}")

    def p_control_action(self, robot):
        obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.head_motors)
        obs = {f"{j}.pos": obs_raw[self.joint_map[j]] for j in self.joint_map}
        action = {}
        for motor in self.target_positions:
            current_obs = obs_raw.get(f"{self.joint_map[motor]}", 0.0)
            error = self.target_positions[motor] - current_obs
            control = self.kp * error
            action[f"{self.joint_map[motor]}.pos"] = current_obs + control
        print(f"[HEAD CONTROL] commanded actions: {action}")
        return action, obs

def move_to_target_full_body_with_ipol(
        robot, left_teleop, right_teleop, head_teleop=None, 
        target_positions=None, duration=5.0, control_freq=200.0, kp=0.8,
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
            head_target_pos = {v: head_teleop.home_pos[k] for k, v in HEAD_JOINT_MAP.items()} if head_teleop else {}
            target_positions = {**left_target_pos, **right_target_pos, **head_target_pos}
        
        # 1) Read current joint positions
        left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        head_motors = robot.head_motors if head_teleop else []
        right_head_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors+head_motors)
        obs = {**left_obs, **right_head_obs}
        print(f"[FULL-BODY IPOL] Current joint positions: {obs}")

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            print(f"[FULL-BODY IPOL] Warning: skipping joints missing in observation: {missing}")

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
            max_vel_per_joint = np.full(J, 15)   # deg/s (pick something reasonable)
        if max_acc_per_joint is None:
            max_acc_per_joint = np.full(J, 30)   # deg/s^2
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
        print(f"[FULL-BODY IPOL] Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
            f"planned duration ≈ {t[-1]:.3f}s (requested {duration:.3f}s)")
        
        t0 = time.perf_counter()
        next_tick = t0
        for k_step in range(len(t)):
            
            left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
            right_head_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors+head_motors)
            obs = {**left_obs, **right_head_obs}

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
        
        print("[FULL-BODY IPOL] Reached end of ipol trajectory.")
        for k, v in LEFT_JOINT_MAP.items():
            left_teleop.target_positions[k] = target_positions[v]
        print(f"[FULL-BODY IPOL] Reset left arm target positions to: {left_teleop.target_positions}")
        for k, v in RIGHT_JOINT_MAP.items():
            right_teleop.target_positions[k] = target_positions[v]
        print(f"[FULL-BODY IPOL] Reset right arm target positions to: {right_teleop.target_positions}")
        if head_teleop:
            for k, v in HEAD_JOINT_MAP.items():
                head_teleop.target_positions[k] = target_positions[v]
            print(f"[FULL-BODY IPOL] Reset head target positions to: {head_teleop.target_positions}")

        left_teleop.ee_relative_to_robot_joints_processor.reset()
        left_teleop.ref_action_when_disabled = None
        right_teleop.ee_relative_to_robot_joints_processor.reset()
        right_teleop.ref_action_when_disabled = None

def get_vr_base_action(robot, vr_goal):
    if not robot.has_mobile_platform:
        return {}

    if vr_goal is None:
        return {}
    
    mode = getattr(vr_goal, "mode", ControlMode.IDLE)
    mode_val = getattr(mode, "value", mode)
    if mode_val == ControlMode.IDLE.value:
        return {}
    
    # Build key set based on VR input (you can customize this mapping)
    pressed_keys = set()
    
    # Example VR to base movement mapping - adjust according to your VR system
    # You may need to customize these mappings based on your VR controller buttons
    thumb = getattr(vr_goal, "thumbstick", None)
    buttons = getattr(vr_goal, "buttons", None)
    if thumb is not None:
        thumb_x = thumb.get('x', 0)
        thumb_y = thumb.get('y', 0)
        if abs(thumb_x) > 0.5:
            if thumb_x > 0:
                pressed_keys.add('6')  # Move right
            else:
                pressed_keys.add('4')  # Move left
        if abs(thumb_y) > 0.5:
            if thumb_y > 0:
                pressed_keys.add('2')  # Move backward
            else:
                pressed_keys.add('8')  # Move forward
    if buttons is not None:
        button_x = buttons.get('X', 0)
        button_y = buttons.get('Y', 0)
        if button_x:
            pressed_keys.add('7') # Turn left
        if button_y:
            pressed_keys.add('9') # Turn right
    
    # Convert to numpy array and get base action
    keyboard_keys = np.array(list(pressed_keys))
    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
    
    return base_action

def main():
    print("XLerobot Pro VR Control Example")
    print("="*50)

    shutdown_event = threading.Event()
    # saving_in_progress_event = threading.Event()

    robot, vr_monitor, head_teleop = None, None, None
    robot_name = "ambient_xlerobot_pinc"
    try:
        robot_config = XLerobotPincConfig(
            id=robot_name,
            use_degrees=True,
        )
        robot = XLerobotPinc(robot_config)
        robot.connect()
        print("[MAIN] Robot connection successful!")
        print(f"[MAIN] Motor bus_left_base info: {robot.bus_left_base.motors}")
        print(f"[MAIN] Motor bus_right_head info: {robot.bus_right_head.motors}") 
        print(f"[MAIN] Mobile platform enabled: {robot.has_mobile_platform}")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(f"[MAIN] Robot config: {robot_config}")
        print(f"[MAIN] Robot: {robot}")
        traceback.print_exc()
        return
    
    continue_choice = input("Do you want to continue to teleop the robot? (y/n, [default y]): ").strip().lower()
    if continue_choice in ['n', 'no']:
        print(f"[MAIN] User decided not to continue")
        left_bus_pos = robot.bus_left_base.sync_read("Present_Position", normalize=False)
        right_head_pos = robot.bus_right_head.sync_read("Present_Position", normalize=False)
        print(f"[MAIN] Left bus states: {left_bus_pos}")
        print(f"[MAIN] Right head states: {right_head_pos}")
        return
    
    try:
        # Init the arm and head instances
        print("🔧 Moving robot to start pose...")
        obs = robot.get_observation()
        left_arm_teleop = SimpleTeleopArm(LEFT_JOINT_MAP, obs, prefix="left")
        right_arm_teleop = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, prefix="right")
        if ENABLE_HEAD:
            head_teleop = SimpleHeadControl(HEAD_JOINT_MAP)
        # Move both arms and head to zero position at start
        move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop, target_positions=FULL_START_POS)
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
                head_action, _ = head_teleop.p_control_action(robot) if (ENABLE_HEAD and head_teleop) else ({}, {})
                base_action = {}

                action = {**left_action, **right_action, **head_action, **base_action}
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
                        move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop, target_positions=FULL_START_POS)
                        print("User moved all joints to start pose by pressing right button X")
            if shutdown_event.is_set():
                print("Shutdown event triggered")
                break

            camera_obs = robot.get_camera_observation()

            # Debug prints (optional)
            if left_goal:
                print(f"Left goal: {left_goal}")
            if right_goal:
                print(f"Right goal: {right_goal}")

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

            # HEAD: you were using right controller; pick latest right goal
            if ENABLE_HEAD and head_teleop:
                head_goal = right_motion or right_reset
                if head_goal is not None:
                    head_teleop.handle_vr_input(head_goal)

            # Compute actions from both arms and head ----
            left_action, left_obs = left_arm_teleop.p_control_action(robot)
            right_action, right_obs = right_arm_teleop.p_control_action(robot)
            head_action, head_obs = head_teleop.p_control_action(robot) if (ENABLE_HEAD and head_teleop) else ({}, {})

            # Base control from LEFT controller ----
            # Use the latest left goal (motion preferred, else reset)
            if ENABLE_BASE and robot.has_mobile_platform:
                base_source_goal = left_motion or left_reset
                base_action = get_vr_base_action(robot, base_source_goal)
            else:
                base_action = {}

            # Merge all actions
            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = {**left_obs, **right_obs, **head_obs, **camera_obs}

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
            move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop)
            robot.disconnect()

if __name__ == "__main__":
    main()
