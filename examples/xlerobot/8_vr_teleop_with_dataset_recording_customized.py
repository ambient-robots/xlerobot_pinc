#!/usr/bin/env python3
"""
VR control for XLerobot robot
Uses handle_vr_input with delta action control
"""

# Standard library imports
import asyncio
import logging
import math
import sys
import threading
import time
import traceback
import queue
from scipy.spatial.transform import Rotation

# Third-party imports
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Local imports
from XLeVR.vr_monitor import VRMonitor
from XLeVR.xlevr.inputs.base import ControlGoal, ControlMode
from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
from lerobot.utils.robot_utils import precise_sleep
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    robot_action_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEReferenceAndDelta,
    EEBoundsAndSafety,
    GripperVelocityToJoint,
    ForwardKinematicsJointsToEE,
    InverseKinematicsEEToJoints,
)
from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.visualization_utils import log_rerun_data, init_rerun

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Joint mapping configurations
LEFT_JOINT_MAP = {
    "shoulder_pan": "left_arm_shoulder_pan",
    "shoulder_lift": "left_arm_shoulder_lift",
    "elbow_flex": "left_arm_elbow_flex",
    "wrist_flex": "left_arm_wrist_flex",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}

RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
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
    "left_arm_wrist_flex": 45.0,
    "left_arm_wrist_roll": 0.0,
    "left_arm_gripper": 50.0,
    "right_arm_shoulder_pan": 0.0,
    "right_arm_shoulder_lift": -90.0,
    "right_arm_elbow_flex": 45.0,
    "right_arm_wrist_flex": 45.0,
    "right_arm_wrist_roll": 0.0,
    "right_arm_gripper": 50.0,
    "head_pan": 0.0,
    "head_tilt": 25.0,
}

# --- Minimal flags to disable one whole hand and head teleop ---
# Set to False to disable left-hand teleop and head teleop.
# Re-enable by setting to True.
ENABLE_LEFT_HAND = True
ENABLE_HEAD = True
ENABLE_BASE = False

EPISODE_LEN = 450  # Number of steps per episode
RESET_LEN = 150
NR_OF_EPISODES = 110
TASK = "Grab the cup"
MAIN_CAMERA_INDEX = "/dev/ttyACM0" 
#RIGHT_ARM_CAMERA_INDEX = "/dev/video2"
LEFT_ARM_CAMERA_INDEX = "/dev/video2"
DATASET_REPO = "Grigorij/XLeRobot_arms_5"
FPS = 50

class SimpleTeleopArm:
    """
    A class for controlling a robot arm using VR input with delta action control.
    
    This class provides inverse kinematics-based arm control with proportional control
    for smooth movement and gripper operations based on VR controller input.
    """
    
    def __init__(self, joint_map, initial_obs, prefix="right", kp=0.75):
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp

        # Set target positions to zero for P control
        self.target_positions = {k: FULL_START_POS[v] for k, v in self.joint_map.items()}

        # Initial joint positions
        self.home_pos = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }

        joint_names_wo_gripper = [j for j in self.target_positions if j != 'gripper']
        self.kinematics= RobotKinematics(
            urdf_path="/home/that/SO-ARM100/Simulation/SO101/so101_new_calib.urdf", 
            target_frame_name="gripper_frame_link",
            joint_names=joint_names_wo_gripper,
        )
        self.ee_relative_to_robot_joints_processor = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
            [   
                EEReferenceAndDelta(
                    kinematics=self.kinematics,
                    end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
                    motor_names=list(self.target_positions.keys()),
                    use_latched_reference=False,
                ),
                EEBoundsAndSafety(
                    end_effector_bounds={"min": [-0.5, -0.5, -0.5], "max": [0.5, 0.5, 0.5]},
                    max_ee_step_m=0.03,
                ),
                GripperVelocityToJoint(
                    speed_factor=5.0,
                    clip_max=50,
                ),
                InverseKinematicsEEToJoints(
                    kinematics=self.kinematics,
                    motor_names=list(self.target_positions.keys()),
                    weights={"position": 1.0, "orientation": 0.1},
                    initial_guess_current_joints=False,
                ),
            ],
            to_transition=robot_action_observation_to_transition,
            to_output=transition_to_robot_action,
        )
        self.ref_action_when_disabled = None
        
        # Delta control state variables for VR input
        self.vr_relative_position_scaling = 1.1
        self.gripper_vel_step = 1

        self.vr_calibrated = False
        self.vr_headset_to_base = Rotation.from_matrix(np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float))
        self.vr_ctrl_to_ee = None

    def move_to_target_with_ipol(self, robot, target_positions=None, duration=3.0, control_freq=200.0,
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

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            print(f"[ipol] Warning: skipping joints missing in observation: {missing}")
        print(f"motor names in obs: {names}")

        # 3) build current/goal vectors in that SAME order (no sorting)
        q_now = []
        q_goal = []
        for n in names:
            v = float(obs[n])
            q_now.append(v)
            q_goal.append(float(target_positions[n]))
        q_now  = np.array(q_now,  dtype=float)
        q_goal = np.array(q_goal, dtype=float)
        print(f"Current pos: {q_now}")

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
        print(f"Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
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
        
        self.target_positions = target_positions.copy()
        print(f"ipol resets {self.prefix} arm target positions to: {self.target_positions}")

        self.ee_relative_to_robot_joints_processor.reset()
        self.ref_action_when_disabled = None
        print("Reached target pos of with ipol trajectory.")

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
        current_obs = {f"{j}.pos": obs_raw[f"{self.joint_map[j]}"] for j in self.joint_map}
        
        if mode_val == ControlMode.RESET.value:
            q_now = []
            for n in current_obs:
                if n != 'gripper.pos':
                    v = float(current_obs[n])
                    q_now.append(v)
            q_now  = np.array(q_now,  dtype=float)
            ee_frame = self.kinematics.forward_kinematics(q_now)
            R_ee = Rotation.from_matrix(ee_frame[:3, :3])
            R_vr = vr_goal.vr_ctrl_rotation
            R_vr_to_base = self.vr_headset_to_base * R_vr
            self.vr_ctrl_to_ee =  R_ee.inv() * R_vr_to_base
            self.vr_calibrated = True
            print("REST EXECUTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
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
        
        delta_pos_limit = 0.003  # Maximum delta per update (meters)
        delta_rotvec_limit = 3*(np.pi/180)  # Maximum angle delta per update (radians)
        
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

        grip_moved = False
        if abs(delta_x) > 0.1*delta_pos_limit:
            target_action["target_x"] = delta_x
            grip_moved = True
        if abs(delta_y) > 0.1*delta_pos_limit:
            target_action["target_y"] = delta_y
            grip_moved = True
        if abs(delta_z) > 0.1*delta_pos_limit:
            target_action["target_z"] = delta_z
            grip_moved = True
        if abs(delta_wx) > 0.1*delta_rotvec_limit:
            target_action["target_wx"] = delta_wx
            grip_moved = True
        if abs(delta_wy) > 0.1*delta_rotvec_limit:
            target_action["target_wy"] = delta_wy
            grip_moved = True
        if abs(delta_wz) > 0.1*delta_rotvec_limit:
            target_action["target_wz"] = delta_wz
            grip_moved = True

        # Handle gripper state directly
        trigger = getattr(vr_goal, "trigger", None)
        if trigger is not None:
            if trigger > 0.5:
                # close gripper when triggerdown
                target_action["gripper_vel"] = -self.gripper_vel_step
            else:
                # open gripper when triggerup
                target_action["gripper_vel"] = self.gripper_vel_step

        print(f"{self.prefix}_arm relative actions: {target_action}")
        desired_action = self.ee_relative_to_robot_joints_processor((target_action, current_obs))
        if not grip_moved:
            if self.ref_action_when_disabled is None:
                self.ref_action_when_disabled = current_obs.copy()
            ref_action = self.ref_action_when_disabled.copy()
            ref_action['gripper.pos'] = desired_action['gripper.pos']   
        else:
            ref_action = desired_action.copy()
        self.ref_action_when_disabled = ref_action.copy()
        
        for key, ref_pos in ref_action.items():
            self.target_positions[key.removesuffix('.pos')] = ref_pos
        # print(f"{self.prefix}_arm ref positions: {self.target_positions}")
    
    def p_control_action(self, robot):
        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        current_obs = {j: obs_raw[f"{self.joint_map[j]}"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current_obs[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current_obs[j] + control
        
        # print(f"{self.prefix}_arm commanded actions: {action}")
        return action


class SimpleHeadControl:
    """
    A class for controlling robot head motors using VR thumbstick input.
    
    Provides simple head movement control with proportional control for smooth operation.
    """
    
    def __init__(self, joint_map, kp=0.5):
        self.joint_map = joint_map
        self.kp = kp
        self.degree_step = 0.4
        self.target_positions = {k: FULL_START_POS[v] for k, v in self.joint_map.items()}
        self.home_pos = {"head_pan": 0.0, "head_tilt": 0.0}

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

    def move_to_target_with_ipol(self, robot, target_positions=None, duration=3.0, control_freq=200.0,
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

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            print(f"[ipol] Warning: skipping joints missing in observation: {missing}")
        print(f"motor names in obs: {names}")

        # 3) build current/goal vectors in that SAME order (no sorting)
        q_now = []
        q_goal = []
        for n in names:
            v = float(obs[n])
            q_now.append(v)
            q_goal.append(float(target_positions[n]))
        q_now  = np.array(q_now,  dtype=float)
        q_goal = np.array(q_goal, dtype=float)
        print(f"Current pos: {q_now}")

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
        print(f"Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
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
        print(f"ipol resets head target positions to: {self.target_positions}")
        print("Reached zero pos of with ipol trajectory.")

    def p_control_action(self, robot):
        obs = robot.bus_right_head.sync_read("Present_Position", robot.head_motors)
        action = {}
        for motor in self.target_positions:
            current_obs = obs.get(f"{self.joint_map[motor]}", 0.0)
            error = self.target_positions[motor] - current_obs
            control = self.kp * error
            action[f"{self.joint_map[motor]}.pos"] = current_obs + control
        # print(f"head commanded actions: {action}")
        return action

def move_to_target_full_body_with_ipol(
        robot, left_teleop, right_teleop, head_teleop, 
        target_positions=None, duration=3.0, control_freq=200.0, kp=0.5,
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
            head_target_pos = {v: head_teleop.home_pos[k] for k, v in HEAD_JOINT_MAP.items()}
            target_positions = {**left_target_pos, **right_target_pos, **head_target_pos}
        
        # 1) Read current joint positions
        left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        right_head_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors+robot.head_motors)
        obs = {**left_obs, **right_head_obs}
        print(f"current pos: {obs}")

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            print(f"[ipol] Warning: skipping joints missing in observation: {missing}")
        print(f"motor names in obs: {names}")

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
        print(f"Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
            f"planned duration ≈ {t[-1]:.3f}s (requested {duration:.3f}s)")
        
        t0 = time.perf_counter()
        next_tick = t0
        for k_step in range(len(t)):
            
            left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
            right_head_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors+robot.head_motors)
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
        
        for k, v in LEFT_JOINT_MAP.items():
            left_teleop.target_positions[k] = target_positions[v]
        print(f"ipol resets left arm target positions to: {left_teleop.target_positions}")
        for k, v in RIGHT_JOINT_MAP.items():
            right_teleop.target_positions[k] = target_positions[v]
        print(f"ipol resets right arm target positions to: {right_teleop.target_positions}")
        for k, v in HEAD_JOINT_MAP.items():
            head_teleop.target_positions[k] = target_positions[v]
        print(f"ipol resets head target positions to: {head_teleop.target_positions}")

        left_teleop.ee_relative_to_robot_joints_processor.reset()
        left_teleop.ref_action_when_disabled = None
        right_teleop.ee_relative_to_robot_joints_processor.reset()
        right_teleop.ref_action_when_disabled = None
        print("Reached target pos of full body with ipol trajectory.")

def get_vr_base_action(robot, vr_goal):
    """
    Get base control commands from VR input.
    
    Args:
        vr_goal: VR controller goal data containing metadata
        robot: Robot instance for action conversion
        
    Returns:
        dict: Base movement actions based on VR thumbstick input
    """
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


def init_dataset():
    features = {
        "action": {
            "dtype": "float32",
            "shape": (6,),
            "names": [
                #"left_arm_shoulder_pan.pos", "left_arm_shoulder_lift.pos", "left_arm_elbow_flex.pos", 
                #"left_arm_wrist_flex.pos", "left_arm_wrist_roll.pos", "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos", "right_arm_shoulder_lift.pos", "right_arm_elbow_flex.pos", 
                "right_arm_wrist_flex.pos", "right_arm_wrist_roll.pos", "right_arm_gripper.pos",]
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (6,),
            "names": [
                #"left_arm_shoulder_pan.pos", "left_arm_shoulder_lift.pos", "left_arm_elbow_flex.pos", 
                #"left_arm_wrist_flex.pos", "left_arm_wrist_roll.pos", "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos", "right_arm_shoulder_lift.pos", "right_arm_elbow_flex.pos", 
                "right_arm_wrist_flex.pos", "right_arm_wrist_roll.pos", "right_arm_gripper.pos",]
        },
        "observation.images.main": {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        },
        # "observation.images.right_arm": {
        #     "dtype": "video",
        #     "shape": (480, 640, 3),
        #     "names": ["height", "width", "channel"],
        # },
        "observation.images.left_arm": {
            "dtype": "video",
            "shape": (480, 640, 3),
            "names": ["height", "width", "channel"],
        },
        "timestamp": {
            "dtype": "float32",
            "shape": (1,),
            "names": None
        },
    }
    import time
    dataset = LeRobotDataset.create(
        repo_id=DATASET_REPO,
        root=f"my_dataset_{time.time()}",
        features=features,
        fps=FPS,
        image_writer_processes=0,
        image_writer_threads=4,
    )
    return dataset



def saving_dataset_worker(frame_queue, shutdown_event, saving_in_progress_event):
    """
    Worker function to save dataset frames in the background.
    
    Continuously checks for new dataset frames and saves them to disk.
    """
    try:
        dataset = init_dataset()
        # save videos into separate files to avoid concatenation problems
        dataset.meta.update_chunk_settings(video_files_size_in_mb=0.001)
        recording_dataset = True
        frame_nr = 0
        episode = 0
        while not shutdown_event.is_set():
            try:
                lerobot_frame = frame_queue.get(timeout=1)
            except queue.Empty:
                continue
    
            if recording_dataset:
                dataset.add_frame(lerobot_frame)
                print(f"step {frame_nr} added to dataset")

            frame_nr += 1

            if frame_nr == EPISODE_LEN:
                print(f"Finishing episode {episode}, reset...")
                saving_in_progress_event.set()
                dataset.save_episode()
                dataset.image_writer.wait_until_done()
                #recording_dataset = False
                saving_in_progress_event.clear()
                #recording_dataset = True
                frame_nr = 0
                episode += 1
                if episode >= NR_OF_EPISODES:
                    print("Reached maximum number of episodes. Exiting...")
                    shutdown_event.set()
                    break
                print(f"Starting episode {episode} recording...")
   
    except Exception as e:
        logger.error(f"Error in dataset saving worker: {e}", exc_info=True)
    finally:
        if dataset:
            dataset.image_writer.wait_until_done()
            dataset.save_episode()
            dataset.push_to_hub()

def main():
    """
    Main function for VR teleoperation of XLerobot.
    
    Initializes the robot connection, VR monitoring, and runs the main control loop
    for dual-arm robot control with VR input.
    """
    print("XLerobot VR Control Example")
    print("="*50)

    shutdown_event = threading.Event()
    # saving_in_progress_event = threading.Event()

    robot, vr_monitor, camera_main, camera_left, dataset_saving_thread = None, None, None, None, None

    robot_name = "ambient_xlerobot"
    try:
        robot_config = XLerobotConfig(id=robot_name, use_degrees=True)
        robot = XLerobot(robot_config)
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
        if robot.is_calibrated:
            print(f"[MAIN] Robot is calibrated and ready to use!")
            print(f"[MAIN] Motor bus_left_base info: {robot.bus_left_base.motors}")
            print(f"[MAIN] Motor bus_right_head info: {robot.bus_right_head.motors}")
        else:
            print(f"[MAIN] Robot requires calibration")
        
        continue_choice = input("Do you want to continue to teleop the robot? (y/n, [default y]): ").strip().lower()
        if continue_choice in ['n', 'no']:
            print(f"[MAIN] User decided not to continue")
            return
        
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(f"[MAIN] Robot config: {robot_config}")
        print(f"[MAIN] Robot: {robot}")
        return
    
    try:
        # Init the arm and head instances
        print("🔧 Moving robot to start pose...")
        obs = robot.get_observation()
        left_arm_teleop = SimpleTeleopArm(LEFT_JOINT_MAP, obs, prefix="left") if ENABLE_LEFT_HAND else None
        right_arm_teleop = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, prefix="right")
        head_teleop = SimpleHeadControl(HEAD_JOINT_MAP) if ENABLE_HEAD else None

        # Move both arms and head to zero position at start
        if ENABLE_LEFT_HAND and ENABLE_HEAD:
            move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop, 
                                        target_positions=FULL_START_POS)
        else:
            right_start_positions = {k: FULL_START_POS[v] for k, v in RIGHT_JOINT_MAP.items()}
            right_arm_teleop.move_to_target_with_ipol(robot, target_positions=right_start_positions)
            if ENABLE_LEFT_HAND and left_arm_teleop:
                left_start_positions = {k: FULL_START_POS[v] for k, v in LEFT_JOINT_MAP.items()}
                left_arm_teleop.move_to_target_with_ipol(robot, target_positions=left_start_positions)
            if ENABLE_HEAD and head_teleop:
                head_start_positions = {k: FULL_START_POS[v] for k, v in HEAD_JOINT_MAP.items()}
                head_teleop.move_to_target_with_ipol(robot, target_positions=head_start_positions)
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

        # Main VR control loop
        # from lerobot.cameras.opencv import OpenCVCamera
        # from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
		
        # config_main_camera = OpenCVCameraConfig(index_or_path=MAIN_CAMERA_INDEX)
        # #config_right_camera = OpenCVCameraConfig(index_or_path=RIGHT_ARM_CAMERA_INDEX)
        # config_left_camera =  OpenCVCameraConfig(index_or_path=LEFT_ARM_CAMERA_INDEX)
        # camera_main = OpenCVCamera(config_main_camera)
        # #camera_right = OpenCVCamera(config_right_camera) 
        # camera_left = OpenCVCamera(config_left_camera)
        # camera_main.connect()
        # #camera_right.connect()
        # camera_left.connect()

        #init_rerun(session_name="ambient_xlerobot_vr_teleop")

        print("Starting VR control loop.")
        # frame_queue = queue.Queue()
        # thread_args = (frame_queue, shutdown_event, saving_in_progress_event)
        # dataset_saving_thread = threading.Thread(target=saving_dataset_worker, args=thread_args, daemon=False)
        # dataset_saving_thread.start()

        while not shutdown_event.is_set():
            start = time.perf_counter()
            # if saving_in_progress_event.is_set():
            #     print("[MAIN] Waiting for dataset saving to complete...  ", end='\r')
            #     time.sleep(0.1)
            #     continue

            # ---- 1) Fetch ordered goals from queues ----
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
                left_action = left_arm_teleop.p_control_action(robot) if (ENABLE_LEFT_HAND and left_arm_teleop) else {}
                right_action = right_arm_teleop.p_control_action(robot)
                head_action = head_teleop.p_control_action(robot) if (ENABLE_HEAD and head_teleop) else {}
                base_action = {}

                action = {**left_action, **right_action, **head_action, **base_action}
                robot.send_action(action)

                dt_ms = (time.perf_counter() - start) * 1e3
                #print(f"control delay: {dt_ms:.1f}ms")
                busy_wait(max(1.0 / FPS - (time.perf_counter() - start), 0.0))
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

            # Debug prints (optional)
            if left_goal:
                print(f"Left goal: {left_goal}")
            if right_goal:
                print(f"Right goal: {right_goal}")

            # ---- 4) Handle VR input for both arms (RESET first, then motion) ----
            # LEFT ARM
            if ENABLE_LEFT_HAND and left_arm_teleop:
                if left_reset is not None:
                    left_arm_teleop.handle_vr_input(robot, left_reset)   # calibration
                if left_motion is not None:
                    left_arm_teleop.handle_vr_input(robot, left_motion)  # position/orientation

            # RIGHT ARM
            if right_arm_teleop:
                if right_reset is not None:
                    right_arm_teleop.handle_vr_input(robot, right_reset)
                if right_motion is not None:
                    right_arm_teleop.handle_vr_input(robot, right_motion)

            # HEAD: you were using right controller; pick latest right goal
            if ENABLE_HEAD and head_teleop:
                head_goal = right_motion or right_reset
                if head_goal is not None:
                    head_teleop.handle_vr_input(head_goal)

            # ---- 5) Compute actions from both arms and head ----
            left_action = left_arm_teleop.p_control_action(robot) if (ENABLE_LEFT_HAND and left_arm_teleop) else {}
             #print(f"left action: {left_action}")
            right_action = right_arm_teleop.p_control_action(robot)
            #print(f"right action: {right_action}")
            head_action = head_teleop.p_control_action(robot) if (ENABLE_HEAD and head_teleop) else {}
            #print(f"head action: {head_action}")

            # ---- 6) Base control from LEFT controller ----
            # Use the latest left goal (motion preferred, else reset)
            if ENABLE_BASE:
                base_source_goal = left_motion or left_reset
                base_action = get_vr_base_action(robot, base_source_goal)
            else:
                base_action = {}
            #print(f"base action: {base_action}")           

            # Merge all actions
            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            # for k in robot.action_features:
            #     print(f"[MAIN] Observation: {k}: {obs[k]}")
            
            # image_array_main = camera_main.read()
            # #image_array_right = camera_right.read()
            # image_array_left = camera_left.read()
            # #action_values = list(left_action.values())
            # #action_values.extend(right_action.values())
            # action_values = list(right_action.values())
            
            # lerobot_frame = {
            #     'action': np.array(action_values, dtype=np.float32),
            #     'observation.state': np.array(list(robot.get_observation().values())[6:12], dtype=np.float32),
            #     'observation.images.main': image_array_main,
            #     #'observation.images.right_arm': image_array_right,
            #     'observation.images.left_arm': image_array_left,
            #     'task': TASK,
            # }
            # frame_queue.put(lerobot_frame)

            #log_rerun_data(obs, action)

            dt_ms = (time.perf_counter() - start) * 1e3
            # print(f"control delay: {dt_ms:.1f}ms")
            precise_sleep(max(1.0 / FPS - (time.perf_counter() - start), 0.0))
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        
    finally:
        # Cleanup
        shutdown_event.set()

        # if dataset_saving_thread and dataset_saving_thread.is_alive():
        #     print("[MAIN] Waiting for dataset saving thread to finish...")
        #     dataset_saving_thread.join()

        # if camera_main:
        #     camera_main.disconnect()
        # #if camera_right:
        # #    camera_right.disconnect()
        # if camera_left:
        #     camera_left.disconnect()
        if robot:
            if ENABLE_LEFT_HAND and ENABLE_HEAD:
                move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop)
            else:
                right_arm_teleop.move_to_target_with_ipol(robot)
                if ENABLE_LEFT_HAND and left_arm_teleop:
                    left_arm_teleop.move_to_target_with_ipol(robot)
                if ENABLE_HEAD and head_teleop:
                    head_teleop.move_to_target_with_ipol(robot)
            robot.disconnect()


if __name__ == "__main__":
    main()
