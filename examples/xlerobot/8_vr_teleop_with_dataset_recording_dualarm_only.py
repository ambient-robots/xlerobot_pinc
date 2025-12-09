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
    GripperVelocityToJointDeg,
    InverseKinematicsEEToJoints,
)
from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.visualization_utils import log_rerun_data, init_rerun
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.datasets.utils import hw_to_dataset_features, build_dataset_frame

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

FULL_START_POS = {
    "left_arm_shoulder_pan": 0.0,
    "left_arm_shoulder_lift": -90.0,
    "left_arm_elbow_flex": 45.0,
    "left_arm_wrist_flex": 85.0,
    "left_arm_wrist_roll": -90.0,
    "left_arm_gripper": 50.0,
    "right_arm_shoulder_pan": 0.0,
    "right_arm_shoulder_lift": -90.0,
    "right_arm_elbow_flex": 45.0,
    "right_arm_wrist_flex": 85.0,
    "right_arm_wrist_roll": -90.0,
    "right_arm_gripper": 50.0,
}

TASK_DESCRIPTION = "Put the pieces on the table into the box and close the lid"
HF_REPO_ID = "xuweiwu/bimanual-toy-box-cleanup"
FPS = 60
NUM_EPISODES = 50
EPISODE_TIME_SEC = 120

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
        self.vr_relative_position_scaling = 1.2
        self.gripper_vel_step = 1.2

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
                precise_sleep(next_tick - now)
        
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
        
        if self.prefix == "left":
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
        thumb = getattr(vr_goal, "thumbstick", None)
        if thumb is not None:
            thumb_y = thumb.get('y', 0)
            if abs(thumb_y) > 0.25:
                if thumb_y > 0:
                    target_action["gripper_vel"] = self.gripper_vel_step  # Move thumbstick backward to open gripper
                else:
                    target_action["gripper_vel"] = -self.gripper_vel_step  # Move thumbstick forward to close gripper

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

        obs_pos_suffix = {}
        obs_no_prefix = {}
        for j in self.joint_map:
            joint_name = self.joint_map[j]
            raw_value = obs_raw[joint_name]
            obs_pos_suffix[f"{joint_name}.pos"] = raw_value
            obs_no_prefix[j] = raw_value
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - obs_no_prefix[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = obs_no_prefix[j] + control
        
        # print(f"{self.prefix}_arm commanded actions: {action}")
        return action, obs_pos_suffix

class JointIpol:
    def __init__(self, kp=0.5, duration=3.0):
        self.ipol_path = None
        self.ipol_step = 0
        self.num_via_points = 0
        self.target_positions = None
        self.joint_names = FULL_START_POS.keys()
        self.kp = kp
        self.ctrl_freq = FPS
        self.duration = duration

    def plan_to_target(
            self, robot, left_teleop, right_teleop, ctrl_freq=FPS,
            target_positions=None, max_vel_per_joint=None, max_acc_per_joint=None, max_dev_per_joint=None):
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
            self.target_positions = target_positions
            self.ctrl_freq = ctrl_freq
            
            # 1) Read current joint positions
            left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
            right_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
            obs = {**left_obs, **right_obs}
            print(f"current pos: {obs}")

            # 2) build current/goal vectors in that SAME order (no sorting)
            q_now = []
            q_goal = []
            for n in self.joint_names:
                v = float(obs[n])
                q_now.append(v)
                q_goal.append(float(target_positions[n]))
            q_now  = np.array(q_now,  dtype=float)
            q_goal = np.array(q_goal, dtype=float)

            # 3) Limits (defaults if not provided)
            J = q_now.size
            if max_vel_per_joint is None:
                max_vel_per_joint = np.full(J, 15)   # deg/s (pick something reasonable)
            if max_acc_per_joint is None:
                max_acc_per_joint = np.full(J, 30)   # deg/s^2
            if max_dev_per_joint is None:
                # for a 2-via move, deviation isn't essential; keep tiny to retain quadratic plumbing
                max_dev_per_joint = np.full(J, 0.0)

            # 4) Build a 2-via path (current -> goal). You can insert mid vias if you want shaping.
            via = [
                Via(q=q_now,  max_dev=max_dev_per_joint),
                Via(q=q_goal, max_dev=max_dev_per_joint),
            ]
            lim = Limits(max_vel=np.asarray(max_vel_per_joint),
                        max_acc=np.asarray(max_acc_per_joint))

            ipol = QuadraticSplineInterpolator(via, lim)
            ipol.build()  # builds pieces, samples, ds envelope and forward/backward feasible ds(s)
            
            # 5) Slow-down scale so we finish exactly at 'duration'
            # Scaling ds(s) by k scales time as T = T_min / k -> choose k = T_min / duration
            ipol.scale_to_duration(self.duration)

            # 6) Generate time samples and joint references at controller rate
            dt = 1.0/ float(self.ctrl_freq)
            t, q, qd, qdd = ipol.resample(dt)  # will end ~ at `duration`

            self.ipol_path = q.copy()
            self.num_via_points = len(t)
            self.ipol_step = 0

            # 7) Stream to the robot
            print(f"Streaming ipol trajectory: {len(t)} steps at {self.ctrl_freq:.1f} Hz; "
                f"planned duration ≈ {t[-1]:.3f}s (requested {self.duration:.3f}s)")
    
    def get_next_action(self, robot):
        if self.ipol_path is None:
            return {}
        
        left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        right_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        obs = {**left_obs, **right_obs}

        q_meas = np.array([float(obs[n]) for n in self.joint_names], dtype=float)
        q_ref = self.ipol_path[self.ipol_step, :]
        q_cmd = q_meas + self.kp*(q_ref - q_meas)
        action = {f"{j}.pos": q_cmd[i] for i, j in enumerate(self.joint_names)}

        self.ipol_step += 1

        obs_pos_suffix = {f"{k}.pos": v for k, v in obs.items()}
        return action, obs_pos_suffix
    
    def execute_plan(self, robot, left_teleop, right_teleop):
        t0 = time.perf_counter()
        next_tick = t0
        dt = 1.0/ float(self.ctrl_freq)
        while self.ipol_step < self.num_via_points:
            action, _ = self.get_next_action(robot)
            robot.send_action(action)

            # sleep to maintain control_freq (best-effort wall clock pacing)
            next_tick += dt
            
            now = time.perf_counter()
            if now < next_tick:
                precise_sleep(next_tick - now)
        
        self.reset_ipol(left_teleop, right_teleop)
    
    def reset_ipol(self, left_teleop, right_teleop):

        for k, v in LEFT_JOINT_MAP.items():
            left_teleop.target_positions[k] = self.target_positions[v]
        print(f"ipol resets left arm target positions to: {left_teleop.target_positions}")
        for k, v in RIGHT_JOINT_MAP.items():
            right_teleop.target_positions[k] = self.target_positions[v]
        print(f"ipol resets right arm target positions to: {right_teleop.target_positions}")

        self.ipol_path = None
        self.ipol_step = 0
        self.num_via_points = 0
        self.target_positions = None

        left_teleop.ee_relative_to_robot_joints_processor.reset()
        left_teleop.ref_action_when_disabled = None
        right_teleop.ee_relative_to_robot_joints_processor.reset()
        right_teleop.ref_action_when_disabled = None
        print("Reached target pos of full body with ipol trajectory.")


def init_dataset(robot):
    customized_features = {
        "action": {
            "dtype": "float32",
            "shape": (12,),
            "names": [
                "left_arm_shoulder_pan.pos", "left_arm_shoulder_lift.pos", "left_arm_elbow_flex.pos", 
                "left_arm_wrist_flex.pos", "left_arm_wrist_roll.pos", "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos", "right_arm_shoulder_lift.pos", "right_arm_elbow_flex.pos", 
                "right_arm_wrist_flex.pos", "right_arm_wrist_roll.pos", "right_arm_gripper.pos",]
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (12,),
            "names": [
                "left_arm_shoulder_pan.pos", "left_arm_shoulder_lift.pos", "left_arm_elbow_flex.pos", 
                "left_arm_wrist_flex.pos", "left_arm_wrist_roll.pos", "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos", "right_arm_shoulder_lift.pos", "right_arm_elbow_flex.pos", 
                "right_arm_wrist_flex.pos", "right_arm_wrist_roll.pos", "right_arm_gripper.pos",]
        },
    }
    camera_features = hw_to_dataset_features(robot._cameras_ft, OBS_STR)
    dataset_features = {**customized_features, **camera_features}

    from pathlib import Path
    from datetime import datetime
    timestamp_suffix = datetime.now().strftime("%Y-%m-%d-%H-%M")
    root = Path("/home/that/.cache/huggingface/lerobot") / HF_REPO_ID / f"dataset_{timestamp_suffix}"

    dataset = LeRobotDataset.create(
        repo_id=HF_REPO_ID,
        root=str(root),
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        image_writer_processes=10,
        image_writer_threads=5,
    )
    
    return dataset

def main():
    """
    Main function for VR teleoperation of XLerobot.
    
    Initializes the robot connection, VR monitoring, and runs the main control loop
    for dual-arm robot control with VR input.
    """
    print("XLerobot VR Control Example")
    print("="*50)

    robot, vr_monitor = None, None
    robot_name = "ambient_xlerobot"
    try:
        robot_config = XLerobotConfig(id=robot_name, use_degrees=True)
        robot = XLerobot(robot_config)
        robot.connect()
        print(f"[INIT] Successfully connected to robot")
        if robot.is_calibrated:
            print(f"[INIT] Robot is calibrated and ready to use!")
            print(f"[INIT] Motor bus_left_base info: {robot.bus_left_base.motors}")
            print(f"[INIT] Motor bus_right_head info: {robot.bus_right_head.motors}")
        else:
            print(f"[INIT] Robot requires calibration")

    except Exception as e:
        print(f"[INIT] Failed to connect to robot: {e}")
        print(f"[INIT] Robot config: {robot_config}")
        print(f"[INIT] Robot: {robot}")
        traceback.print_exc()
        return
    
    try:
        # Create the dataset
        dataset = init_dataset(robot)

        # Initiailize events
        events = {
            "exit_early": False,       # Left controller right: Exit loop early
            "rerecord_episode": False, # Left controller left: Re-record episode
            "stop_recording": False,   # Left controller up: Stop recording
            "reset_position": False,   # Left controller down: Reset robot
        }
    except Exception as e:
        print(f"[INIT] Failed to initialize dataset: {e}")
        traceback.print_exc()
        return

    try:
        # Init the arm instances
        print("🔧 Moving robot to start pose...")
        obs = robot.get_observation()
        left_arm_teleop = SimpleTeleopArm(LEFT_JOINT_MAP, obs, prefix="left")
        right_arm_teleop = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, prefix="right")

        # Move both arms to zero position at start
        joint_ipol = JointIpol()
        joint_ipol.plan_to_target(robot, left_arm_teleop, right_arm_teleop, ctrl_freq=200, target_positions=FULL_START_POS)
        joint_ipol.execute_plan(robot, left_arm_teleop, right_arm_teleop)
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
        display_data = False
        recorded_episodes = 0
        while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
            start_episode_t = None
            episode_started = False
            timestamp = 0
            vr_monitor.reset_goal_queues()
            print(f"✅ Start episode: {recorded_episodes}")
            
            while timestamp < EPISODE_TIME_SEC:
                start_loop_t = time.perf_counter()

                if events["exit_early"]:
                    events["exit_early"] = False
                    print("exit early")
                    break
                
                # Fetch ordered goals from queues
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
                    left_action, left_obs = left_arm_teleop.p_control_action(robot)
                    right_action, right_obs = right_arm_teleop.p_control_action(robot)

                    action = {**left_action, **right_action}
                    robot.send_action(action)

                    dt_s = time.perf_counter() - start_loop_t
                    precise_sleep(1 / FPS - dt_s)
                    continue

                # --- FIRST NON-NONE VR GOAL: start episode timer here ---
                if not episode_started:
                    episode_started = True
                    start_episode_t = time.perf_counter()
                    timestamp = 0.0

                # Right B on right controller to stop
                for g in (right_motion, right_reset):
                    if g and getattr(g, "buttons", None):
                        if g.buttons.get("B", 0):
                            events["exit_early"] = True
                            print("🔧 Exit early requested")
                        if g.buttons.get("A", 0):
                            events["rerecord_episode"] = True
                            events["exit_early"] = True
                            print("🔧 Rerecord requested")

                for g in (left_motion, left_reset):
                    if g and getattr(g, "buttons", None):
                        if g.buttons.get("X", 0):
                            events["stop_recording"] = True
                            events["exit_early"] = True
                            print("🔧 Stop recording requested")

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
                #print(f"left action: {left_action}")
                right_action, right_obs = right_arm_teleop.p_control_action(robot)
                #print(f"right action: {right_action}")

                # Merge all actions
                action = {**left_action, **right_action}
                robot.send_action(action)
                
                # Write to dataset
                if dataset is not None:
                    obs = {**left_obs, **right_obs, **camera_obs}
                    observation_frame = build_dataset_frame(dataset.features, obs, prefix=OBS_STR)
                    action_frame = build_dataset_frame(dataset.features, action, prefix=ACTION)
                    frame = {**observation_frame, **action_frame, "task": TASK_DESCRIPTION}
                    dataset.add_frame(frame)

                if display_data:
                    log_rerun_data(observation=obs, action=action)
                
                dt_s = time.perf_counter() - start_loop_t
                precise_sleep(1 / FPS - dt_s)
                timestamp = time.perf_counter() - start_episode_t

            
            if not events["stop_recording"] and (
            (recorded_episodes < NUM_EPISODES - 1) or events["rerecord_episode"]
            ):
                print(f"✅ Reset environment after episode: {recorded_episodes}")
                joint_ipol.plan_to_target(robot, left_arm_teleop, right_arm_teleop, ctrl_freq=FPS, target_positions=FULL_START_POS)                
                while joint_ipol.ipol_step < joint_ipol.num_via_points:
                    start_loop_t = time.perf_counter()

                    camera_obs = robot.get_camera_observation()

                    action, joint_obs = joint_ipol.get_next_action(robot)
                    robot.send_action(action)

                    # Write to dataset
                    if dataset is not None:
                        obs = {**joint_obs, **camera_obs}
                        observation_frame = build_dataset_frame(dataset.features, obs, prefix=OBS_STR)
                        action_frame = build_dataset_frame(dataset.features, action, prefix=ACTION)
                        frame = {**observation_frame, **action_frame, "task": TASK_DESCRIPTION}
                        dataset.add_frame(frame)

                    dt_s = time.perf_counter() - start_loop_t
                    precise_sleep(1 / FPS - dt_s)
                
                joint_ipol.reset_ipol(left_arm_teleop, right_arm_teleop)
            
            if events["rerecord_episode"]:
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            print(f"🚀 Saved episode: {recorded_episodes}")
            recorded_episodes += 1
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        
    finally:
        # Cleanup
        if robot:
            joint_ipol.plan_to_target(robot, left_arm_teleop, right_arm_teleop, ctrl_freq=200)
            joint_ipol.execute_plan(robot, left_arm_teleop, right_arm_teleop)
            robot.disconnect()
        
        if dataset:
            dataset.finalize()
            print(f"Collected Dataset: {dataset.meta.total_episodes} episodes, {dataset.meta.total_frames} frames")
            print(f"Features: {list(dataset.meta.features.keys())}")
            #dataset.push_to_hub()
            print(f"🚀 Finished dataset collection")

if __name__ == "__main__":
    main()
