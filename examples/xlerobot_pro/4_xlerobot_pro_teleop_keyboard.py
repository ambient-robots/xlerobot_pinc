#!/usr/bin/env python3

import time
import logging
import traceback
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
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

# Setup logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    force=True)
logger = logging.getLogger(__name__)

# Keymaps (semantic action: key)
LEFT_KEYMAP = {
    'x+': 'q', 'x-': 'a', 
    'y+': 'w', 'y-': 's', 
    'z+': 'e', 'z-': 'd',
    'wx+': 'r', 'wx-': 'f', 
    'wy+': 't', 'wy-': 'g', 
    'wz+': 'z', 'wz-': 'h',
    'gripper+': 'y', 'gripper-': 'x',
    'reset': 'c',
}
RIGHT_KEYMAP = {
    'x+': 'u', 'x-': 'j', 
    'y+': 'i', 'y-': 'k', 
    'z+': 'o', 'z-': 'l',
    'wx+': 'p', 'wx-': 'ö', 
    'wy+': 'ü', 'wy-': 'ä', 
    #'wz+': '+', 'wz-': '#',
    'gripper+': ',', 'gripper-': '.',
    'reset': '-',    
}
HEAD_KEYMAP = {
    "head_pan+": "v", "head_pan-": "b",
    "head_tilt+": "n", "head_tilt-": "m",
    'reset': 'ß',
}

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
    "left_arm_wrist_flex": 85.0,
    "left_arm_wrist_roll": 0.0,
    "left_arm_gripper": 0.0,
    "right_arm_shoulder_pan": 0.0,
    "right_arm_shoulder_lift": -90.0,
    "right_arm_elbow_flex": 45.0,
    "right_arm_elbow_roll": 0.0,
    "right_arm_wrist_flex": 85.0,
    "right_arm_wrist_roll": 0.0,
    "right_arm_gripper": 0.0,
    "head_pan": 0.0,
    "head_tilt": 0.0,
}

FPS = 50
LOG_RERUN = True

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
        
    def handle_keys(self, key_state):
        if not any(key_state.values()):
            return
        
        if key_state.get('head_pan+'):
            self.target_positions["head_pan"] += self.degree_step
        elif key_state.get('head_pan-'):
            self.target_positions["head_pan"] -= self.degree_step
        if key_state.get('head_tilt+'):
            self.target_positions["head_tilt"] += self.degree_step
        elif key_state.get('head_tilt-'):
            self.target_positions["head_tilt"] -= self.degree_step            
        print(f"[HEAD TELEOP] head_pan: {self.target_positions['head_pan']}")
        print(f"[HEAD TELEOP] head_tilt: {self.target_positions['head_tilt']}")

    def p_control_action(self, robot):
        obs = robot.bus_right_head.sync_read("Present_Position", robot.head_motors)
        action = {}
        for motor in self.target_positions:
            current_obs = obs.get(f"{self.joint_map[motor]}", 0.0)
            error = self.target_positions[motor] - current_obs
            control = self.kp * error
            action[f"{self.joint_map[motor]}.pos"] = current_obs + control
        print(f"[HEAD CONTROL] Commanded actions: {action}")
        return action

class SimpleTeleopArm:
    def __init__(self, joint_map, initial_obs, prefix="left", kp=0.8):
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
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

        joint_names_wo_gripper = [j for j in self.target_positions if j != 'gripper']
        self.kinematics= RobotKinematics(
            urdf_path="/home/that/ambient_urdf/robot.urdf", 
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
                    speed_factor=25.0,
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
    
        # Set the rad step and pos step in [m]
        self.rad_step = 15 * (np.pi/180) / FPS
        self.pos_step = 0.2 / FPS
        self.gripper_vel_step = 1

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

    def handle_keys(self, robot, key_state):
        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        obs = {f"{j}.pos": obs_raw[self.joint_map[j]] for j in self.joint_map}
        print(f"[{self.prefix.capitalize()} ARM TELEOP] Current joint positions: {obs}")

        target_action = {
            "enabled": False,
            "target_x": 0.0,
            "target_y": 0.0,
            "target_z": 0.0,
            "target_wx": 0.0,
            "target_wy": 0.0,
            "target_wz": 0.0,
            "gripper_vel": 0.0,
        }

        if key_state.get('x+'):
            target_action["target_x"] = self.pos_step
        elif key_state.get('x-'):
            target_action["target_x"] = -self.pos_step
        if key_state.get('y+'):
            target_action["target_y"] = self.pos_step
        elif key_state.get('y-'):
            target_action["target_y"] = -self.pos_step
        if key_state.get('z+'):
            target_action["target_z"] = self.pos_step
        elif key_state.get('z-'):
            target_action["target_z"] = -self.pos_step
        if key_state.get('wx+'):
            target_action["target_wx"] = self.rad_step
        elif key_state.get('wx-'):
            target_action["target_wx"] = -self.rad_step
        if key_state.get('wy+'):
            target_action["target_wy"] = self.rad_step
        elif key_state.get('wy-'):
            target_action["target_wy"] = -self.rad_step
        if key_state.get('wz+'):
            target_action["target_wz"] = self.rad_step
        elif key_state.get('wz-'):
            target_action["target_wz"] = -self.rad_step
        if key_state.get('gripper+'):
            target_action["gripper_vel"] = self.gripper_vel_step
        elif key_state.get('gripper-'):
            target_action["gripper_vel"] = -self.gripper_vel_step
        
        if any(key_state.values()):
            target_action["enabled"] = True
        print(f"[{self.prefix.capitalize()} ARM TELEOP] Relative ee actions: {target_action}")

        if target_action["enabled"]:
            ref_action = self.ee_relative_to_robot_joints_processor((target_action, obs))
            self.ref_action_when_disabled = ref_action.copy()
        else:
            if self.ref_action_when_disabled is None:
                self.ref_action_when_disabled = obs.copy()
            ref_action = self.ref_action_when_disabled.copy()
        
        for key, ref_pos in ref_action.items():
            self.target_positions[key.removesuffix('.pos')] = ref_pos
        print(f"[{self.prefix.capitalize()} ARM TELEOP] Ref joint positions: {self.target_positions}")

    def p_control_action(self, robot):
        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        obs = {j: obs_raw[f"{self.joint_map[j]}"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - obs[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = obs[j] + control
        
        print(f"[{self.prefix.capitalize()} ARM CONTROL] Commanded actions: {action}")
        return action


def move_to_target_full_body_with_ipol(
        robot, left_teleop, right_teleop, head_teleop, 
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
            head_target_pos = {v: head_teleop.home_pos[k] for k, v in HEAD_JOINT_MAP.items()}
            target_positions = {**left_target_pos, **right_target_pos, **head_target_pos}
        
        # 1) Read current joint positions
        left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        right_head_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors+robot.head_motors)
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

        print("[FULL-BODY IPOL] Reached end of ipol trajectory.")
        for k, v in LEFT_JOINT_MAP.items():
            left_teleop.target_positions[k] = target_positions[v]
        print(f"[FULL-BODY IPOL] Reset left arm target positions to: {left_teleop.target_positions}")
        for k, v in RIGHT_JOINT_MAP.items():
            right_teleop.target_positions[k] = target_positions[v]
        print(f"[FULL-BODY IPOL] Reset right arm target positions to: {right_teleop.target_positions}")
        for k, v in HEAD_JOINT_MAP.items():
            head_teleop.target_positions[k] = target_positions[v]
        print(f"[FULL-BODY IPOL] Reset head target positions to: {head_teleop.target_positions}")

        left_teleop.ee_relative_to_robot_joints_processor.reset()
        left_teleop.ref_action_when_disabled = None
        right_teleop.ee_relative_to_robot_joints_processor.reset()
        right_teleop.ref_action_when_disabled = None
    

def main():
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

    #Init the keyboard instance
    keyboard_config = KeyboardTeleopConfig()
    keyboard = KeyboardTeleop(keyboard_config)
    keyboard.connect()

    # Init the arm and head instances
    obs = robot.get_observation()
    left_arm_teleop = SimpleTeleopArm(LEFT_JOINT_MAP, obs, prefix="left")
    right_arm_teleop = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, prefix="right")
    head_teleop = SimpleHeadControl(HEAD_JOINT_MAP)

    if LOG_RERUN: 
        init_rerun(session_name="ambient_xlerobot_pro_keyboard_teleop")

    move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop, 
                                       target_positions=FULL_START_POS)
    
    try:
        while True:
            start = time.perf_counter()

            pressed_keys = set(keyboard.get_action().keys())
            print(f"[MAIN] Pressed Keys: {pressed_keys}")

            # Move to home pose when stopped
            if '*' in pressed_keys:
                print("[MAIN] User ended teleop")
                break
            
            # Move to start pose
            if '^' in pressed_keys:
                move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop, target_positions=FULL_START_POS)
                continue

            left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
            right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}
            head_key_state = {action: (key in pressed_keys) for action, key in HEAD_KEYMAP.items()}

            # Handle reset for left arm
            if left_key_state.get('reset'):
                left_arm_teleop.move_to_target_with_ipol(robot, target_positions=left_arm_teleop.start_pos)
                continue  

            # Handle reset for right arm
            if right_key_state.get('reset'):
                right_arm_teleop.move_to_target_with_ipol(robot, target_positions=right_arm_teleop.start_pos)
                continue

            # Handle reset for head motors
            if head_key_state.get('reset'):
                head_teleop.move_to_target_with_ipol(robot, target_positions=head_teleop.start_pos)
                continue

            left_arm_teleop.handle_keys(robot, left_key_state)
            right_arm_teleop.handle_keys(robot, right_key_state)
            head_teleop.handle_keys(head_key_state)

            left_action = left_arm_teleop.p_control_action(robot)
            right_action = right_arm_teleop.p_control_action(robot)
            head_action = head_teleop.p_control_action(robot)

            # Base action
            keyboard_keys = np.array(list(pressed_keys))
            base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
            print(f"[MAIN] Base action: {base_action}")

            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            for k in robot.action_features:
                logger.debug(f"[MAIN] Observation: {k}: {obs[k]}")

            if LOG_RERUN:
                log_rerun_data(obs, action)

            dt_s = time.perf_counter() - start
            dt_ms = dt_s * 1e3
            logger.debug(f"[MAIN] Control delay: {dt_ms:.1f}ms")
            precise_sleep(1.0 / FPS - dt_s)

    except KeyboardInterrupt:
            print("[MAIN] User interrupted program")
    finally:
        move_to_target_full_body_with_ipol(robot, left_arm_teleop, right_arm_teleop, head_teleop)
        robot.disconnect()
        keyboard.disconnect()
        print("[MAIN] Teleoperation ended.")

if __name__ == "__main__":
    main()
