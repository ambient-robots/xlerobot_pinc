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
from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
from lerobot.utils.robot_utils import precise_sleep

from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.visualization_utils import log_rerun_data, init_rerun
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.datasets.utils import hw_to_dataset_features, build_dataset_frame

from openpi_client import image_tools
from openpi_client import websocket_client_policy

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

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
#TASK_DESCRIPTION = "Do nothing"
HF_REPO_ID = "xuweiwu/bimanual-toy-box-cleanup"
FPS = 50
NUM_EPISODES = 2
EPISODE_TIME_SEC = 600

class SimpleArmControl:
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
        print("Reached target pos of with ipol trajectory.")
    
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
            self, robot, left_arm, right_arm, ctrl_freq=FPS,
            target_positions=None, max_vel_per_joint=None, max_acc_per_joint=None, max_dev_per_joint=None):
            """
            Plan a quadratic-spline trajectory from current q to zero/init pos and execute it
            at fixed control frequency. Finishes exactly at `duration` if feasible.

            Raises:
                ValueError if requested duration is infeasible given the limits.
            """
            # 0) define target order explicitly via target_positions (canonical order)
            if target_positions is None:
                left_target_pos = {v: left_arm.home_pos[k] for k, v in LEFT_JOINT_MAP.items()}
                right_target_pos = {v: right_arm.home_pos[k] for k, v in RIGHT_JOINT_MAP.items()}
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
    
    def execute_plan(self, robot, left_arm, right_arm):
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
        
        self.reset_ipol(left_arm, right_arm)
    
    def reset_ipol(self, left_arm, right_arm):

        for k, v in LEFT_JOINT_MAP.items():
            left_arm.target_positions[k] = self.target_positions[v]
        print(f"ipol resets left arm target positions to: {left_arm.target_positions}")
        for k, v in RIGHT_JOINT_MAP.items():
            right_arm.target_positions[k] = self.target_positions[v]
        print(f"ipol resets right arm target positions to: {right_arm.target_positions}")

        self.ipol_path = None
        self.ipol_step = 0
        self.num_via_points = 0
        self.target_positions = None
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

    robot = None
    robot_name = "ambient_xlerobot"
    try:
        robot_config = XLerobotConfig(id=robot_name, use_degrees=True)
        robot = XLerobot(robot_config)
        robot.connect()

        #Init the keyboard instance
        keyboard_config = KeyboardTeleopConfig()
        keyboard = KeyboardTeleop(keyboard_config)
        keyboard.connect()

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
        # Init the arm instances
        print("🔧 Moving robot to start pose...")
        obs = robot.get_observation()
        left_arm = SimpleArmControl(LEFT_JOINT_MAP, obs, prefix="left")
        right_arm = SimpleArmControl(RIGHT_JOINT_MAP, obs, prefix="right")

        # Move both arms to zero position at start
        joint_ipol = JointIpol()
        joint_ipol.plan_to_target(robot, left_arm, right_arm, ctrl_freq=200, target_positions=FULL_START_POS)
        joint_ipol.execute_plan(robot, left_arm, right_arm)
        print("✅ Robot in start pose")

        client = websocket_client_policy.WebsocketClientPolicy(host="192.168.0.63", port=8000)
        action_horizon =50

        # Initiailize events
        events = {
            "stop_inference": False,   # Press key ^: Stop inference
        }

        print("Starting inference loop...")
        inference_episodes = 0
        while inference_episodes < NUM_EPISODES and not events["stop_inference"]:

            start_episode_t = time.perf_counter()
            timestamp = 0
            count_action_chunks = 0
            print(f"✅ Start episode: {inference_episodes}")
            
            while timestamp < EPISODE_TIME_SEC:
                pressed_keys = set(keyboard.get_action().keys())
                if '^' in pressed_keys:
                    events["stop_inference"] = True
                    break

                left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
                right_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
                joint_obs = {**left_obs, **right_obs}
                joint_states = np.array([float(joint_obs[n]) for n in FULL_START_POS.keys()], dtype=float)
                camera_obs = robot.get_camera_observation()
                observation = {
                "image/head": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(camera_obs['head'], 224, 224)
                ),
                "image/left_wrist": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(camera_obs['left_wrist'], 224, 224)
                ),
                "image/right_wrist": image_tools.convert_to_uint8(
                    image_tools.resize_with_pad(camera_obs['right_wrist'], 224, 224)
                ),
                "state": joint_states,
                "prompt": TASK_DESCRIPTION,
                }
                if count_action_chunks%10 ==0:
                    print(f"Sent observation for inference: {observation}")

                action_chunk = client.infer(observation)["actions"]

                for t in range(action_horizon):
                    start_loop_t = time.perf_counter()
                    target = action_chunk[t]  # shape (12,)

                    for i, joint_name in enumerate(left_arm.target_positions.keys()):
                        left_arm.target_positions[joint_name] = target[i]

                    for i, joint_name in enumerate(right_arm.target_positions.keys()):
                        right_arm.target_positions[joint_name] = target[i+6]

                    # Compute actions from both arms
                    left_action, _ = left_arm.p_control_action(robot)
                    #print(f"left action: {left_action}")
                    right_action, _ = right_arm.p_control_action(robot)
                    #print(f"right action: {right_action}")

                    # Merge all actions
                    action = {**left_action, **right_action}
                    robot.send_action(action)
                    
                    dt_s = time.perf_counter() - start_loop_t
                    remaining = 1 / FPS - dt_s
                    if remaining > 0:
                        precise_sleep(remaining)

                timestamp = time.perf_counter() - start_episode_t
                count_action_chunks += 1
            
            if inference_episodes < NUM_EPISODES - 1:
                print(f"✅ Reset environment after episode: {inference_episodes}")
                joint_ipol.plan_to_target(robot, left_arm, right_arm, ctrl_freq=200, target_positions=FULL_START_POS)   
                joint_ipol.execute_plan(robot, left_arm, right_arm)

            print(f"🚀 Finished episode: {inference_episodes}")
            inference_episodes += 1
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        
    finally:
        # Cleanup
        if robot:
            joint_ipol.plan_to_target(robot, left_arm, right_arm, ctrl_freq=200)
            joint_ipol.execute_plan(robot, left_arm, right_arm)
            robot.disconnect()

if __name__ == "__main__":
    main()
