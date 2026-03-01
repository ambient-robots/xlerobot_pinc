#!/usr/bin/env python3
"""
VR control for XLerobot robot
Uses handle_vr_input with delta action control
"""

# Standard library imports
import time
import logging
import traceback
import numpy as np

from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.robot_utils import precise_sleep
from lerobot.robots.xlerobot_pro import XLerobotProConfig, XLerobotPro

from openpi_client import image_tools
from openpi_client import websocket_client_policy

from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

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

TASK_DESCRIPTION = "Take apart the LEGO build and put all pieces into the yellow zippered bag."
NUM_EPISODES = 50
EPISODE_TIME_SEC = 300
OPENPI_SERVER_IP = "192.168.0.63"
#OPENPI_SERVER_IP = "xxx.xxx.x.xx" # ip address of the server that provides pi05 inference as service

FPS = 60

class SimpleControlArm:
    def __init__(self, joint_map, initial_obs, prefix="left", kp=0.75):
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
    
    def p_control_action(self, robot):

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

class JointIpol:
    def __init__(self, kp=0.8, duration=3.0):
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
                max_vel_per_joint = np.full(J, 30)   # deg/s (pick something reasonable)
            if max_acc_per_joint is None:
                max_acc_per_joint = np.full(J, 60)   # deg/s^2
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
            logger.debug(f"[FULL-BODY IPOL] Streaming ipol trajectory: {len(t)} steps at {self.ctrl_freq:.1f} Hz; "
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
            precise_sleep(next_tick - now)
        
        self.reset_ipol(left_teleop, right_teleop)
    
    def reset_ipol(self, left_teleop, right_teleop):

        for k, v in LEFT_JOINT_MAP.items():
            left_teleop.target_positions[k] = self.target_positions[v]
        logger.debug(f"[FULL-BODY IPOL] Resets left arm target positions to: {left_teleop.target_positions}")
        for k, v in RIGHT_JOINT_MAP.items():
            right_teleop.target_positions[k] = self.target_positions[v]
        logger.debug(f"[FULL-BODY IPOL] Resets right arm target positions to: {right_teleop.target_positions}")

        self.ipol_path = None
        self.ipol_step = 0
        self.num_via_points = 0
        self.target_positions = None

def main():
    print("XLerobot Pro Pi0.5 Inference Example")
    print("="*50)

    robot = None
    robot_name = "ambient_xlerobot_pro"
    try:
        robot_config = XLerobotProConfig(id=robot_name, use_degrees=True)
        robot = XLerobotPro(robot_config)
        robot.connect()

        #Init the keyboard instance
        keyboard_config = KeyboardTeleopConfig()
        keyboard = KeyboardTeleop(keyboard_config)
        keyboard.connect()

        print("[INIT] Robot connection successful!")
        print(f"[INIT] Motor bus_left_base info: {robot.bus_left_base.motors}")
        print(f"[INIT] Motor bus_right_head info: {robot.bus_right_head.motors}")
    except Exception as e:
        print(f"[INIT] Failed to connect to robot: {e}")
        print(f"[INIT] Robot config: {robot_config}")
        print(f"[INIT] Robot: {robot}")
        traceback.print_exc()
        return
    
    continue_choice = input("Do you want to continue to control the robot? (y/n, [default y]): ").strip().lower()
    if continue_choice in ['n', 'no']:
        print(f"[INIT] User decided not to continue")
        left_base_pos = robot.bus_left_base.sync_read("Present_Position", normalize=False)
        right_head_pos = robot.bus_right_head.sync_read("Present_Position", normalize=False)
        print(f"[INIT] Left base states: {left_base_pos}")
        print(f"[INIT] Right head states: {right_head_pos}")
        return
    
    try:
        # Init the arm instances
        print("🔧 Moving robot to start pose...")
        obs = robot.get_observation()
        left_arm = SimpleControlArm(LEFT_JOINT_MAP, obs, prefix="left")
        right_arm = SimpleControlArm(RIGHT_JOINT_MAP, obs, prefix="right")

        # Move both arms to zero position at start
        joint_ipol = JointIpol()
        joint_ipol.plan_to_target(robot, left_arm, right_arm, ctrl_freq=FPS, target_positions=FULL_START_POS)
        joint_ipol.execute_plan(robot, left_arm, right_arm)
        print("✅ Robot in start pose")
        
        # Initialize policy server
        print("🔧 Initializing Policy Server...")
        client = websocket_client_policy.WebsocketClientPolicy(host=OPENPI_SERVER_IP, port=8000)
        action_horizon =50

        # Initiailize events
        events = {
            "stop_inference": False,   # Press key ^: Stop inference
        }
        print("✅ VR system ready")

        print("Starting inference loop...")
        inference_episodes = 0
        while inference_episodes < NUM_EPISODES and not events["stop_inference"]:
            start_episode_t = time.perf_counter()
            timestamp = 0
            count_action_chunks = 0

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
                    target = action_chunk[t]

                    for i, joint_name in enumerate(left_arm.target_positions.keys()):
                        left_arm.target_positions[joint_name] = target[i]

                    for i, joint_name in enumerate(right_arm.target_positions.keys()):
                        right_arm.target_positions[joint_name] = target[i+7]

                    # Compute actions from both arms
                    left_action, _ = left_arm.p_control_action(robot)
                    right_action, _ = right_arm.p_control_action(robot)

                    # Merge all actions
                    action = {**left_action, **right_action}
                    robot.send_action(action)
                    
                    dt_s = time.perf_counter() - start_loop_t
                    precise_sleep(1 / FPS - dt_s)

                timestamp = time.perf_counter() - start_episode_t
                count_action_chunks += 1

            if inference_episodes < NUM_EPISODES - 1:
                print(f"✅ Reset environment after episode: {inference_episodes}")
                joint_ipol.plan_to_target(robot, left_arm, right_arm, ctrl_freq=FPS, target_positions=FULL_START_POS)   
                joint_ipol.execute_plan(robot, left_arm, right_arm)
            
            print(f"🚀 Saved episode: {inference_episodes}")
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
