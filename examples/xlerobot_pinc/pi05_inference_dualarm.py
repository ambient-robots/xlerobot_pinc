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
from lerobot.robots.xlerobot_pinc import XLerobotPincConfig, XLerobotPinc

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

TASK_DESCRIPTION = "Take apart the LEGO build and put the pieces into the zippered bag."
NUM_EPISODES = 50
EPISODE_TIME_SEC = 300
OPENPI_LOCAL_SERVER_IP = "127.0.1.1"
OPENPI_REMOTE_SERVER_IP = "192.168.0.63"

FPS = 50
LOCAL_INFERENCE = False

class SimpleControlArm:
    def __init__(self, joint_map, initial_obs, prefix="left", kp=0.98):
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

        action = {}
        for j in self.target_positions:
            q_mes = obs_no_prefix[j]
            q_err = self.target_positions[j] - q_mes
            action[f"{self.joint_map[j]}.pos"] = q_mes + self.kp * q_err
        
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

def build_policy_observation(robot):
    left_obs = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
    right_obs = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
    joint_obs = {**left_obs, **right_obs}
    joint_states = np.array([float(joint_obs[n]) for n in FULL_START_POS.keys()], dtype=float)

    camera_obs = robot.get_camera_observation()
    return {
        "image/head": image_tools.convert_to_uint8(
            image_tools.resize_with_pad(camera_obs["head"], 224, 224)
        ),
        "image/left_wrist": image_tools.convert_to_uint8(
            image_tools.resize_with_pad(camera_obs["left_wrist"], 224, 224)
        ),
        "image/right_wrist": image_tools.convert_to_uint8(
            image_tools.resize_with_pad(camera_obs["right_wrist"], 224, 224)
        ),
        "state": joint_states,
        "prompt": TASK_DESCRIPTION,
    }

def execute_target(robot, left_arm, right_arm, target):
    for i, joint_name in enumerate(left_arm.target_positions.keys()):
        left_arm.target_positions[joint_name] = target[i]

    for i, joint_name in enumerate(right_arm.target_positions.keys()):
        right_arm.target_positions[joint_name] = target[i + 7]

    left_action, _ = left_arm.p_control_action(robot)
    right_action, _ = right_arm.p_control_action(robot)
    robot.send_action({**left_action, **right_action})

def main():
    print("XLerobot Pro Pi0.5 Inference Example")
    print("="*50)

    robot = None
    robot_name = "ambient_xlerobot_pinc"
    try:
        robot_config = XLerobotPincConfig(id=robot_name, use_degrees=True)
        robot = XLerobotPinc(robot_config)
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
        openpi_server_ip = OPENPI_LOCAL_SERVER_IP if LOCAL_INFERENCE else OPENPI_REMOTE_SERVER_IP
        client = websocket_client_policy.WebsocketClientPolicy(host=openpi_server_ip, port=8000)
        action_horizon = 50

        # Initiailize events
        events = {
            "stop_inference": False,   # Press key q: Stop inference
            "reset_episode": False,    # Press key w: reset episode
        }
        print("✅ Policy server ready")

        print("Starting inference loop...")
        inference_episodes = 0
        while inference_episodes < NUM_EPISODES and not events["stop_inference"]:
            start_episode_t = time.perf_counter()
            timestamp = 0
            input("🔧Press ENTER to start the next episode ...")
            print(f"✅ Start episode: {inference_episodes}")

            while timestamp < EPISODE_TIME_SEC:
                pressed_keys = set(keyboard.get_action().keys())
                if 'q' in pressed_keys:
                    events["stop_inference"] = True
                    break
                if 'w' in pressed_keys:
                    events["reset_episode"] = True
                    break
                
                observation = build_policy_observation(robot)

                print("Sent observation for inference")
                start_infer_t = time.perf_counter()
                response = client.infer(observation)
                action_chunk = response["actions"]
                policy_timing_infer_ms = response["policy_timing"]["infer_ms"]
                server_timing_infer_ms = response["server_timing"]["infer_ms"]
                client_timing_total_ms = (time.perf_counter() - start_infer_t )* 1e3
                print(f"Policy timing: infer={policy_timing_infer_ms:.1f} ms")
                print(f"Server timing: infer={server_timing_infer_ms:.1f} ms")
                print(f"Client timing: total={client_timing_total_ms:.1f} ms")

                for t in range(action_horizon):
                    start_loop_t = time.perf_counter()
                    target = action_chunk[t]
                    execute_target(robot, left_arm, right_arm, target)
                    dt_s = time.perf_counter() - start_loop_t
                    precise_sleep(1 / FPS - dt_s)

                timestamp = time.perf_counter() - start_episode_t

            if inference_episodes < NUM_EPISODES - 1:
                print(f"✅ Reset environment after episode: {inference_episodes}")
                joint_ipol.plan_to_target(robot, left_arm, right_arm, ctrl_freq=FPS, target_positions=FULL_START_POS)   
                joint_ipol.execute_plan(robot, left_arm, right_arm)

            if events["reset_episode"]:
                events["reset_episode"] = False
                continue
            
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
