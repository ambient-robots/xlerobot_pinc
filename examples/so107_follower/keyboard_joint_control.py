#!/usr/bin/env python3
"""
Simplified keyboard control for SO100/SO101 robot
Fixed action format conversion issues
Uses P control, keyboard only changes target joint angles
"""

import time
import logging
import traceback
import numpy as np
from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.robot_utils import precise_sleep
from lerobot.robots.so107_follower import SO107Follower, SO107FollowerConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.visualization_utils import log_rerun_data, init_rerun

from lerobot.cameras.configs import Cv2Rotation, ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

# Setup logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    force=True)
logger = logging.getLogger(__name__)

DEFAULT_PORT = "/dev/xlerobot_left_base"
DEFAULT_ID = "xlerobot_left_base"

START_POS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "elbow_roll": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}

# Keymaps (semantic action: key)
JOINT_KEYMAP = {
    'shoulder_pan+':  'q', 'shoulder_pan-':  'a', 
    'shoulder_lift+': 'w', 'shoulder_lift-': 's', 
    'elbow_flex+':    'e', 'elbow_flex-':    'd',
    'elbow_roll+':    'r', 'elbow_roll-':    'f', 
    'wrist_flex+':    't', 'wrist_flex-':    'g', 
    'wrist_roll+':    'z', 'wrist_roll-':    'h',
    'gripper+':       'y', 'gripper-':       'x',
    'reset':          'c',
}

FPS = 50
LOG_RERUN = False

class SimpleTeleopArm:
    def __init__(self, initial_obs, kp=0.5):
        self.kp = kp

        # Set target positions to zero for P control
        self.target_positions = START_POS

        # Initial joint positions
        self.home_pos = {
            "shoulder_pan": initial_obs["shoulder_pan.pos"],
            "shoulder_lift": initial_obs["shoulder_lift.pos"],
            "elbow_flex": initial_obs["elbow_flex.pos"],
            "elbow_roll": initial_obs["elbow_roll.pos"],
            "wrist_flex": initial_obs["wrist_flex.pos"],
            "wrist_roll": initial_obs["wrist_roll.pos"],
            "gripper": initial_obs["gripper.pos"],
        }
    
        # Set the deg step and gripper step
        self.deg_step = 0.5
        self.gripper_step = 1

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
        obs = robot.bus.sync_read("Present_Position")

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
            
            obs = robot.bus.sync_read("Present_Position")
            q_meas = np.array([float(obs[n]) for n in names], dtype=float)
            q_ref = np.array([q[k_step, i] for i, n in enumerate(names)], dtype=float)
            q_cmd = q_meas + self.kp*(q_ref - q_meas)
            action = {f"{j}.pos": q_cmd[i] for i, j in enumerate(names)}
            robot.send_action(action)

            # sleep to maintain control_freq (best-effort wall clock pacing)
            next_tick += dt
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(next_tick - now)
        
        self.target_positions = target_positions.copy()
        print(f"ipol resets arm target positions to: {self.target_positions}")
        print("Reached target pos of with ipol trajectory.")

    def handle_keys(self, key_state):

        if key_state.get('shoulder_pan+'):
            self.target_positions["shoulder_pan"] += self.deg_step
        elif key_state.get('shoulder_pan-'):
            self.target_positions["shoulder_pan"] -= self.deg_step

        if key_state.get('shoulder_lift+'):
            self.target_positions["shoulder_lift"] += self.deg_step
        elif key_state.get('shoulder_lift-'):
            self.target_positions["shoulder_lift"] -= self.deg_step

        if key_state.get('elbow_flex+'):
            self.target_positions["elbow_flex"] += self.deg_step
        elif key_state.get('elbow_flex-'):
            self.target_positions["elbow_flex"] -= self.deg_step

        if key_state.get('elbow_roll+'):
            self.target_positions["elbow_roll"] += self.deg_step
        elif key_state.get('elbow_roll-'):
            self.target_positions["elbow_roll"] -= self.deg_step

        if key_state.get('wrist_flex+'):
            self.target_positions["wrist_flex"] += self.deg_step
        elif key_state.get('wrist_flex-'):
            self.target_positions["wrist_flex"] -= self.deg_step
        
        if key_state.get('wrist_roll+'):
            self.target_positions["wrist_roll"] += self.deg_step
        elif key_state.get('wrist_roll-'):
            self.target_positions["wrist_roll"] -= self.deg_step

        if key_state.get('gripper+'):
            self.target_positions["gripper"] += self.gripper_step
        elif key_state.get('gripper-'):
            self.target_positions["gripper"] -= self.gripper_step

        logger.debug(f"arm ref positions: {self.target_positions}")
    
    def p_control_action(self, robot):
        obs_raw = robot.bus.sync_read("Present_Position")
        obs = {f"{motor}.pos": val for motor, val in obs_raw.items()}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - obs_raw[j]
            control = self.kp * error
            action[f"{j}.pos"] = obs_raw[j] + control
        
        logger.debug(f"arm commanded actions: {action}")
        return action, obs

def main():
    """Main function"""
    print("LeRobot Simplified Keyboard Control Example (P Control)")
    print("="*50)
    
    try:
        port = input(
            "Please enter SO107 robot USB port "
            f"(e.g.: /dev/xlerobot_left_base or /dev/xlerobot_right_head) [{DEFAULT_PORT}]: "
        ).strip() or DEFAULT_PORT
        print(f"Connecting to port: {port}")

        robot_id = input(
            "Please enter SO107 robot id "
            f"(e.g.: xlerobot_left_base or xlerobot_right_head) [{DEFAULT_ID}]: "
        ).strip() or DEFAULT_ID
        print(f"Using id: {robot_id}")
        
        # Configure and init robot
        cameras = {
        # "left_wrist": OpenCVCameraConfig(
        #     index_or_path='/dev/v4l/by-path/platform-a80aa10000.usb-usb-0:1.3:1.0-video-index0', 
        #     fps=60,
        #     width=640,
        #     height=480,
        #     color_mode=ColorMode.RGB,
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     fourcc="MJPG"
        # ),

        # "right_wrist": OpenCVCameraConfig(
        #     index_or_path='/dev/v4l/by-path/platform-a80aa10000.usb-usb-0:1.4:1.0-video-index0',
        #     fps=60,
        #     width=640,
        #     height=480,
        #     color_mode=ColorMode.RGB,
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     fourcc="MJPG"
        # ),
        
        # "head": RealSenseCameraConfig(
        #     serial_number_or_name="213622300072",  # Replace with camera SN
        #     fps=60,
        #     width=640,
        #     height=480,
        #     color_mode=ColorMode.RGB,
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     use_depth=False
        # ),

        # "rear": RealSenseCameraConfig(
        #     serial_number_or_name="308222301716",  # Replace with camera SN
        #     fps=60,
        #     width=640,
        #     height=480,
        #     color_mode=ColorMode.RGB,
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     use_depth=False
        # ),
        }
        robot_config = SO107FollowerConfig(port=port, id=robot_id, cameras=cameras, use_degrees=True)
        robot = SO107Follower(robot_config)
        robot.connect()

        print("Robot connection successful!")
        print(f"motor info: {robot.bus.motors}")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(robot_config)
        print(robot)
        traceback.print_exc()
        return

    continue_choice = input("Do you want to continue to teleop the robot? (y/n, [default y]): ").strip().lower()
    if continue_choice in ['n', 'no']:
        print(f"[MAIN] User decided not to continue")
        return
        
    # Configure and init keyboard
    keyboard_config = KeyboardTeleopConfig()
    keyboard = KeyboardTeleop(keyboard_config)
    keyboard.connect()
    
    # Read starting joint angles and init arm teleop
    print("Reading starting joint angles...")
    obs = robot.get_observation()
    arm_teleop = SimpleTeleopArm(obs)
    
    print("Starting joint angles:")
    for joint_name, position in obs.items():
        print(f"{joint_name}: {position}°")

    if LOG_RERUN:
        init_rerun(session_name="so107_joint_teleop")
    
    # Move to zero position
    arm_teleop.move_to_target_with_ipol(robot, target_positions=START_POS)

    try:
        while True:
            start = time.perf_counter()

            pressed_keys = set(keyboard.get_action().keys())
            logger.debug(f"[MAIN] Pressed Keys: {pressed_keys}")

            # Move to home pose when stopped
            if '*' in pressed_keys:
                print("User ended teleop")
                break

            key_state = {action: (key in pressed_keys) for action, key in JOINT_KEYMAP.items()}
            
            # Handle reset for left arm
            if key_state.get('reset'):
                arm_teleop.move_to_target_with_ipol(robot, target_positions=START_POS)
                continue

            arm_teleop.handle_keys(key_state)
            action, joint_obs = arm_teleop.p_control_action(robot)
            robot.send_action(action)

            camera_obs = robot.get_camera_observation()
            obs = {**joint_obs, **camera_obs}
            for k in robot.action_features:
                logger.debug(f"[MAIN] Observation: {k}: {obs[k]}")

            if LOG_RERUN:
                log_rerun_data(obs, action)
            
            dt_s = time.perf_counter() - start
            dt_ms = dt_s * 1e3
            logger.info(f"control delay: {dt_ms:.1f}ms")
            precise_sleep(1 / FPS - dt_s)

    except KeyboardInterrupt:
        print("User interrupted program")
    finally:
        arm_teleop.move_to_target_with_ipol(robot)
        robot.disconnect()
        keyboard.disconnect()
        print("Teleoperation ended.")
        

if __name__ == "__main__":
    main() 
