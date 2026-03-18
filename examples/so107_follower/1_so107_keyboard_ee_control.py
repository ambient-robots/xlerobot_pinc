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

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import EnvAction, EnvTransition, RobotAction, RobotObservation, RobotProcessorPipeline, TransitionKey
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

# Setup logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    force=True)
logger = logging.getLogger(__name__)

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
    'x+': 'q', 'x-': 'a', 
    'y+': 'w', 'y-': 's', 
    'z+': 'e', 'z-': 'd',
    'wx+': 'r', 'wx-': 'f', 
    'wy+': 't', 'wy-': 'g', 
    'wz+': 'z', 'wz-': 'h',
    'gripper+': 'y', 'gripper-': 'x',
    'reset': 'c',
}

FPS = 50
LOG_RERUN = False

class SimpleTeleopArm:
    def __init__(self, initial_obs, kp=0.8):
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
    
        joint_names_wo_gripper = [j for j in self.target_positions if j != 'gripper']
        self.kinematics= RobotKinematics(
            urdf_path="/home/that/xlerobot_pinc_urdf/robot.urdf", 
            target_frame_name="gripper_frame_link",
            joint_names=joint_names_wo_gripper,
        )
        self.kinematics.solver.dt = 1.0/FPS

        def print_act_obs(step_idx: int, transition: EnvTransition):
            action = transition.get(TransitionKey.ACTION)
            obs = transition.get(TransitionKey.OBSERVATION)
            if action:
                for key, value in action.items():
                    print(f"[Processor] Action value in {key} at step {step_idx}: {value}")
            if obs:
                for key, value in obs.items():
                    print(f"[Processor] Observation value in {key} at step {step_idx}: {value}")

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
        # Register the hook to run after each step
        self.ee_relative_to_robot_joints_processor.register_after_step_hook(print_act_obs)
        self.ee_relative_to_robot_joints_processor.register_before_step_hook(print_act_obs)
        self.ref_action_when_disabled = None
    
        # Set the rad step and pos step in [m]
        scale = 1
        self.rad_step = 30 * (np.pi/180) / FPS * scale
        self.pos_step = 0.2 / FPS * scale
        self.gripper_vel_step = 1 * scale

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
        print(f"[IPOL] Current joint positions: {obs}")

        # 2) choose names in the order of home_positions, filter to those present in obs
        names = [n for n in target_positions if n in obs]
        missing = [n for n in target_positions if n not in obs]
        if missing:
            print(f"[IPOL] Warning: skipping joints missing in observation: {missing}")

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
        print(f"[IPOL] Streaming ipol trajectory: {len(t)} steps at {control_freq:.1f} Hz; "
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
        
        print("[IPOL] Reached end of ipol trajectory.")
        self.target_positions = target_positions.copy()
        print(f"[IPOL] Reset target positions to: {self.target_positions}")
        self.ee_relative_to_robot_joints_processor.reset()
        self.ref_action_when_disabled = None

    def handle_keys(self, robot, key_state):
        obs_raw = robot.bus.sync_read("Present_Position")
        obs = {f"{motor}.pos": val for motor, val in obs_raw.items()}
        print(f"[TELEOP] Current joint positions: {obs}")
        q_now = np.array([val for val in obs_raw.values()],  dtype=float)
        print(q_now)
        t = self.kinematics.forward_kinematics(q_now)
        print("[TELEOP] Current EE pos xyz:", t[:3, 3])

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
        print(f"[TELEOP] Relative ee actions: {target_action}")

        if target_action["enabled"]:
            ref_action = self.ee_relative_to_robot_joints_processor((target_action, obs))
            self.ref_action_when_disabled = ref_action.copy()
        else:
            if self.ref_action_when_disabled is None:
                self.ref_action_when_disabled = obs.copy()
            ref_action = self.ref_action_when_disabled.copy()
        
        for key, ref_pos in ref_action.items():
            self.target_positions[key.removesuffix('.pos')] = ref_pos
        print(f"[TELEOP] Reference joint positions: {self.target_positions}")
        q_ref = np.array([val for val in self.target_positions.values()],  dtype=float)
        t = self.kinematics.forward_kinematics(q_ref)
        print("[TELEOP] Reference ee pos xyz:", t[:3, 3])

    
    def p_control_action(self, robot):
        obs_raw = robot.bus.sync_read("Present_Position")
        obs = {f"{motor}.pos": val for motor, val in obs_raw.items()}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - obs_raw[j]
            control = self.kp * error
            action[f"{j}.pos"] = obs_raw[j] + control
        
        print(f"[CONTROL] Commanded actions: {action}")
        return action, obs

def main():
    """Main function"""
    print("LeRobot Simplified Keyboard Control Example (P Control)")
    print("="*50)
    
    try:
        # Get port
        port = input("Please enter SO107 robot USB port (e.g.: /dev/xlerobot_left_base or /dev/xlerobot_right_head): ").strip()
        # If directly press enter, use default port
        if not port:
            port = "/dev/xlerobot_left_base"
            print(f"[MAIN] Using default port: {port}")
        else:
            print(f"[MAIN] Connecting to port: {port}")

        # Get id
        id = input("Please enter SO107 robot id (e.g.: xlerobot_left_base or xlerobot_right_head): ").strip()
        if not id:
            id = "xlerobot_left_base"
            print(f"[MAIN] Using default id: {id}")
        else:
            print(f"[MAIN] Using id: {id}")
        
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
        robot_config = SO107FollowerConfig(port=port, id=id, cameras=cameras, use_degrees=True)
        robot = SO107Follower(robot_config)
        robot.connect()

        print("[MAIN] Robot connection successful!")
        print(f"[MAIN] Motor info: {robot.bus.motors}")
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
    obs = robot.get_observation()
    arm_teleop = SimpleTeleopArm(obs)
    
    print("[MAIN] Starting joint angles:")
    for joint_name, position in obs.items():
        print(f"{joint_name}: {position}°")

    if LOG_RERUN:
        init_rerun(session_name="so107_ee_teleop")
    
    # Move to zero position
    arm_teleop.move_to_target_with_ipol(robot, target_positions=START_POS)

    try:
        while True:
            start = time.perf_counter()

            pressed_keys = set(keyboard.get_action().keys())
            print(f"[MAIN] Pressed Keys: {pressed_keys}")

            # Move to home pose when stopped
            if '*' in pressed_keys:
                print("[MAIN] User ended teleop")
                break

            key_state = {action: (key in pressed_keys) for action, key in JOINT_KEYMAP.items()}
            print(f"[MAIN] Key state: {key_state}")
            
            # Handle reset for left arm
            if key_state.get('reset'):
                arm_teleop.move_to_target_with_ipol(robot, target_positions=START_POS)
                continue

            arm_teleop.handle_keys(robot, key_state)
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
            logger.debug(f"[MAIN] Control delay: {dt_ms:.1f}ms")
            precise_sleep(1.0 / FPS - dt_s)

    except KeyboardInterrupt:
        print("[MAIN] User interrupted program")
    finally:
        arm_teleop.move_to_target_with_ipol(robot)
        robot.disconnect()
        keyboard.disconnect()
        print("[MAIN] Teleoperation ended.")
        

if __name__ == "__main__":
    main() 
