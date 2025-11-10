#!/usr/bin/env python3
"""
Simplified keyboard control for SO100/SO101 robot
Fixed action format conversion issues
Uses P control, keyboard only changes target joint angles
"""

import time
import logging
import traceback
import math
import cv2
import numpy as np
import threading

from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    robot_action_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEReferenceAndDelta,
    EEBoundsAndSafety,
    GripperVelocityToJoint,
    ForwardKinematicsJointsToEE,
    InverseKinematicsEEToJoints,
)
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from placo_utils.tf import tf

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def move_to_target_with_ipol(
    robot,
    duration=5.0,
    control_freq=200.0,
    max_vel_per_joint=None,
    max_acc_per_joint=None,
    max_dev_per_joint=None,
    target_positions=None,
):
    """
    Plan a quadratic-spline trajectory from current q to target and execute it
    at fixed control frequency. Finishes exactly at `duration` if feasible.

    Raises:
        ValueError if requested duration is infeasible given the limits.
    """
    
    # 0) define target order explicitly via target_positions (canonical order)
    if target_positions is None:
        target_positions = {
            'shoulder_pan':  0.0,
            'shoulder_lift': 0.0,
            'elbow_flex':    0.0,
            'wrist_flex':    0.0,
            'wrist_roll':    -90.0,
            'gripper':       0.0,
        }

    # 1) Read current joint positions (calibrated if provided)
    obs = robot.get_observation()

    # 2) choose names in the order of zero_positions, filter to those present in obs
    names = [n for n in target_positions.keys() if f"{n}.pos" in obs]
    missing = [n for n in target_positions.keys() if f"{n}.pos" not in obs]
    if missing:
        print(f"[ipol] Warning: skipping joints missing in observation: {missing}")
    print(f"motor names in obs: {names}")

    # 3) build current/goal vectors in that SAME order (no sorting)
    q_now = []
    q_goal = []
    for n in names:
        v = float(obs[f"{n}.pos"])
        q_now.append(v)
        q_goal.append(float(target_positions[n]))
    q_now  = np.array(q_now,  dtype=float)
    q_goal = np.array(q_goal, dtype=float)
    print(f"Current pos: {q_now}")

    # 4) Limits (defaults if not provided)
    J = q_now.size
    if max_vel_per_joint is None:
        max_vel_per_joint = np.full(J, 10)   # deg/s (pick something reasonable)
    if max_acc_per_joint is None:
        max_acc_per_joint = np.full(J, 20)   # deg/s^2
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
        
        obs = robot.get_observation()
        q_meas = np.array([float(obs[f"{n}.pos"]) for n in names], dtype=float)
        q_ref = np.array([q[k_step, i] for i, n in enumerate(names)], dtype=float)
        q_cmd = q_meas + 0.5*(q_ref- q_meas)
        cmd = {f"{n}.pos": q_cmd[i] for i, n in enumerate(names)}
        robot.send_action(cmd)

        # sleep to maintain control_freq (best-effort wall clock pacing)
        next_tick += dt
        now = time.perf_counter()
        if now < next_tick:
            time.sleep(next_tick - now)
    
    obs = robot.get_observation()
    q_now = []
    for n in names:
        v = float(obs[f"{n}.pos"])
        q_now.append(v)
    q_now  = np.array(q_now,  dtype=float)
    print(f"Reached pos: {q_now}")
    print("Reached target with ipol trajectory.")

def keyboard_teleop(
    robot,
    keyboard,
    kinematics_solver,
    start_positions,
    kp=0.5,
    fps=30,
):
    keyboard_to_ee_pose = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        [   
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
                motor_names=list(robot.bus.motors.keys()),
                use_latched_reference=False,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-0.5, -0.5, -0.5], "max": [0.5, 0.5, 0.5]},
                max_ee_step_m=0.05,
            ),
            GripperVelocityToJoint(
                speed_factor=10.0,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    ee_to_robot_joints = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
        [
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(robot.bus.motors.keys()),
                weights={"position": 1.0, "orientation": 0.1},
                initial_guess_current_joints=False,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    # ee control
    ee_controls = {
        'q': ('x', 0.002),   # x decrease
        'a': ('x', -0.002),    # x increase
        'w': ('y', 0.002),   # y decrease
        's': ('y', -0.002),    # y increase
        'e': ('z', 0.002),   # z decrease
        'd': ('z', -0.002),    # z increase
        'z': ('wx', 0.01),   # wx decrease
        'h': ('wx', -0.01),    # wx increase
        'u': ('wy', 0.01),   # wy decrease
        'j': ('wy', -0.01),    # wy increase
        'i': ('wz', 0.01),   # wz decrease
        'k': ('wz', -0.01),    # wz increase
        'r': ('gripper', -1), # close gripper
        't': ('gripper', 1),  # open gripper
    }
    robot_action_when_disabled = None

    print(f"Starting keyboard teleop")
    while True:
        try:
            t0 = time.perf_counter()

            # Get current robot state
            robot_obs = robot.get_observation()

            # Get keyboard input
            keyboard_action = keyboard.get_action()

            # Extract current joint positions
            current_positions = {}
            for key, value in robot_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    current_positions[motor_name] = value

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
            
            if keyboard_action:
                # Process keyboard input, update target positions
                for key, value in keyboard_action.items():
                    if key == 'x':
                        # Exit program, first return to start position
                        print("Exit command detected, returning to start position...")
                        move_to_target_with_ipol(robot, duration=5.0, control_freq=200.0,
                               max_vel_per_joint=np.array([15, 15, 15, 15, 15, 10]),
                               max_acc_per_joint=np.array([30, 30, 30, 30, 30, 20]),
                               target_positions=start_positions)
                        print("Return to start position completed")
                        return
                    
                    if key in ee_controls:
                        coord, delta = ee_controls[key]                        
                        if coord == 'x':
                            target_action["target_x"] = delta
                        elif coord == 'y':
                            target_action["target_y"] = delta
                        elif coord == 'z':
                            target_action["target_z"] = delta
                        elif coord == 'wx':
                            target_action["target_wx"] = delta
                        elif coord == 'wy':
                            target_action["target_wy"] = delta
                        elif coord == 'wz':
                            target_action["target_wz"] = delta
                        elif coord == 'gripper':
                            target_action["gripper_vel"] = delta
                        target_action["enabled"] = True
            
            desired_ee_pos = keyboard_to_ee_pose((target_action, robot_obs))
            if target_action["enabled"]:
                robot_action = ee_to_robot_joints((desired_ee_pos, robot_obs))
                robot_action_when_disabled = robot_action.copy()
            else:
                if robot_action_when_disabled is None:
                    # If we've never had an enabled command yet, freeze current FK pose once.
                    robot_action_when_disabled = {f"{motor}.pos": pos for motor, pos in current_positions.items()}
                robot_action = robot_action_when_disabled.copy()
            print(f"target actions: {target_action}")
            print(f"robot actions: {robot_action}")

            # P control calculation
            for key, ref_pos in robot_action.items():
                motor_name = key.removesuffix('.pos')
                if motor_name in current_positions:
                    current_pos = current_positions[motor_name]
                    error = ref_pos - current_pos
                    
                    # P control: output = Kp * error
                    control_output = kp * error
                    
                    # Convert control output to position command
                    new_position = current_pos + control_output
                    robot_action[key] = new_position
                    if target_action["enabled"]:
                        print(f"Observed {motor_name}: {current_pos}")
                        print(f"Set command {motor_name}: {new_position}")

            # Send action to robot
            if robot_action:
                robot.send_action(robot_action)
            
            busy_wait(max(1.0 / fps - (time.perf_counter() - t0), 0.0))
            
        except KeyboardInterrupt:
            print("User interrupted program")
            break
        except Exception as e:
            print(f"P control loop error: {e}")
            traceback.print_exc()
            break

def main():
    """Main function"""
    print("LeRobot Simplified Keyboard Control Example (P Control)")
    print("="*50)
    
    try:        
        # Get port
        port = input("Please enter SO100 robot USB port (e.g.: /dev/xlerobot_right_head or /dev/xlerobot_left_base): ").strip()
        # If directly press enter, use default port
        if not port:
            port = "/dev/xlerobot_right_head"
            print(f"Using default port: {port}")
        else:
            print(f"Connecting to port: {port}")

        # Get id
        id = input("Please enter SO100 robot id (e.g.: xlerobot_right_only or xlerobot_left_only): ").strip()
        if not id:
            id = "xlerobot_right_only"
            print(f"Using default id: {id}")
        else:
            print(f"Using id: {id}")
        
        # Configure robot
        robot_config = SO100FollowerConfig(port=port, id=id, use_degrees=True)
        robot = SO100Follower(robot_config)
        
        # Configure keyboard
        keyboard_config = KeyboardTeleopConfig()
        keyboard = KeyboardTeleop(keyboard_config)
        
        # Connect devices
        robot.connect()
        keyboard.connect()
        
        print("Device connection successful!")
        print(f"motor bus info: {robot.bus.motors}")
        # Ask whether to recalibrate
        while True:
            calibrate_choice = input("Do you want to recalibrate the robot? (y/n): ").strip().lower()
            if calibrate_choice in ['y', 'yes']:
                print("Starting recalibration...")
                robot.calibrate()
                print("Calibration completed!")
                break
            elif calibrate_choice in ['n', 'no']:
                print("Using previous calibration file")
                break
            else:
                print("Please enter y or n")

        # Initialize target positions as current positions (integers)
        target_positions = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
            } 
        
        # Initialize robot kinematics
        joint_names_wo_gripper = [j for j in robot.bus.motors.keys() if j != 'gripper']
        kinematics_solver = RobotKinematics(
            urdf_path="/home/that/SO-ARM100/Simulation/SO101/so101_new_calib.urdf", 
            target_frame_name="gripper_frame_link",
            joint_names=joint_names_wo_gripper,
            )
        print(f"joint names for kinematics: {joint_names_wo_gripper}")
        
        # Read initial joint angles
        print("Reading initial joint angles...")
        start_obs = robot.get_observation()
        start_positions = {}
        for key, value in start_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = value 
        
        print("Initial joint angles:")
        for joint_name, position in start_positions.items():
            print(f"  {joint_name}: {position}°")

        # Move to zero position
        move_to_target_with_ipol(robot, duration=5.0, control_freq=200.0,
                               max_vel_per_joint=np.array([15, 15, 15, 15, 15, 10]),
                               max_acc_per_joint=np.array([30, 30, 30, 30, 30, 20]))

        print("Keyboard control instructions:")
        print("- Q/A: Control end effector x coordinate in base frame (+/-))")
        print("- W/S: Control end effector y coordinate in base frame (+/-)")
        print("- E/D: Control end effector z coordinate in base frame (+/-)")
        print("- Z/H: Control end effector wx coordinate in tool frame (+/-)")
        print("- U/J: Control end effector wy coordinate in tool frame (+/-)")
        print("- I/K: Control end effector wz coordinate in tool frame (+/-)")
        print("- R/T: Close/open gripper")
        print("- X: Exit program (return to start position first)")
        print("="*50)
        print("Note: Robot will continuously move to target positions")
        
        keyboard_teleop(
            robot,
            keyboard,
            kinematics_solver,
            start_positions,
            kp=0.5, fps=50
        )
        
        # Disconnect
        robot.disconnect()
        keyboard.disconnect()
        print("Program ended")
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        print("Please check:")
        print("1. Whether the robot is properly connected")
        print("2. Whether the USB port is correct")
        print("3. Whether you have sufficient permissions to access USB devices")
        print("4. Whether the robot is properly configured")

if __name__ == "__main__":
    main() 