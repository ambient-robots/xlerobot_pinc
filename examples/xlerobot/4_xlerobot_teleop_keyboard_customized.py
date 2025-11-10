# To Run on the host
'''python
PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot
'''

# To Run the teleop:
'''python
PYTHONPATH=src python -m examples.xlerobot.teleoperate_Keyboard
'''

import time
import numpy as np
import math

from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator

from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
# from lerobot.robots.xlerobot import XLerobotClient, XLerobotClientConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
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
from lerobot.model.SO101Robot import SO101Kinematics
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

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
    'wz+': '+', 'wz-': '#',
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

# Head motor mapping
HEAD_MOTOR_MAP = {
    "head_pan": "head_pan",
    "head_tilt": "head_tilt",
}

class SimpleHeadControl:
    def __init__(self, initial_obs, kp=0.5):
        self.kp = kp
        self.degree_step = 1
        # Initialize head motor positions
        self.target_positions = {
            "head_pan": initial_obs.get("head_pan.pos", 0.0),
            "head_tilt": initial_obs.get("head_tilt.pos", 0.0),
        }
        self.zero_pos = {"head_pan": 0.0, "head_tilt": 0.0}

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
            target_positions = self.zero_pos.copy()
        
        # 1) Read current joint positions (calibrated if provided)
        obs = robot.bus_right_head.sync_read("Present_Position", robot.head_motors)

        # 2) choose names in the order of zero_positions, filter to those present in obs
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
        print("Reached zero pos of with ipol trajectory.")

    def handle_keys(self, key_state):
        if not any(key_state.values()):
            return
        
        if key_state.get('head_pan+'):
            self.target_positions["head_pan"] += self.degree_step
            print(f"[HEAD] head_pan: {self.target_positions['head_pan']}")
        if key_state.get('head_pan-'):
            self.target_positions["head_pan"] -= self.degree_step
            print(f"[HEAD] head_pan: {self.target_positions['head_pan']}")
        if key_state.get('head_tilt+'):
            self.target_positions["head_tilt"] += self.degree_step
            print(f"[HEAD] head_tilt: {self.target_positions['head_tilt']}")
        if key_state.get('head_tilt-'):
            self.target_positions["head_tilt"] -= self.degree_step
            print(f"[HEAD] head_tilt: {self.target_positions['head_tilt']}")
        print(f"head ref positions: {self.target_positions}")

    def p_control_action(self, robot):
        obs = robot.bus_right_head.sync_read("Present_Position", robot.head_motors)
        action = {}
        for motor in self.target_positions:
            current_obs = obs.get(f"{HEAD_MOTOR_MAP[motor]}", 0.0)
            error = self.target_positions[motor] - current_obs
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current_obs + control
        print(f"head commanded actions: {action}")
        return action

class SimpleTeleopArm:
    def __init__(self, joint_map, initial_obs, prefix="left", kp=0.5):
        #self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp

        # Set target positions to zero for P control
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": -90.0,
            "gripper": 0.0,
        }

        # Initial joint positions
        self.zero_pos = {
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
        self.keyboard_to_ee_pose = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
            [   
                EEReferenceAndDelta(
                    kinematics=self.kinematics,
                    end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
                    motor_names=list(self.target_positions.keys()),
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
        self.ee_to_robot_joints = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
            [
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
    
        # Set the degree step and pos step in [m]
        self.degree_step = 0.01
        self.pos_step = 0.003
        self.gripper_vel_step = 1

        self.ref_action_when_disabled = None

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
            target_positions = self.zero_pos.copy()
        
        # 1) Read current joint positions (calibrated if provided)
        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        obs = {j: obs_raw[f"{self.joint_map[j]}"] for j in self.joint_map}

        # 2) choose names in the order of zero_positions, filter to those present in obs
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
        self.ref_action_when_disabled = None
        print("Reached target pos of with ipol trajectory.")

    def handle_keys(self, robot, key_state):
        if not any(key_state.values()):
            return

        if self.prefix=="left":
            obs_raw = robot.bus_left_base.sync_read("Present_Position", robot.left_arm_motors)
        else:
            obs_raw = robot.bus_right_head.sync_read("Present_Position", robot.right_arm_motors)
        current_obs = {f"{j}.pos": obs_raw[f"{self.joint_map[j]}"] for j in self.joint_map}

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

        if key_state.get('x+'):
            target_action["target_x"] = self.pos_step
        elif key_state.get('x-'):
            target_action["target_x"] = -self.pos_step
        elif key_state.get('y+'):
            target_action["target_y"] = self.pos_step
        elif key_state.get('y-'):
            target_action["target_y"] = -self.pos_step
        elif key_state.get('z+'):
            target_action["target_z"] = self.pos_step
        elif key_state.get('z-'):
            target_action["target_z"] = -self.pos_step
        elif key_state.get('wx+'):
            target_action["target_wx"] = self.degree_step
        elif key_state.get('wx-'):
            target_action["target_wx"] = -self.degree_step
        elif key_state.get('wy+'):
            target_action["target_wy"] = self.degree_step
        elif key_state.get('wy-'):
            target_action["target_wy"] = -self.degree_step
        elif key_state.get('wz+'):
            target_action["target_wz"] = self.degree_step
        elif key_state.get('wz-'):
            target_action["target_wz"] = -self.degree_step
        elif key_state.get('gripper+'):
            target_action["gripper_vel"] = self.gripper_vel_step
        elif key_state.get('gripper-'):
            target_action["gripper_vel"] = -self.gripper_vel_step
        print(f"{self.prefix}_arm relative actions: {target_action}")

        desired_ee_pos = self.keyboard_to_ee_pose((target_action, current_obs))
        if target_action["enabled"]:
            ref_action = self.ee_to_robot_joints((desired_ee_pos, current_obs))
            self.ref_action_when_disabled = ref_action.copy()
        else:
            if self.ref_action_when_disabled is None:
                self.ref_action_when_disabled = current_obs.copy()
            ref_action = self.ref_action_when_disabled.copy()
        
        for key, ref_pos in ref_action.items():
            self.target_positions[key.removesuffix('.pos')] = ref_pos
        print(f"{self.prefix}_arm ref positions: {self.target_positions}")

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
        
        print(f"{self.prefix}_arm commanded actions: {action}")
        return action
    

def main():
    # Teleop parameters
    FPS = 50
    # ip = "192.168.1.123"  # This is for zmq connection
    ip = "localhost"  # This is for local/wired connection
    robot_name = "ambient_xlerobot"

    # For zmq connection
    # robot_config = XLerobotClientConfig(remote_ip=ip, id=robot_name)
    # robot = XLerobotClient(robot_config)    

    # For local/wired connection
    robot_config = XLerobotConfig(id=robot_name, use_degrees=True)
    robot = XLerobot(robot_config)
    
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(robot_config)
        print(robot)
        return
    
    print("Device connection successful!")
    print(f"motor bus_left_base info: {robot.bus_left_base.motors}")
    print(f"motor bus_right_head info: {robot.bus_right_head.motors}")

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
        
    # init_rerun(session_name="xlerobot_teleop_v2")

    #Init the keyboard instance
    keyboard_config = KeyboardTeleopConfig()
    keyboard = KeyboardTeleop(keyboard_config)
    keyboard.connect()

    # Init the arm and head instances
    obs = robot.get_observation()
    left_arm_teleop = SimpleTeleopArm(LEFT_JOINT_MAP, obs, prefix="left")
    right_arm_tekeop = SimpleTeleopArm(RIGHT_JOINT_MAP, obs, prefix="right")
    head_teleop = SimpleHeadControl(obs)

    # Move both arms and head to zero position at start
    left_arm_target_positions = {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": -90.0,
        "gripper": 0.0,
    }
    right_arm_target_positions = left_arm_target_positions.copy()
    left_arm_teleop.move_to_target_with_ipol(robot, target_positions=left_arm_target_positions)
    right_arm_tekeop.move_to_target_with_ipol(robot, target_positions=right_arm_target_positions)

    try:
        while True:
            start = time.perf_counter()

            pressed_keys = set(keyboard.get_action().keys())
            print(f"[MAIN] Pressed Keys: {pressed_keys}")
            if '^' in pressed_keys:
                left_arm_teleop.move_to_target_with_ipol(robot)
                right_arm_tekeop.move_to_target_with_ipol(robot)
                head_teleop.move_to_target_with_ipol(robot)
                print("User exited program")
                break

            left_key_state = {action: (key in pressed_keys) for action, key in LEFT_KEYMAP.items()}
            right_key_state = {action: (key in pressed_keys) for action, key in RIGHT_KEYMAP.items()}
            head_key_state = {action: (key in pressed_keys) for action, key in HEAD_KEYMAP.items()}

            # Handle reset for left arm
            if left_key_state.get('reset'):
                left_arm_teleop.move_to_target_with_ipol(robot)
                continue  

            # Handle reset for right arm
            if right_key_state.get('reset'):
                right_arm_tekeop.move_to_target_with_ipol(robot)
                continue

            # Handle reset for head motors with '?'
            if head_key_state.get('reset'):
                head_teleop.move_to_target_with_ipol(robot)
                continue

            left_arm_teleop.handle_keys(robot, left_key_state)
            right_arm_tekeop.handle_keys(robot, right_key_state)
            head_teleop.handle_keys(head_key_state)

            left_action = left_arm_teleop.p_control_action(robot)
            print(f"left action: {left_action}")
            right_action = right_arm_tekeop.p_control_action(robot)
            print(f"right action: {right_action}")
            head_action = head_teleop.p_control_action(robot)
            print(f"head action: {head_action}")

            # Base action
            keyboard_keys = np.array(list(pressed_keys))
            base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
            print(f"base action: {base_action}")

            action = {**left_action, **right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            for k in robot.action_features:
                print(f"[MAIN] Observation: {k}: {obs[k]}")
            # print(f"[MAIN] Observation: {obs}")
            # log_rerun_data(obs, action)

            dt_ms = (time.perf_counter() - start) * 1e3
            print(f"control delay: {dt_ms:.1f}ms")
            busy_wait(max(1.0 / FPS - (time.perf_counter() - start), 0.0))
            # busy_wait(1.0 / FPS)
    except KeyboardInterrupt:
            print("User interrupted program")
    finally:
        robot.disconnect()
        keyboard.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()
