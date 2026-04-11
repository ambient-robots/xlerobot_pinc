#!/usr/bin/env python3
"""
VR control for XLerobot robot
Uses handle_vr_input with delta action control
"""

# Standard library imports
from collections import deque
import math
import time
import logging
import traceback
import numpy as np
import concurrent.futures

from lerobot.utils.quadratic_spline_via_ipol import Via, Limits, QuadraticSplineInterpolator
from lerobot.utils.robot_utils import precise_sleep
from lerobot.robots.xlerobot_pinc import XLerobotPincConfig, XLerobotPinc

from openpi_client import rtc as rtc_utils
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

FPS = 60
LOCAL_INFERENCE = False

# Real-time chunking parameters
ACTION_HORIZON = 50
MIN_EXECUTE_HORIZON = 12
DELAY_BUFFER_SIZE = 10
DEFAULT_DELAY_STEPS = 8
WARMUP_CYCLES_TO_IGNORE = 2
DELAY_ESTIMATE_PERCENTILE = 90
DELAY_ESTIMATE_SAFETY_STEPS = 1
PREFIX_ATTENTION_SCHEDULE = "exp"
MAX_GUIDANCE_WEIGHT = 4.0
NUM_STEPS = 5
PRINT_RTC_TIMING = True

class SimpleControlArm:
    def __init__(self, joint_map, initial_obs, prefix="left", kp=0.85):
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


def choose_execute_horizon(estimated_delay: int) -> int:
    if estimated_delay < 0:
        raise ValueError(f"estimated_delay must be non-negative, got {estimated_delay}")
    if estimated_delay > ACTION_HORIZON // 2:
        raise ValueError(
            "estimated_delay violates the RTC constraint d <= s <= H - d: "
            f"delay={estimated_delay}, action_horizon={ACTION_HORIZON}"
        )
    return min(max(estimated_delay, MIN_EXECUTE_HORIZON), ACTION_HORIZON - estimated_delay)


def _control_period_ms() -> float:
    return 1e3 / FPS


def _delay_steps_from_ms(delay_ms: float) -> int:
    # Round up to the next control step so the configured delay is conservative.
    return max(0, math.ceil(delay_ms / _control_period_ms()))


def summarize_delay_history(delay_history: deque[int]) -> dict[str, float] | None:
    if not delay_history:
        return None

    delays = np.asarray(delay_history, dtype=float)
    return {
        "count": float(delays.size),
        "min": float(np.min(delays)),
        "median": float(np.median(delays)),
        "p95": float(np.percentile(delays, 95)),
        "max": float(np.max(delays)),
    }


def estimate_delay_steps(delay_history: deque[int], *, default: int) -> int:
    if not delay_history:
        return default

    delays = np.asarray(delay_history, dtype=float)
    percentile_delay = float(np.percentile(delays, DELAY_ESTIMATE_PERCENTILE))
    estimated_delay = int(math.ceil(percentile_delay)) + DELAY_ESTIMATE_SAFETY_STEPS
    estimated_delay = max(default, estimated_delay)
    return min(estimated_delay, ACTION_HORIZON // 2)


def print_rtc_timing(
    *,
    label: str,
    request_start_t: float,
    request_done_t: float,
    estimated_delay: int | None,
    execute_horizon: int,
    executed_prefix: int | None = None,
    overran_horizon: bool = False,
    response: dict | None = None,
    delay_history: deque | None = None,
    history_note: str | None = None,
) -> None:
    if not PRINT_RTC_TIMING:
        return

    wall_ms = (request_done_t - request_start_t) * 1e3
    wall_steps = wall_ms / _control_period_ms()
    suggested_delay_steps = _delay_steps_from_ms(wall_ms)

    message = (
        f"[RTC][{label}] wall={wall_ms:.1f} ms ({wall_steps:.2f} steps @ {FPS} Hz) | "
        f"suggested_delay_steps={suggested_delay_steps} | execute_horizon={execute_horizon}"
    )
    if estimated_delay is not None:
        message += f" | estimated_delay={estimated_delay}"
    if executed_prefix is not None:
        message += f" | executed_before_ready={executed_prefix}"

    if response is not None:
        policy_timing = response.get("policy_timing", {})
        server_timing = response.get("server_timing", {})
        policy_ms = policy_timing.get("infer_ms")
        server_ms = server_timing.get("infer_ms")
        if policy_ms is not None:
            message += f" | policy_infer={policy_ms:.1f} ms"
        if server_ms is not None:
            message += f" | server_infer={server_ms:.1f} ms"

    print(message)

    if delay_history is not None:
        print(f"[RTC][{label}] recent_delay_history={list(delay_history)}")
        delay_stats = summarize_delay_history(delay_history)
        if delay_stats is not None:
            next_estimate = estimate_delay_steps(delay_history, default=DEFAULT_DELAY_STEPS)
            print(
                f"[RTC][{label}] delay_stats count={int(delay_stats['count'])} "
                f"min={delay_stats['min']:.0f} median={delay_stats['median']:.1f} "
                f"p95={delay_stats['p95']:.1f} max={delay_stats['max']:.0f} "
                f"next_estimate={next_estimate}"
            )
    if history_note is not None:
        print(f"[RTC][{label}] history_note={history_note}")
    if overran_horizon:
        print(
            f"[RTC][{label}] WARNING: inference exceeded execute_horizon={execute_horizon}. "
            f"Consider increasing MIN_EXECUTE_HORIZON / DEFAULT_DELAY_STEPS or reducing latency."
        )


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
        metadata = client.get_server_metadata()
        if "infer_realtime_chunk" not in metadata.get("supported_methods", []):
            raise RuntimeError("Server does not support infer_realtime_chunk")

        # Initiailize events
        events = {
            "stop_inference": False,   # Press key q: Stop inference
            "reset_episode": False,    # Press key w: reset episode
        }
        print("✅ Policy server ready")
        if PRINT_RTC_TIMING:
            print(f"[RTC] control_period={_control_period_ms():.1f} ms at FPS={FPS}")
            print(
                f"[RTC] delay_estimator=p{DELAY_ESTIMATE_PERCENTILE}+{DELAY_ESTIMATE_SAFETY_STEPS} "
                f"| warmup_cycles_ignored={WARMUP_CYCLES_TO_IGNORE} | delay_buffer_size={DELAY_BUFFER_SIZE}"
            )

        print("Starting inference loop...")
        inference_episodes = 0
        while inference_episodes < NUM_EPISODES and not events["stop_inference"]:
            start_episode_t = time.perf_counter()
            timestamp = 0
            input("🔧Press ENTER to start the next episode ...")
            print(f"✅ Start episode: {inference_episodes}")

            bootstrap_obs = build_policy_observation(robot)
            bootstrap_execute_horizon = choose_execute_horizon(DEFAULT_DELAY_STEPS)
            bootstrap_start_t = time.perf_counter()
            bootstrap = client.infer_realtime_chunk(
                bootstrap_obs,
                execute_horizon=bootstrap_execute_horizon,
                prefix_attention_schedule=PREFIX_ATTENTION_SCHEDULE,
                max_guidance_weight=MAX_GUIDANCE_WEIGHT,
                num_steps=NUM_STEPS,
            )
            bootstrap_done_t = time.perf_counter()
            print_rtc_timing(
                label="bootstrap",
                request_start_t=bootstrap_start_t,
                request_done_t=bootstrap_done_t,
                estimated_delay=DEFAULT_DELAY_STEPS,
                execute_horizon=bootstrap_execute_horizon,
                response=bootstrap,
            )
            current_external = bootstrap["actions"]
            current_internal = bootstrap["rtc_internal_actions"]
            observed_delays = deque(maxlen=DELAY_BUFFER_SIZE)
            completed_cycles = 0

            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                while timestamp < EPISODE_TIME_SEC:
                    pressed_keys = set(keyboard.get_action().keys())
                    if 'q' in pressed_keys:
                        events["stop_inference"] = True
                        break
                    if 'w' in pressed_keys:
                        events["reset_episode"] = True
                        break
                    
                    observation = build_policy_observation(robot)
                    estimated_delay = estimate_delay_steps(observed_delays, default=DEFAULT_DELAY_STEPS)
                    execute_horizon = choose_execute_horizon(estimated_delay)

                    request_start_t = time.perf_counter()
                    future = executor.submit(
                        client.infer_realtime_chunk,
                        observation,
                        prev_internal_actions=current_internal,
                        inference_delay=estimated_delay,
                        execute_horizon=execute_horizon,
                        prefix_attention_schedule=PREFIX_ATTENTION_SCHEDULE,
                        max_guidance_weight=MAX_GUIDANCE_WEIGHT,
                        num_steps=NUM_STEPS,
                    )

                    executed_prefix = 0

                    while executed_prefix < execute_horizon and not future.done():
                        pressed_keys = set(keyboard.get_action().keys())
                        if "q" in pressed_keys:
                            events["stop_inference"] = True
                            break
                        if "w" in pressed_keys:
                            events["reset_episode"] = True
                            break

                        start_loop_t = time.perf_counter()
                        target = current_external[executed_prefix]
                        execute_target(robot, left_arm, right_arm, target)
                        dt_s = time.perf_counter() - start_loop_t
                        precise_sleep(1 / FPS - dt_s)
                        executed_prefix += 1

                    if events["stop_inference"] or events["reset_episode"]:
                        break

                    overran_horizon = executed_prefix >= execute_horizon and not future.done()
                    next_result = future.result()
                    request_done_t = time.perf_counter()
                    next_external = next_result["actions"]
                    next_internal = next_result["rtc_internal_actions"]

                    actual_delay = min(executed_prefix, execute_horizon)
                    history_note = None
                    if completed_cycles < WARMUP_CYCLES_TO_IGNORE:
                        history_note = (
                            f"skipped_delay_sample={actual_delay} because cycle "
                            f"{completed_cycles + 1} is within warmup ignore window"
                        )
                    elif overran_horizon:
                        history_note = (
                            f"skipped_delay_sample={actual_delay} because this cycle overran "
                            f"execute_horizon={execute_horizon}"
                        )
                    else:
                        observed_delays.append(actual_delay)

                    print_rtc_timing(
                        label="cycle",
                        request_start_t=request_start_t,
                        request_done_t=request_done_t,
                        estimated_delay=estimated_delay,
                        execute_horizon=execute_horizon,
                        executed_prefix=executed_prefix,
                        overran_horizon=overran_horizon,
                        response=next_result,
                        delay_history=observed_delays,
                        history_note=history_note,
                    )
                    completed_cycles += 1

                    hybrid_chunk = rtc_utils.splice_action_chunks(
                        current_external,
                        next_external,
                        inference_delay=actual_delay,
                        execute_horizon=execute_horizon,
                    )

                    for t in range(actual_delay, execute_horizon):
                        pressed_keys = set(keyboard.get_action().keys())
                        if "q" in pressed_keys:
                            events["stop_inference"] = True
                            break
                        if "w" in pressed_keys:
                            events["reset_episode"] = True
                            break

                        start_loop_t = time.perf_counter()
                        target = hybrid_chunk[t]
                        execute_target(robot, left_arm, right_arm, target)
                        dt_s = time.perf_counter() - start_loop_t
                        precise_sleep(1 / FPS - dt_s)

                    if events["stop_inference"] or events["reset_episode"]:
                        break

                    current_external = rtc_utils.shift_chunk(next_external, execute_horizon)
                    current_internal = rtc_utils.shift_chunk(next_internal, execute_horizon)
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
