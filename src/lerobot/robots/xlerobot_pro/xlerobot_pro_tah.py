#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from itertools import chain
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_xlerobot_pro_tah import XLerobotProTAHConfig

logger = logging.getLogger(__name__)


class XLerobotProTAH(Robot):
    config_class = XLerobotProTAHConfig
    name = "xlerobot_pro_tah"

    def __init__(self, config: XLerobotProTAHConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        if self.calibration.get("left_arm_shoulder_pan") is not None:
            calibration_left_base = {
                "left_arm_shoulder_pan": self.calibration.get("left_arm_shoulder_pan"),
                "left_arm_shoulder_lift": self.calibration.get("left_arm_shoulder_lift"),
                "left_arm_elbow_flex": self.calibration.get("left_arm_elbow_flex"),
                "left_arm_elbow_roll": self.calibration.get("left_arm_elbow_roll"),
                "left_arm_wrist_flex": self.calibration.get("left_arm_wrist_flex"),
                "left_arm_wrist_roll": self.calibration.get("left_arm_wrist_roll"),
                "left_arm_gripper": self.calibration.get("left_arm_gripper"),
            }
        else:
            calibration_left_base = self.calibration
        
        self.bus_left_base = FeetechMotorsBus(
            port=self.config.port_left_base,
            motors={
                # left arm
                "left_arm_shoulder_pan": Motor(1, "sts3250", norm_mode_body),
                "left_arm_shoulder_lift": Motor(2, "sts3250", norm_mode_body),
                "left_arm_elbow_flex": Motor(3, "sts3250", norm_mode_body),
                "left_arm_elbow_roll": Motor(4, "sts3215", norm_mode_body),
                "left_arm_wrist_flex": Motor(5, "sts3215", norm_mode_body),
                "left_arm_wrist_roll": Motor(6, "sts3215", norm_mode_body),
                "left_arm_gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=calibration_left_base,
        )
        if self.calibration.get("right_arm_shoulder_pan") is not None:
            calibration_right_head = {
                "right_arm_shoulder_pan": self.calibration.get("right_arm_shoulder_pan"),
                "right_arm_shoulder_lift": self.calibration.get("right_arm_shoulder_lift"),
                "right_arm_elbow_flex": self.calibration.get("right_arm_elbow_flex"),
                "right_arm_elbow_roll": self.calibration.get("right_arm_elbow_roll"),
                "right_arm_wrist_flex": self.calibration.get("right_arm_wrist_flex"),
                "right_arm_wrist_roll": self.calibration.get("right_arm_wrist_roll"),
                "right_arm_gripper": self.calibration.get("right_arm_gripper"),
                "head_pan": self.calibration.get("head_pan"),
                "head_tilt": self.calibration.get("head_tilt"),
            }
        else:
            calibration_right_head = self.calibration
            
        self.bus_right_head= FeetechMotorsBus(
            port=self.config.port_right_head,
            motors={
                # right arm
                "right_arm_shoulder_pan": Motor(1, "sts3250", norm_mode_body),
                "right_arm_shoulder_lift": Motor(2, "sts3250", norm_mode_body),
                "right_arm_elbow_flex": Motor(3, "sts3250", norm_mode_body),
                "right_arm_elbow_roll": Motor(4, "sts3215", norm_mode_body),
                "right_arm_wrist_flex": Motor(5, "sts3215", norm_mode_body),
                "right_arm_wrist_roll": Motor(6, "sts3215", norm_mode_body),
                "right_arm_gripper": Motor(7, "sts3215", MotorNormMode.RANGE_0_100),
                # head
                "head_pan": Motor(8, "sts3215", norm_mode_body),
                "head_tilt": Motor(9, "sts3215", norm_mode_body),
            },
            calibration=calibration_right_head,
        )
        self.left_arm_motors = [motor for motor in self.bus_left_base.motors if motor.startswith("left_arm")]
        self.right_arm_motors = [motor for motor in self.bus_right_head.motors if motor.startswith("right_arm")]
        self.head_motors = [motor for motor in self.bus_right_head.motors if motor.startswith("head")]
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "left_arm_shoulder_pan.pos",
                "left_arm_shoulder_lift.pos",
                "left_arm_elbow_flex.pos",
                "left_arm_elbow_roll.pos",
                "left_arm_wrist_flex.pos",
                "left_arm_wrist_roll.pos",
                "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos",
                "right_arm_shoulder_lift.pos",
                "right_arm_elbow_flex.pos",
                "right_arm_elbow_roll.pos",
                "right_arm_wrist_flex.pos",
                "right_arm_wrist_roll.pos",
                "right_arm_gripper.pos",
                "head_pan.pos",
                "head_tilt.pos",
            ),
            float,
        )

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self.bus_left_base.is_connected and self.bus_right_head.is_connected and all(
            cam.is_connected for cam in self.cameras.values()
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus_left_base.connect()
        self.bus_right_head.connect()

        # Check if calibration file exists and ask user if they want to restore it
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            user_input = input(
                f"Press ENTER to restore calibration from file, or type 'c' and press ENTER to run manual calibration: "
            )
            if user_input.strip().lower() != "c":
                try:
                    is_gripper_updated = False
                    fixed_range_motor = "left_arm_gripper"
                    calibration_left_base = self.bus_left_base.read_calibration()
                    if self.bus_left_base.calibration[fixed_range_motor] != calibration_left_base[fixed_range_motor]:
                        logger.info(f"Updating saved calibration of {fixed_range_motor} from device")
                        self.bus_left_base.calibration[fixed_range_motor].homing_offset = calibration_left_base[fixed_range_motor].homing_offset
                        self.calibration[fixed_range_motor].homing_offset = calibration_left_base[fixed_range_motor].homing_offset
                        is_gripper_updated = True
                    if set(calibration_left_base) != set(self.bus_left_base.calibration):
                        logger.debug(f"Calibrations mismatched at port: {self.bus_left_base.port}")
                        logger.debug(f"Calibrations from device at port {self.bus_left_base.port}: {calibration_left_base}")
                        logger.debug(f"Calibrations from saved config at port {self.bus_left_base.port}: {self.bus_left_base.calibration}")
                        logger.info(f"Writing calibration file associated with the id {self.id} to the motors at the port {self.bus_left_base.port}")
                        calib_to_write = {k: v for k, v in self.bus_left_base.calibration.items() if k != fixed_range_motor}
                        self.bus_left_base.write_calibration(calib_to_write, cache=False)
                    
                    fixed_range_motor = "right_arm_gripper"
                    calibration_right_head = self.bus_right_head.read_calibration()
                    if self.bus_right_head.calibration[fixed_range_motor] != calibration_right_head[fixed_range_motor]:
                        logger.info(f"Updating saved calibration of {fixed_range_motor} from device")
                        self.bus_right_head.calibration[fixed_range_motor].homing_offset = calibration_right_head[fixed_range_motor].homing_offset
                        self.calibration[fixed_range_motor].homing_offset = calibration_right_head[fixed_range_motor].homing_offset
                        is_gripper_updated = True
                    if set(calibration_right_head) != set(self.bus_right_head.calibration):
                        logger.debug(f"Calibrations mismatched at port: {self.bus_right_head.port}")
                        logger.debug(f"Calibrations from device at port {self.bus_right_head.port}: {calibration_right_head}")
                        logger.debug(f"Calibrations from saved config at port {self.bus_right_head.port}: {self.bus_right_head.calibration}")
                        logger.info(f"Writing calibration file associated with the id {self.id} to the motors at port {self.bus_right_head.port}")
                        calib_to_write = {k: v for k, v in self.bus_right_head.calibration.items() if k != fixed_range_motor}
                        self.bus_right_head.write_calibration(calib_to_write, cache=False)

                    if is_gripper_updated:
                        self._save_calibration()
                        logger.info(f"Updated calibration file saved to {self.calibration_fpath} after gripper calibration update")
                    
                except Exception as e:
                    logger.warning(f"Failed to restore calibration from file: {e}")
                    logger.info("Proceeding with manual calibration...")
                    self.calibrate()
            else:
                self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus_left_base.is_calibrated and self.bus_right_head.is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        ## calib right motors
        right_motors = self.right_arm_motors + self.head_motors
        self.bus_right_head.disable_torque()
        for motor in right_motors:
            self.bus_right_head.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        fixed_range_motor = "right_arm_gripper"
        unknown_range_motors = [motor for motor in right_motors if motor != fixed_range_motor]
        homing_offsets = {}
        for motor in unknown_range_motors:
            input(
                f"Move '{motor}' to the middle of its range of motion and press ENTER...."
            )
            self.bus_right_head.enable_torque(motor)
            homing_offsets.update(self.bus_right_head.set_half_turn_homings(motor))
        homing_offsets[fixed_range_motor] = self.bus_right_head.read("Homing_Offset", fixed_range_motor, normalize=False)
        input("Hold the right arm. Press ENTER when ready...")
        self.bus_right_head.disable_torque()

        print(
            f"Move all right arm and head joints (except '{fixed_range_motor}') sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus_right_head.record_ranges_of_motion(unknown_range_motors)
        range_mins[fixed_range_motor] = 548 # 2048 -1500
        range_maxes[fixed_range_motor] = 2048
        
        calibration_right_head = {}
        for motor, m in self.bus_right_head.motors.items():
            calibration_right_head[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        calib_to_write = {k: v for k, v in calibration_right_head.items() if k != fixed_range_motor}
        self.bus_right_head.write_calibration(calib_to_write, cache=False)
        self.bus_right_head.calibration = calibration_right_head
        
        # calib left motors
        self.bus_left_base.disable_torque(self.left_arm_motors)
        for motor in self.left_arm_motors:
            self.bus_left_base.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        
        fixed_range_motor = "left_arm_gripper"
        unknown_range_motors = [motor for motor in self.left_arm_motors if motor != fixed_range_motor]
        homing_offsets = {}
        for motor in unknown_range_motors:
            input(
                f"Move '{motor}' to the middle of its range of motion and press ENTER...."
            )
            self.bus_left_base.enable_torque(motor)
            homing_offsets.update(self.bus_left_base.set_half_turn_homings(motor))
        homing_offsets[fixed_range_motor] = self.bus_left_base.read("Homing_Offset", fixed_range_motor, normalize=False)
        input("Hold the left arm. Press ENTER when ready...")
        self.bus_left_base.disable_torque()
        
        print(
            f"Move all left arm joints except '{fixed_range_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus_left_base.record_ranges_of_motion(unknown_range_motors)
        range_mins[fixed_range_motor] = 548 # 2048 -1500
        range_maxes[fixed_range_motor] = 2048
        
        calibration_left_base = {}
        for motor, m in self.bus_left_base.motors.items():
            calibration_left_base[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )
        calib_to_write = {k: v for k, v in calibration_left_base.items() if k != fixed_range_motor}
        self.bus_left_base.write_calibration(calib_to_write, cache=False)
        self.bus_left_base.calibration = calibration_left_base

        self.calibration = {**calibration_left_base, **calibration_right_head}
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)
        

    def configure(self):
        # Set-up arm actuators (position mode)
        # We assume that at connection time, arm is in a rest position,
        # and torque can be safely disabled to run calibration        
        self.bus_left_base.disable_torque()
        self.bus_right_head.disable_torque()
        self.bus_left_base.configure_motors()
        self.bus_right_head.configure_motors()
        
        for motor in self.left_arm_motors:
            self.bus_left_base.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus_left_base.write("P_Coefficient", motor, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus_left_base.write("I_Coefficient", motor, 0)
            self.bus_left_base.write("D_Coefficient", motor, 32)
            if motor == "left_arm_gripper":
                self.bus_left_base.write("Max_Torque_Limit", motor, 800)  # 80% of the max torque limit to avoid burnout
                self.bus_left_base.write("Torque_Limit", motor, 800)  # 80% of the max torque limit to avoid burnout
                self.bus_left_base.write("Protection_Current", motor, 250)  # 50% of max current to avoid burnout
                self.bus_left_base.write("Overload_Torque", motor, 80)  # 80% torque when overloaded
                self.bus_left_base.write("Protection_Time", motor, 2) # 20ms overload protection time
                self.bus_left_base.write("Protective_Torque", motor, 20) # 20% torque when overload is detected
                self.bus_left_base.write("Acceleration", motor, 25) # 25*100 steps/s^2 = 220 deg/s^2 goal accelertion
        
        for motor in self.right_arm_motors:
            self.bus_right_head.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus_right_head.write("P_Coefficient", motor, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus_right_head.write("I_Coefficient", motor, 0)
            self.bus_right_head.write("D_Coefficient", motor, 32)
            if motor == "right_arm_gripper":
                self.bus_right_head.write("Max_Torque_Limit", motor, 800)  # 80% of the max torque limit to avoid burnout
                self.bus_right_head.write("Torque_Limit", motor, 800)  # 80% of the max torque limit to avoid burnout
                self.bus_right_head.write("Protection_Current", motor, 250)  # 50% of max current to avoid burnout
                self.bus_right_head.write("Overload_Torque", motor, 80)  # 80% torque when overloaded
                self.bus_right_head.write("Protection_Time", motor, 2) # 20ms overload protection time
                self.bus_right_head.write("Protective_Torque", motor, 20) # 20% torque when overload is detected
                self.bus_right_head.write("Acceleration", motor, 25) # 25*100 steps/s^2 = 220 deg/s^2 goal accelertion
        
        for motor in self.head_motors:
            self.bus_right_head.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus_right_head.write("P_Coefficient", motor, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus_right_head.write("I_Coefficient", motor, 0)
            self.bus_right_head.write("D_Coefficient", motor, 32)


    def setup_motors(self) -> None:
        for motor in chain(reversed(self.left_arm_motors)):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus_left_base.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus_left_base.motors[motor].id}")
        
        # Set up right arm motors
        for motor in chain(reversed(self.right_arm_motors), reversed(self.head_motors)):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus_right_head.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus_right_head.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read actuators position for arm and vel for base
        start = time.perf_counter()
        left_arm_pos = self.bus_left_base.sync_read("Present_Position", self.left_arm_motors)
        right_arm_pos = self.bus_right_head.sync_read("Present_Position", self.right_arm_motors)
        head_pos = self.bus_right_head.sync_read("Present_Position", self.head_motors)
        
        left_arm_state = {f"{k}.pos": v for k, v in left_arm_pos.items()}
        right_arm_state = {f"{k}.pos": v for k, v in right_arm_pos.items()}
        head_state = {f"{k}.pos": v for k, v in head_pos.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        camera_obs = self.get_camera_observation()

        # Combine all observations
        obs_dict = {**left_arm_state, **right_arm_state, **head_state, **camera_obs}

        return obs_dict
    
    def get_camera_observation(self):
        obs_dict = {}
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        
        return obs_dict 


    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command lekiwi to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            np.ndarray: the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        left_arm_pos = {k: v for k, v in action.items() if k.startswith("left_arm_") and k.endswith(".pos")}
        right_arm_pos = {k: v for k, v in action.items() if k.startswith("right_arm_") and k.endswith(".pos")}
        head_pos = {k: v for k, v in action.items() if k.startswith("head_") and k.endswith(".pos")}
        
        if self.config.max_relative_target is not None:
            # Read present positions for left arm, right arm, and head
            present_pos_left = self.bus_left_base.sync_read("Present_Position", self.left_arm_motors)
            present_pos_right = self.bus_right_head.sync_read("Present_Position", self.right_arm_motors)
            present_pos_head = self.bus_right_head.sync_read("Present_Position", self.head_motors)

            # Combine all present positions
            present_pos = {**present_pos_left, **present_pos_right, **present_pos_head}

            # Ensure safe goal position for each arm and head
            goal_present_pos = {
                key: (g_pos, present_pos[key]) for key, g_pos in chain(left_arm_pos.items(), right_arm_pos.items(), head_pos.items())
            }
            safe_goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

            # Update the action with the safe goal positions
            left_arm_pos = {k: v for k, v in safe_goal_pos.items() if k in left_arm_pos}
            right_arm_pos = {k: v for k, v in safe_goal_pos.items() if k in right_arm_pos}
            head_pos = {k: v for k, v in safe_goal_pos.items() if k in head_pos}
        
        left_arm_pos_raw = {k.replace(".pos", ""): v for k, v in left_arm_pos.items()}
        right_arm_pos_raw = {k.replace(".pos", ""): v for k, v in right_arm_pos.items()}
        head_pos_raw = {k.replace(".pos", ""): v for k, v in head_pos.items()}
        
        # Only sync_write if there are motors to write to
        if left_arm_pos_raw:
            self.bus_left_base.sync_write("Goal_Position", left_arm_pos_raw)
        if right_arm_pos_raw:
            self.bus_right_head.sync_write("Goal_Position", right_arm_pos_raw)
        if head_pos_raw:
            self.bus_right_head.sync_write("Goal_Position", head_pos_raw)
        return {
            **left_arm_pos,
            **right_arm_pos,
            **head_pos,
        }

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus_left_base.disconnect(self.config.disable_torque_on_disconnect)
        self.bus_right_head.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
