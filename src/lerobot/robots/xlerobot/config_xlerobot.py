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

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, Cv2Rotation, ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

from ..config import RobotConfig


def xlerobot_cameras_config() -> dict[str, CameraConfig]:
    return {
        "left_wrist": OpenCVCameraConfig(
            index_or_path='/dev/v4l/by-path/platform-a80aa10000.usb-usb-0:2.3:1.0-video-index0', 
            fps=30,
            width=640,
            height=480,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
            fourcc="MJPG"
        ),

        "right_wrist": OpenCVCameraConfig(
            index_or_path='/dev/v4l/by-path/platform-a80aa10000.usb-usb-0:2.4:1.0-video-index0',
            fps=30,
            width=640,
            height=480,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
            fourcc="MJPG"
        ),
        
        "head": RealSenseCameraConfig(
            serial_number_or_name="213622300072",  # Replace with camera SN
            fps=30,
            width=640,
            height=480,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
            use_depth=False
        ),

        # "rear": RealSenseCameraConfig(
        #     serial_number_or_name="308222301716",  # Replace with camera SN
        #     fps=30,
        #     width=640,
        #     height=480,
        #     color_mode=ColorMode.RGB,
        #     rotation=Cv2Rotation.NO_ROTATION,
        #     use_depth=False
        # ),
 
    }


@RobotConfig.register_subclass("xlerobot")
@dataclass
class XLerobotConfig(RobotConfig):
    
    port_right_head: str = "/dev/xlerobot_right_head"  # port to connect to the bus (so101 + head camera)
    port_left_base: str = "/dev/xlerobot_left_base"  # port to connect to the bus (same as lekiwi setup)
    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False

    base_teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "8",
            "backward": "2",
            "left": "4",
            "right": "6",
            "rotate_left": "7",
            "rotate_right": "9",
            # Speed control
            "speed_up": "1",
            "speed_down": "3",
            # quit teleop
            "quit": "0",
        }
    )



@dataclass
class XLerobotHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 3600

    # Watchdog: stop the robot if no command is received for over 0.5 seconds.
    watchdog_timeout_ms: int = 500

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30

@RobotConfig.register_subclass("xlerobot_client")
@dataclass
class XLerobotClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    base_teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "8",
            "backward": "2",
            "left": "4",
            "right": "6",
            "rotate_left": "7",
            "rotate_right": "9",
            # Speed control
            "speed_up": "1",
            "speed_down": "3",
            # quit teleop
            "quit": "0",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
