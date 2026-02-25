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

import os
from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, Cv2Rotation, ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig

from ..config import RobotConfig


def _env_bool(name: str) -> bool | None:
    value = os.getenv(name)
    if value is None:
        return None

    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False

    raise ValueError(
        f"Invalid boolean value for env var '{name}': {value!r}. "
        "Use one of: 1/0, true/false, yes/no, on/off."
    )


def _required_env_nonempty_str(name: str) -> str:
    value = os.getenv(name)
    if value is None or not value.strip():
        raise ValueError(
            f"Missing or empty env var '{name}'. "
            "Set it via load_xlerobot_env.sh or export it in your shell."
        )
    stripped = value.strip()
    return stripped


def _env_positive_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default

    try:
        parsed = int(value.strip())
    except ValueError as exc:
        raise ValueError(f"Invalid integer value for env var '{name}': {value!r}.") from exc

    if parsed <= 0:
        raise ValueError(f"Invalid value for env var '{name}': {value!r}. Must be > 0.")

    return parsed


def _env_fourcc(name: str, default: str) -> str:
    value = os.getenv(name)
    if value is None:
        return default

    parsed = value.strip()
    if len(parsed) != 4:
        raise ValueError(f"Invalid FOURCC for env var '{name}': {value!r}. Must be exactly 4 characters.")
    return parsed


def xlerobot_cameras_config() -> dict[str, CameraConfig]:
    left_wrist_index_or_path = _required_env_nonempty_str("XLEROBOT_LEFT_WRIST_INDEX_OR_PATH")
    right_wrist_index_or_path = _required_env_nonempty_str("XLEROBOT_RIGHT_WRIST_INDEX_OR_PATH")
    head_serial_number_or_name = _required_env_nonempty_str("XLEROBOT_HEAD_SERIAL_NUMBER_OR_NAME")
    left_wrist_fps = _env_positive_int("XLEROBOT_LEFT_WRIST_FPS", 60)
    left_wrist_width = _env_positive_int("XLEROBOT_LEFT_WRIST_WIDTH", 640)
    left_wrist_height = _env_positive_int("XLEROBOT_LEFT_WRIST_HEIGHT", 480)
    left_wrist_fourcc = _env_fourcc("XLEROBOT_LEFT_WRIST_FOURCC", "YUYV")

    right_wrist_fps = _env_positive_int("XLEROBOT_RIGHT_WRIST_FPS", 60)
    right_wrist_width = _env_positive_int("XLEROBOT_RIGHT_WRIST_WIDTH", 640)
    right_wrist_height = _env_positive_int("XLEROBOT_RIGHT_WRIST_HEIGHT", 480)
    right_wrist_fourcc = _env_fourcc("XLEROBOT_RIGHT_WRIST_FOURCC", "YUYV")

    head_fps = _env_positive_int("XLEROBOT_HEAD_FPS", 60)
    head_width = _env_positive_int("XLEROBOT_HEAD_WIDTH", 640)
    head_height = _env_positive_int("XLEROBOT_HEAD_HEIGHT", 480)

    return {
        "left_wrist": OpenCVCameraConfig(
            index_or_path=left_wrist_index_or_path,
            fps=left_wrist_fps,
            width=left_wrist_width,
            height=left_wrist_height,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
            warmup_s=3,
            fourcc=left_wrist_fourcc
        ),

        "right_wrist": OpenCVCameraConfig(
            index_or_path=right_wrist_index_or_path,
            fps=right_wrist_fps,
            width=right_wrist_width,
            height=right_wrist_height,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
            warmup_s=3,
            fourcc=right_wrist_fourcc
        ),
        
        "head": RealSenseCameraConfig(
            serial_number_or_name=head_serial_number_or_name,
            fps=head_fps,
            width=head_width,
            height=head_height,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
            warmup_s=3,
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


@RobotConfig.register_subclass("xlerobot_pro")
@dataclass
class XLerobotProConfig(RobotConfig):
    # Whether this setup includes the 3-DoF omni mobile platform.
    has_mobile_platform: bool | None = None

    port_right_head: str = "/dev/xlerobot_right_head"  # port to connect to the bus (so107 + pan-tilt)
    port_left_base: str = "/dev/xlerobot_left_base"  # port to connect to the bus (so107 + lekiwi base)
    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = True

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

    def __post_init__(self):
        super().__post_init__()
        if self.has_mobile_platform is None:
            self.has_mobile_platform = _env_bool("XLEROBOT_HAS_MOBILE_PLATFORM")

        if self.has_mobile_platform is None:
            raise ValueError(
                "Missing 'has_mobile_platform'. Set it explicitly in XLerobotProConfig(...) "
                "or export XLEROBOT_HAS_MOBILE_PLATFORM."
            )
