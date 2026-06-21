import hashlib
import importlib.util
import json
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DUMMY_CALIBRATION_PATH = Path(
    "/home/that/.cache/huggingface/lerobot/calibration/robots/xlerobot_pinc/"
    "ambient_xlerobot_pinc_dummy_test.json"
)

LEFT_BASE_MOTORS = {
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_elbow_roll",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "left_arm_gripper",
    "base_left_wheel",
    "base_back_wheel",
    "base_right_wheel",
}

RIGHT_HEAD_MOTORS = {
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_elbow_roll",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
    "right_arm_gripper",
    "head_pan",
    "head_tilt",
}


def load_calibration_utils():
    module_path = REPO_ROOT / "src/lerobot/robots/xlerobot_pinc/calibration_utils.py"
    spec = importlib.util.spec_from_file_location("xlerobot_pinc_calibration_utils", module_path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def sentinel_calibration(motor_names: set[str], offset: int) -> dict[str, dict[str, int]]:
    return {
        motor: {
            "id": index,
            "drive_mode": 0,
            "homing_offset": offset + index,
            "range_min": offset + index + 100,
            "range_max": offset + index + 200,
        }
        for index, motor in enumerate(sorted(motor_names), start=1)
    }


class TestXLerobotPincCalibrationMerge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not DUMMY_CALIBRATION_PATH.is_file():
            raise AssertionError(f"Missing dummy calibration fixture: {DUMMY_CALIBRATION_PATH}")
        cls.calibration_utils = load_calibration_utils()

    def setUp(self):
        self.before_bytes = DUMMY_CALIBRATION_PATH.read_bytes()
        self.before_hash = hashlib.sha256(self.before_bytes).hexdigest()
        self.fixture = json.loads(self.before_bytes)

    def tearDown(self):
        after_bytes = DUMMY_CALIBRATION_PATH.read_bytes()
        after_hash = hashlib.sha256(after_bytes).hexdigest()
        self.assertEqual(after_hash, self.before_hash)
        self.assertEqual(after_bytes, self.before_bytes)

    def test_right_head_merge_preserves_existing_left_base_calibration(self):
        updated_right_head = sentinel_calibration(RIGHT_HEAD_MOTORS, offset=10_000)

        merged = self.calibration_utils.merge_calibrations(
            self.fixture,
            updated_right_head,
            RIGHT_HEAD_MOTORS,
        )

        for motor in RIGHT_HEAD_MOTORS:
            self.assertEqual(merged[motor], updated_right_head[motor])
        for motor in LEFT_BASE_MOTORS:
            self.assertEqual(merged[motor], self.fixture[motor])

    def test_left_base_merge_preserves_existing_right_head_calibration(self):
        updated_left_base = sentinel_calibration(LEFT_BASE_MOTORS, offset=20_000)

        merged = self.calibration_utils.merge_calibrations(
            self.fixture,
            updated_left_base,
            LEFT_BASE_MOTORS,
        )

        for motor in LEFT_BASE_MOTORS:
            self.assertEqual(merged[motor], updated_left_base[motor])
        for motor in RIGHT_HEAD_MOTORS:
            self.assertEqual(merged[motor], self.fixture[motor])


if __name__ == "__main__":
    unittest.main()
