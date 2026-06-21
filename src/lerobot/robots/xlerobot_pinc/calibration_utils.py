from collections.abc import Iterable, Mapping
from typing import TypeVar


T = TypeVar("T")


def filter_calibration_for_motors(
    calibration: Mapping[str, T] | None,
    motor_names: Iterable[str],
) -> dict[str, T]:
    """Return calibration entries that belong to the given motors."""
    if not calibration:
        return {}

    motor_name_set = set(motor_names)
    return {name: value for name, value in calibration.items() if name in motor_name_set}


def merge_calibrations(
    existing: Mapping[str, T] | None,
    updated: Mapping[str, T],
    motor_names: Iterable[str] | None = None,
) -> dict[str, T]:
    """Merge updated calibration entries without deleting untouched entries."""
    merged = dict(existing or {})
    allowed_names = set(updated) if motor_names is None else set(motor_names)
    for name in allowed_names:
        if name in updated:
            merged[name] = updated[name]
    return merged
