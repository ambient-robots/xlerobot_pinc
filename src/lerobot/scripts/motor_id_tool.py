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

"""
Tool to scan for Feetech motors and set individual motor IDs.

Examples:

Scan for all motors on a port:
    lerobot-motor-id-tool scan --port /dev/tty.usbmodem123

Scan at a specific baudrate:
    lerobot-motor-id-tool scan --port /dev/tty.usbmodem123 --baudrate 1000000

Set a motor's ID (auto-detect current settings):
    lerobot-motor-id-tool set-id --port /dev/tty.usbmodem123 --model sts3215 --id 5

Set a motor's ID with known current settings (faster):
    lerobot-motor-id-tool set-id --port /dev/tty.usbmodem123 --model sts3215 --id 5 \\
        --current-baudrate 1000000 --current-id 1
"""

import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors.feetech.tables import MODEL_NUMBER_TABLE, SCAN_BAUDRATES, STS_SMS_SERIES_BAUDRATE_TABLE

SUPPORTED_MODELS = list(MODEL_NUMBER_TABLE.keys())
SUPPORTED_BAUDRATES = list(STS_SMS_SERIES_BAUDRATE_TABLE.keys())


def scan_motors(port: str, baudrate: int | None = None) -> None:
    """Scan a port for connected motors and print their IDs."""
    print(f"Scanning port: {port}")

    if baudrate is not None:
        # Scan at specific baudrate
        print(f"Scanning at baudrate: {baudrate}")
        bus = FeetechMotorsBus(port, {})
        bus._connect(handshake=False)
        bus.set_baudrate(baudrate)
        ids_models = bus.broadcast_ping()
        bus.port_handler.closePort()

        if ids_models:
            print(f"\nMotors found at baudrate {baudrate}:")
            for motor_id, model_num in ids_models.items():
                model_name = _get_model_name(model_num)
                print(f"  ID: {motor_id}, Model: {model_name} (num: {model_num})")
        else:
            print(f"No motors found at baudrate {baudrate}")
    else:
        # Scan all baudrates
        print(f"Scanning all baudrates: {SCAN_BAUDRATES}")
        baudrate_ids = FeetechMotorsBus.scan_port(port)

        if baudrate_ids:
            print("\nScan complete. Summary:")
            for br, ids in baudrate_ids.items():
                print(f"  Baudrate {br}: {len(ids)} motor(s) found with IDs: {ids}")
        else:
            print("No motors found on any baudrate")


def set_motor_id(
    port: str,
    model: str,
    target_id: int,
    current_baudrate: int | None = None,
    current_id: int | None = None,
    target_baudrate: int = 1_000_000,
) -> None:
    """Set a motor's ID."""
    if model not in SUPPORTED_MODELS:
        raise ValueError(f"Unsupported model '{model}'. Supported: {SUPPORTED_MODELS}")

    if not 1 <= target_id <= 252:
        raise ValueError(f"Target ID must be between 1 and 252, got {target_id}")

    print(f"Setting motor ID on port: {port}")
    print(f"  Model: {model}")
    print(f"  Target ID: {target_id}")
    print(f"  Target baudrate: {target_baudrate}")

    if current_baudrate:
        print(f"  Current baudrate: {current_baudrate}")
    if current_id:
        print(f"  Current ID: {current_id}")

    # Create a temporary motor config with the target ID
    bus = FeetechMotorsBus(
        port=port,
        motors={"motor": Motor(target_id, model, MotorNormMode.RANGE_M100_100)},
    )

    # Override default baudrate if target_baudrate is different
    if target_baudrate != bus.default_baudrate:
        bus.default_baudrate = target_baudrate

    print("\nSearching for motor...")
    bus.setup_motor("motor", initial_baudrate=current_baudrate, initial_id=current_id)

    print(f"\nSuccess! Motor ID set to {target_id} at baudrate {target_baudrate}")


def _get_model_name(model_number: int) -> str:
    """Get model name from model number."""
    for name, num in MODEL_NUMBER_TABLE.items():
        if num == model_number:
            return name
    return "unknown"


def main():
    parser = argparse.ArgumentParser(
        description="Scan for Feetech motors and set individual motor IDs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Scan subcommand
    scan_parser = subparsers.add_parser("scan", help="Scan port for connected motors")
    scan_parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/tty.usbmodem123)")
    scan_parser.add_argument(
        "--baudrate",
        type=int,
        choices=SCAN_BAUDRATES,
        help=f"Specific baudrate to scan (default: scan all). Options: {SCAN_BAUDRATES}",
    )

    # Set-ID subcommand
    setid_parser = subparsers.add_parser("set-id", help="Set a motor's ID")
    setid_parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/tty.usbmodem123)")
    setid_parser.add_argument(
        "--model",
        required=True,
        choices=SUPPORTED_MODELS,
        help=f"Motor model. Options: {SUPPORTED_MODELS}",
    )
    setid_parser.add_argument("--id", type=int, required=True, help="Target ID to set (1-252)")
    setid_parser.add_argument(
        "--current-baudrate",
        type=int,
        choices=SUPPORTED_BAUDRATES,
        help="Current motor baudrate (skips scanning if provided)",
    )
    setid_parser.add_argument(
        "--current-id", type=int, help="Current motor ID (skips scanning if provided)"
    )
    setid_parser.add_argument(
        "--target-baudrate",
        type=int,
        default=1_000_000,
        choices=SUPPORTED_BAUDRATES,
        help="Baudrate to set on motor (default: 1000000)",
    )

    args = parser.parse_args()

    if args.command == "scan":
        scan_motors(args.port, args.baudrate)
    elif args.command == "set-id":
        set_motor_id(
            port=args.port,
            model=args.model,
            target_id=args.id,
            current_baudrate=args.current_baudrate,
            current_id=args.current_id,
            target_baudrate=args.target_baudrate,
        )


if __name__ == "__main__":
    main()
