#!/usr/bin/env bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

# gen local paths for robot urdf
SRC="$SCRIPT_DIR/robot.urdf"
DEST="$SCRIPT_DIR/robot_gen.urdf"
sed "s;package://;$SCRIPT_DIR/;" "$SRC" >"$DEST"
echo "Wrote $DEST"

# gen local paths for gripper urdf
SRC="$SCRIPT_DIR/gripper.urdf"
DEST="$SCRIPT_DIR/gripper_gen.urdf"
sed "s;package://;$SCRIPT_DIR/;" "$SRC" >"$DEST"
echo "Wrote $DEST"

# get local paths for xlerobot urdf
SRC="$SCRIPT_DIR/xlerobot.urdf"
DEST="$SCRIPT_DIR/xlerobot_gen.urdf"
sed "s;package://;$SCRIPT_DIR/;" "$SRC" >"$DEST"
echo "Wrote $DEST"

# patch robot urdf for local inverse kinematics
SRC="$SCRIPT_DIR/robot_gen.urdf"
DEST="$SCRIPT_DIR/robot_gen_ikfast.urdf"
sed "s;-0.00498613 0.0198678;0.0 0.0198678;" "$SRC" >"$DEST"
echo "Wrote $DEST"
