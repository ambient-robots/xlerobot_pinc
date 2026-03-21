// Helper to create dual-arm preset from per-arm values
function dual(leftValues, rightValues) {
  const result = {};
  for (const [k, v] of Object.entries(leftValues)) {
    result[`left_${k}`] = v;
  }
  for (const [k, v] of Object.entries(rightValues || leftValues)) {
    result[`right_${k}`] = v;
  }
  return result;
}

const HOME = {
  shoulder_pan: 0, shoulder_lift: 0, elbow_flex: 0,
  elbow_roll: 0, wrist_flex: 0, wrist_roll: 0, gripper: 0,
};

const READY = {
  shoulder_pan: 0, shoulder_lift: -0.3, elbow_flex: -0.5,
  elbow_roll: 0, wrist_flex: -0.3, wrist_roll: 0, gripper: 0,
};

const PICK = {
  shoulder_pan: 0, shoulder_lift: -0.7, elbow_flex: -0.9,
  elbow_roll: 0, wrist_flex: -0.5, wrist_roll: 0, gripper: 1.0,
};

const MIRROR_LEFT = {
  shoulder_pan: 0.5, shoulder_lift: -0.4, elbow_flex: -0.6,
  elbow_roll: 0, wrist_flex: -0.3, wrist_roll: 0, gripper: 0,
};

const MIRROR_RIGHT = {
  shoulder_pan: -0.5, shoulder_lift: -0.4, elbow_flex: -0.6,
  elbow_roll: 0, wrist_flex: -0.3, wrist_roll: 0, gripper: 0,
};

const WAVE_LEFT = {
  shoulder_pan: 0, shoulder_lift: -0.8, elbow_flex: -0.9,
  elbow_roll: 1.0, wrist_flex: 0.3, wrist_roll: 0.8, gripper: 0,
};

const WAVE_RIGHT = {
  shoulder_pan: 0, shoulder_lift: -0.3, elbow_flex: -0.5,
  elbow_roll: 0, wrist_flex: -0.3, wrist_roll: 0, gripper: 0,
};

export const HOME_VALUES = dual(HOME);

export const ANIMATION_SEQUENCE = [
  { values: dual(READY), duration: 800 },
  { values: dual(MIRROR_LEFT, MIRROR_RIGHT), duration: 800 },
  { values: dual(PICK), duration: 800 },
  { values: dual(WAVE_LEFT, WAVE_RIGHT), duration: 800 },
  { values: dual(HOME), duration: 800 },
];
