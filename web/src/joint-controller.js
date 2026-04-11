// Per-arm joint definitions (same for left and right)
export const ARM_JOINTS = [
  { name: 'shoulder_pan',  label: 'Shoulder Pan',  min: -2.094, max: 2.094 },
  { name: 'shoulder_lift', label: 'Shoulder Lift', min: -1.658, max: 1.658 },
  { name: 'elbow_flex',    label: 'Elbow Flex',    min: -1.658, max: 1.571 },
  { name: 'elbow_roll',    label: 'Elbow Roll',    min: -3.142, max: 3.142 },
  { name: 'wrist_flex',    label: 'Wrist Flex',    min: -1.658, max: 1.658 },
  { name: 'wrist_roll',    label: 'Wrist Roll',    min: -3.142, max: 3.142 },
];

export const GRIPPER_JOINT = { name: 'gripper', label: 'Gripper', min: -3.142, max: 3.142 };

// All joints with side prefix for the full assembly
export const ALL_JOINTS = [
  ...ARM_JOINTS.map((j) => ({ ...j, name: `left_${j.name}`, label: `L ${j.label}`, side: 'left' })),
  { ...GRIPPER_JOINT, name: 'left_gripper', label: 'L Gripper', side: 'left' },
  ...ARM_JOINTS.map((j) => ({ ...j, name: `right_${j.name}`, label: `R ${j.label}`, side: 'right' })),
  { ...GRIPPER_JOINT, name: 'right_gripper', label: 'R Gripper', side: 'right' },
];

export class JointController {
  /**
   * @param {object} assembly - { leftArm, rightArm, leftGripper, rightGripper }
   */
  constructor(assembly) {
    this.robots = {
      left_arm: assembly.leftArm,
      right_arm: assembly.rightArm,
      left_gripper: assembly.leftGripper,
      right_gripper: assembly.rightGripper,
    };
    this.listeners = new Set();
  }

  // Map prefixed joint name to the right robot instance and raw joint name
  _resolve(prefixedName) {
    const [side, ...rest] = prefixedName.split('_');
    const rawName = rest.join('_');

    if (rawName === 'gripper') {
      return { robot: this.robots[`${side}_gripper`], jointName: 'gripper' };
    }
    return { robot: this.robots[`${side}_arm`], jointName: rawName };
  }

  setJoint(prefixedName, valueRad) {
    const { robot, jointName } = this._resolve(prefixedName);
    if (robot?.joints?.[jointName]) {
      robot.setJointValue(jointName, valueRad);
      this.notify(prefixedName, valueRad);
    }
  }

  getJoint(prefixedName) {
    const { robot, jointName } = this._resolve(prefixedName);
    return robot?.joints?.[jointName]?.angle ?? 0;
  }

  onChange(fn) {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }

  notify(name, value) {
    for (const fn of this.listeners) {
      fn(name, value);
    }
  }

  animateTo(targetValues, durationMs = 600) {
    const startValues = {};
    for (const name of Object.keys(targetValues)) {
      startValues[name] = this.getJoint(name);
    }
    const startTime = performance.now();

    return new Promise((resolve) => {
      const step = (now) => {
        const t = Math.min((now - startTime) / durationMs, 1);
        const ease = t < 0.5
          ? 4 * t * t * t
          : 1 - Math.pow(-2 * t + 2, 3) / 2;

        for (const [name, target] of Object.entries(targetValues)) {
          const current = startValues[name] + (target - startValues[name]) * ease;
          this.setJoint(name, current);
        }

        if (t < 1) requestAnimationFrame(step);
        else resolve();
      };
      requestAnimationFrame(step);
    });
  }
}
