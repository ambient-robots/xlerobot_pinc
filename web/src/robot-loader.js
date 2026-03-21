import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import * as THREE from 'three';

function createLoader() {
  const loader = new URDFLoader();
  loader.packages = '';

  loader.loadMeshCb = (path, manager, onComplete) => {
    const stlLoader = new STLLoader(manager);
    stlLoader.load(
      path,
      (geometry) => {
        geometry.computeVertexNormals();
        const material = new THREE.MeshStandardMaterial({
          color: 0xcccccc,
          roughness: 0.6,
          metalness: 0.1,
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        onComplete(mesh);
      },
      undefined,
      (err) => {
        console.error('Failed to load mesh:', path, err);
        onComplete(null, err);
      }
    );
  };

  return loader;
}

function loadURDF(url) {
  return new Promise((resolve, reject) => {
    const loader = createLoader();
    loader.load(url, resolve, undefined, reject);
  });
}

function upgradeMaterials(robot) {
  robot.traverse((child) => {
    if (child.isMesh && child.material) {
      const color = child.material.color
        ? child.material.color.clone()
        : new THREE.Color(0xcccccc);

      child.material = new THREE.MeshPhysicalMaterial({
        color,
        roughness: 0.45,
        metalness: 0.08,
        clearcoat: 0.15,
        clearcoatRoughness: 0.4,
      });
      child.castShadow = true;
      child.receiveShadow = true;
    }
  });
}

/**
 * Loads the full robot assembly:
 * - xlerobot.urdf (base platform)
 * - robot.urdf x2 (left + right arms)
 * - gripper.urdf x2 (left + right grippers)
 */
export async function loadRobotAssembly(scene) {
  const [base, leftArm, rightArm, leftGripper, rightGripper] = await Promise.all([
    loadURDF('./xlerobot.urdf'),
    loadURDF('./robot.urdf'),
    loadURDF('./robot.urdf'),
    loadURDF('./gripper.urdf'),
    loadURDF('./gripper.urdf'),
  ]);

  // Attach arms to base mount points
  const leftMount = base.links['so107_left_base'];
  const rightMount = base.links['so107_right_base'];

  if (leftMount) leftMount.add(leftArm);
  else console.warn('so107_left_base not found in base URDF');

  if (rightMount) rightMount.add(rightArm);
  else console.warn('so107_right_base not found in base URDF');

  // Attach grippers to arm endpoints
  const leftGripperMount = leftArm.links['interface_arm100'];
  const rightGripperMount = rightArm.links['interface_arm100'];

  if (leftGripperMount) leftGripperMount.add(leftGripper);
  else console.warn('interface_arm100 not found in left arm URDF');

  if (rightGripperMount) rightGripperMount.add(rightGripper);
  else console.warn('interface_arm100 not found in right arm URDF');

  // Upgrade all materials for premium rendering
  upgradeMaterials(base);
  upgradeMaterials(leftArm);
  upgradeMaterials(rightArm);
  upgradeMaterials(leftGripper);
  upgradeMaterials(rightGripper);

  // Rotate from Z-up (URDF) to Y-up (Three.js)
  base.rotation.x = -Math.PI / 2;

  scene.add(base);

  return { base, leftArm, rightArm, leftGripper, rightGripper };
}
