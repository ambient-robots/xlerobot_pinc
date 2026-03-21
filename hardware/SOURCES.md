# Hardware Sources

This document lists the upstream and local source locations for the hardware assets referenced by `xlerobot_pinc`.

## Onshape Source Documents

Primary CAD references for this repository:

| Reference | Scope | Link |
|---|---|---|
| SO107 standalone arm | Arm-only CAD reference | <https://cad.onshape.com/documents/497bc5aee616bc3122c03e8e/w/01522a834421a6d358114bf2/e/2197a655705492fa3028d295> |
| `xlerobot_pinc` assembly | Full integrated robot assembly | <https://cad.onshape.com/documents/96925cf817c1a878a85e838a/w/65e84f84222e56821484522f/e/8df6a6c93d54b1b95e807d87> |

## XLeRobot

Upstream repository: [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot)

Relevant upstream hardware paths:

| Upstream Area | Why It Matters For `xlerobot_pinc` |
|---|---|
| [`hardware/readme.md`](https://github.com/Vector-Wangel/XLeRobot/blob/main/hardware/readme.md) | Upstream hardware notes, including the 0.4.0 arm-base and dual-wheel update summary. |
| [`hardware/step/`](https://github.com/Vector-Wangel/XLeRobot/tree/main/hardware/step) | STEP exports for upstream CAD. |
| [`hardware/ongoing_upgrades/`](https://github.com/Vector-Wangel/XLeRobot/tree/main/hardware/ongoing_upgrades) | Newer upstream hardware revisions. |
| [`hardware/XLeRobot_0_4_0_extra.stl`](https://github.com/Vector-Wangel/XLeRobot/blob/main/hardware/XLeRobot_0_4_0_extra.stl) | Extra STL for the upstream 0.4.0 generation. |

For `xlerobot_pinc`, the main upstream inheritance is the 0.4.0 arm-base and upper-body hardware layout, not the dual-wheel base.

## PincOpen

Upstream repository: [pollen-robotics/PincOpen](https://github.com/pollen-robotics/PincOpen)

Relevant upstream source: the `cad/` area and build documentation in the PincOpen repository.

## Local Custom Parts In This Repo

Local custom exports shipped in this repository under [`custom_parts/`](custom_parts/):

For the arm, the original SO101 under-arm section is split into separate elbow-flex and elbow-roll links to introduce the elbow-roll DoF in the modified SO107-style arm.

| Part Files | Notes |
|---|---|
| `SO107_Elbow_Flex_Link.step` / `SO107_Elbow_Flex_Link.stl` | Elbow-flex link in the split elbow module; effectively a copy of the SO101 `Wrist_Roll_Pitch` part. |
| `SO107_Elbow_Roll_Link.step` / `SO107_Elbow_Roll_Link.stl` | Elbow-roll link in the split elbow module; modified from the SO101 `Under Arm` part. |
| `PincOpen_Pointy_Tip_TPU95.step` / `PincOpen_Pointy_Tip_TPU95.stl` | Elastic pointy tip for the gripper. Recommended print material: TPU95. |
| `PincOpen_Fisheye_Camera_Holder_38x38.step` / `PincOpen_Fisheye_Camera_Holder_38x38.stl` | Camera holder adapted for the 38x38 fisheye camera. |
