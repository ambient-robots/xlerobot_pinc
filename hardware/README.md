# Hardware

This folder documents the physical hardware lineage of `xlerobot_pinc`.

## Hardware lineage

`xlerobot_pinc` is extended from [XLeRobot](https://github.com/Vector-Wangel/XLeRobot). More specifically, the hardware basis used here follows the upstream `xlerobot_0_4_0` hardware layout documented in the [`XLeRobot/hardware`](https://github.com/Vector-Wangel/XLeRobot/tree/main/hardware) folder, especially around the arm-base structure and upper-body mounting geometry.

Relative to that upstream platform, this repo is centered on:

- an arm setup upgraded to an SO107-style 6-DoF arm
- a modified elbow section that splits the original SO101 under-arm link into separate elbow-flex and elbow-roll links, introducing the elbow-roll DoF
- a gripper based on [PincOpen](https://github.com/pollen-robotics/PincOpen)
- upper-body-focused customizations for the dual-arm configuration, while leaving the dual-wheel mobile base introduced in upstream 0.4.0 notes out of the current scope

The mobile base can still be added later, since the current custom changes are mainly focused on the upper body.

## What is in this repo today

This repository includes a small set of `xlerobot_pinc`-specific custom parts under [`custom_parts/`](custom_parts/), alongside documentation of the upstream hardware provenance.

Current local custom parts:

- `SO107_Elbow_Flex_Link` for the elbow-flex link in the split elbow module
- `SO107_Elbow_Roll_Link` for the elbow-roll link in the split elbow module
- `PincOpen_Pointy_Tip_TPU95` for the elastic PincOpen tip
- `PincOpen_Fisheye_Camera_Holder_38x38` for the 38x38 fisheye camera mount

See [SOURCES.md](SOURCES.md) for Onshape references, upstream hardware paths, and the detailed basis of each local custom part.

## Future extension

This folder can later be extended with local custom hardware files or assembly notes.
