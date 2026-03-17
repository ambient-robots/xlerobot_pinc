# Hardware Sources

This document lists the upstream sources for the physical build assets referenced by `xlerobot_pinc`.

## XLeRobot

Upstream repository: [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot)

`xlerobot_pinc` is extended from `XLeRobot`. Users should obtain the relevant base hardware and printable assets from the upstream repository, especially its hardware-related areas such as:

- `hardware/`
- `stl/` when present in the upstream documentation or repository layout

## PincOpen

Upstream repository: [pollen-robotics/PincOpen](https://github.com/pollen-robotics/PincOpen)

The gripper used in this repo is based on `PincOpen`. Users should obtain the gripper CAD and print assets from the upstream project, especially its `cad/` area and related build documentation.

## xlerobot_pinc-specific changes

At a conceptual level, this repo differs from the upstream hardware stack by:

- using an SO107-style 6-DoF arm instead of the original SO101-style arm assumption
- integrating a gripper based on `PincOpen`
- supporting a final robot configuration with or without the mobile base

## Current repository status

This repository does not yet ship the actual 3D-print source files. This folder is the documentation scaffold for future hardware publication.
