# Implementation Plan - Parameter Adjustments for Mission Hover & Takeoff

This document outlines the proposed adjustments to the VTOL control configuration and mission scripts for mission hover duration (10s), takeoff slowdown zone (0.2m), and target takeoff altitude (1.2m).

## Proposed Changes

### Configuration & Control (`vtol_control`)

#### [MODIFY] [vtol_config.yaml](../workspace/src/vtol_control/config/vtol_config.yaml)
- Change `takeoff.altitude` from `1` to `1.2`.
- Add `takeoff.slowdown_zone: 0.2`.

#### [MODIFY] [config_reader.py](../workspace/src/vtol_control/vtol_control/config_reader.py)
- Update `get_takeoff_config()` to load `slowdown_zone` parameter from `vtol_config.yaml` (default: 0.2).

#### [MODIFY] [vtol_base.py](../workspace/src/vtol_control/vtol_control/vtol_base.py)
- Update `takeoff()` method to use `SLOWDOWN_ZONE` from `config.get('slowdown_zone', 0.2)` instead of hardcoded `0.1`.

#### [MODIFY] [mission_hover.py](../workspace/src/vtol_control/vtol_control/mission_hover.py)
- Change hover duration from `5.0` seconds to `10.0` seconds in `run_mission()`.

#### [MODIFY] [menu_launcher.py](../workspace/src/vtol_control/vtol_control/menu_launcher.py)
- Update menu text from "Misi Hover 5 Detik" to "Misi Hover 10 Detik".

---

## Verification Plan

### Automated Build Verification
- Run `colcon build --packages-select vtol_control` to verify python package syntax and installation integrity.

### Execution Verification
- Test `mission_hover` or inspect parameter loading via ROS2 python node execution.
