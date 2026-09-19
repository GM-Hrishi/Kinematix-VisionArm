# Firmware Notes

## Current version: Positional V11

Supersedes the earlier CR-based V9 (built when arm joints were still running
on continuous-rotation servos).

**Working:**
- Bluepad32 control of shoulder pair, upper arm, tilt, and spin
- Base rotation integrated (hold X = base to 90°)

**Not yet built:**
- Gripper (GPIO32)

**Known issues:**
- Re-engaging from "limp" mode currently resumes from the previous pose,
  rather than resetting to rest — needs a fix so limp → re-engage always
  starts from rest
- Xbox controller disconnects frequently; root cause is Bluetooth dropouts,
  not ESP32 resets

**Logging:**
- Serial output should be kept to diagnostics/events only — no per-motion
  prints — so logs are lean and useful when sharing for debugging

## Control architecture

- **Manual mode:** ESP32 ↔ Xbox controller directly over Bluetooth
  (Bluepad32). No laptop required. This is the primary, dependable mode.
- **Camera-guided mode:** phone camera → OpenCV (laptop/Pi) → inverse
  kinematics (`ikpy`) → servo angles sent to ESP32 over WebSocket. Secondary,
  autonomous demo mode — kept out of the Round 1 submission, to be
  demonstrated at the finale if ready.

## Migration in progress: Raspberry Pi 5 as control hub

- Raspberry Pi 5 (hostname/username `arm1`) taking over as the main control
  hub; ESP32 kept as the servo driver, connected and powered over USB from
  the Pi
- Xbox controller now recognized on the Pi via USB dongle (confirmed with
  `evtest`)
- No monitor available at college — development happens over SSH/VNC, with
  an HDMI capture card as backup

## Simulation

- Gazebo simulation runs with a visible GUI at `~/ros2_ws/arm_primitives.sdf`
  — 6 links / 5 revolute joints plus a prismatic two-finger gripper, built
  from box/cylinder primitives (not the STL meshes)
- Driven by `arm_control.py` (home / open / close / demo / pick commands over
  the `cmd_pos` gz topic)
- Guide: `VisionArm_Gazebo_Guide.md` (kept alongside the simulation files)
- Note: the STL meshes can't be auto-assembled into a correct model — they're
  exported in print orientation with no joint frames, so real-mesh assembly
  has to be done by hand (e.g. via URDF Studio / urdf.enkeebot.com)
