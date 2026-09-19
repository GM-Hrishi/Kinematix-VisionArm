# Roadmap & Known Issues

## Known issues

**Servos**
- Every MG996R-class servo initially received turned out to be
  continuous-rotation (360°) units despite being sold as 180° positional —
  confirmed by opening them up. Fixed by moving the metal gears from the
  CR/360 servos into fake 180° plastic servos that still had working
  potentiometers (verified by position-hold test and multimeter).
- One of the four original MG996R units was dead — confirmed unresponsive
  after swapping wires and channels.
- At full extension with a 250g payload, the shoulder needs ~1.4 N·m against
  a 1.90 N·m stall torque (~75% of max) — well above the ~30% load where
  servos hold position reliably. Expect sag and buzzing near maximum reach.

**Firmware**
- Xbox controller disconnects frequently over Bluetooth (Bluepad32) — root
  cause is Bluetooth dropouts, not ESP32 resets.

**Mechanical**
- The Emre Kalem design is 6-axis but only 5 DOF for positioning (base,
  shoulder, elbow, wrist pitch, wrist roll) plus the gripper — with 5
  positioning DOF the arm can't freely choose approach orientation, which
  constrains the inverse-kinematics work for camera-guided mode.
- STL meshes are exported in print orientation with no joint frames, so they
  can't be auto-assembled into a simulation model — real-mesh assembly has
  to be done by hand.

## Roadmap

- [x] Fix limp → re-engage to reset to rest position — **done in V11**
      (`exitLimpMode()` snaps all joint targets to REST before motion
      resumes, so the arm doesn't jump from wherever it sagged to while limp)
- [ ] Build the gripper (GPIO32) — wired but mechanically unfinished; code
      is present and gated behind `#define GRIPPER_ENABLED 0`
- [ ] Complete Raspberry Pi 5 migration: Pi as main control hub, ESP32 kept
      as servo driver over USB
- [ ] Camera-guided mode: phone camera + OpenCV + `ikpy` inverse kinematics,
      servo angles over WebSocket to ESP32 — kept out of Round 1, targeted
      for the finale if ready in time
- [ ] Structural improvements to the arm, then explore adding stepper motors
      for the heavier joints
- [ ] Standalone browser-based 3D viewer (drag/slider joint control on the
      actual STL parts) as a separate hackathon demo deliverable

## Event status

- Registration for RMK Innovate 2026 submitted (deadline 8 Sept 2026)
- Round 1 (PPT + 3-minute explainer video) deadline extended to 15 Sept 2026;
  shortlist announced 12 Sept 2026
- Final round delayed from the original 18–19 Sept 2026 date — new date not
  yet announced
- Round 1 PPT scope: track shown as Robotics & Automation only; emphasizes
  all 3D-printed parts made at the college AICTE IDEA Lab; no faculty mentor
  on the team
