# VisionArm — Camera-Guided Autonomous Pick-and-Place Robotic Arm

**Team:** Kinematix
**Event:** RMK Innovate Hackathon 2026
**Track:** Edge AI & IoT (anchor) · Robotics & Automation (exploratory)

A medium-reach (≈45cm) desktop robotic arm built for precise pick-and-place tasks
and plant watering, with two control modes:

- **Manual mode (primary/dependable):** ESP32 pairs directly with an Xbox
  controller over Bluetooth via Bluepad32 — no laptop required.
- **Camera-guided mode (secondary/autonomous demo):** phone camera + OpenCV on
  a laptop/Raspberry Pi 5, inverse kinematics via `ikpy`, servo angles sent to
  the ESP32 over WebSocket.

## Team

| Name | Year | Branch | Role |
|---|---|---|---|
| Hrishi | II | EEV | Lead applicant |
| Shivani S | III | EEV | Member |
| Vasanthapriyan P | IV | EEV | Anchor |
| Akash S | II | VLSI | Member |
| Keerthana S | III | EEV | Member |
| Magesh Kumar V | III | EEV | Member |

## Repo contents

- [`docs/wiring.md`](docs/wiring.md) — ESP32 GPIO/servo pin map and control scheme
- [`docs/firmware.md`](docs/firmware.md) — Firmware version history and behavior notes
- [`CREDITS.md`](CREDITS.md) — Design attribution

Code (firmware, `arm_control.py`, Gazebo world files) will be added in a
follow-up commit.

## Hardware summary

- 4x MG996R + 3x MG90S servos (seven total, mirrored shoulder pair), driven
  directly from ESP32 GPIO
- 5V 10A supply on soldered perfboard rails with bulk capacitors
- ESP32 powered separately (mobile charger / laptop), common ground
- Custom 3D-printed chassis based on the Emre Kalem reference design (see
  [CREDITS.md](CREDITS.md))
- Raspberry Pi 5 (`arm1`) taking over vision processing / control-hub duties

## License

TBD — add a license before making this public if you want to formally set
usage terms.
