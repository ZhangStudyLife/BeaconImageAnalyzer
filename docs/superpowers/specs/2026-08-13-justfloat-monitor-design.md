# JustFloat Monitor Design

## Goal

Create a standalone Windows application for receiving, recording, and replaying the fixed 43-channel three-camera JustFloat telemetry stream. The application must not expose or depend on BeaconImageAnalyzer features such as CarPlan, image algorithms, annotation, OpenCV video, or auxiliary log layouts.

## Protocol

- UDP payload is exactly 43 little-endian IEEE-754 `float32` values (172 bytes).
- A standard 4-byte JustFloat/VOFA tail (`00 00 80 7F`) may follow the values (176 bytes total).
- Text datagrams and every other payload length are rejected.
- Channels map exactly as requested:
  - `I0`: aircraft millisecond timestamp.
  - `I1-I18`: front, center, and back camera beacon 0/1 `x`, `y`, `area`.
  - `I19-I33`: front, center, and back camera lamp `cx`, `cy`, `angle`, `width`, `length`.
  - `I34-I36`: aircraft pitch, roll, yaw.
  - `I37`: repeated aircraft timestamp.
  - `I38`: reserved, expected to be zero.
  - `I39`: vehicle forward velocity.
  - `I40`: vehicle yaw.
  - `I41-I42`: planned forward and strafe velocity.

## Application

The application is a separate Qt 6 Widgets/Network target under `justfloat_monitor/`. It has one window:

- Top controls: mode, CSV import, local IPv4 address, UDP port, listen toggle, and record toggle.
- Center: three fixed camera panels named Front, Center, and Back. Each uses the existing 188 x 120 coordinate convention and draws two beacons plus one oriented car-lamp rectangle.
- Bottom playback controls: play/pause, previous/next frame, frame selector, timeline, and speed.
- Status area: timestamps, aircraft attitude, reserved channel, vehicle velocity/yaw, planned velocity, packet/error counts, and latest sender.

Live UDP rows are rendered immediately. Recording writes the same fixed `I0-I42` CSV schema. CSV import switches to replay mode and uses timestamp deltas for playback, scaled by the selected speed.

## Isolation

The target uses only its own protocol, CSV, camera widget, and main-window files. It links Qt6 Core, Network, and Widgets. It does not compile or link files from the main BeaconImageAnalyzer target.

## Verification

- Unit tests verify exact 172/176-byte parsing, channel mapping, invalid lengths/tails, and CSV save/load round trips.
- Release build must produce `JustFloatMonitor.exe`.
- The packaged application must remain running after launch.
- A portable ZIP includes the Qt and MinGW runtime dependencies needed on another Windows machine.
