# WiFi JustFloat Planner Window Design

## Scope

Upgrade JustFloatMonitor to support only the latest WiFi JustFloat packet: one automatic timestamp plus 69 user floats. Preserve UDP listening, in-memory CSV recording, CSV replay, and the existing dark Qt Widgets UI. Add one independent top-level planner window without adding another UDP socket.

## Protocol

- Accept exactly 70 little-endian float32 values: 280 bytes without a tail or 284 bytes with `00 00 80 7F`.
- Reject the old 43-float packet and every CSV row that is not exactly 70 columns.
- `TelemetryProtocol` is the only byte/channel mapping implementation.
- `TelemetryFrame` contains three raw cameras, three Center-mapped beacons and lamp, three CameraModel beacons and lamp, vehicle state, aircraft state, and `selectedTargetId`.
- Raw cameras consume I1-I30 as three consecutive 10-value groups. Car lamps contain x, y, angle, and length only.
- Center mapped values consume I31-I46, CameraModel values I47-I59, vehicle I60-I64, aircraft I65-I68, and selected target I69.
- Non-finite coordinates or values at or below -900 are invalid. Beacon area, lamp length, and fused camera mask must be positive for the corresponding object to be valid.

## Data Flow And Performance

`MainWindow` remains the sole owner of `QUdpSocket`. Its ready-read loop drains all pending datagrams. Every valid frame increments counters and is appended to an active recording. The newest parsed frame replaces a pending UI frame. A 16 ms timer refreshes the visible UI from only the latest pending frame, preventing 200 Hz packets from producing 200 Hz repaints.

CSV replay also supplies parsed `TelemetryFrame` objects to the same display path. No drawing widget reads raw channel indexes.

## Planner Window

`PlannerDebugWindow` is a separate resizable top-level QWidget. Starting UDP listening automatically shows it. A `规划窗口` button reopens it. Closing it hides only that window; UDP and recording continue. Stopping UDP leaves the window available but changes its state to `UDP 已停止` and clears the live presentation. Destroying `MainWindow` closes the planner window.

The layout contains Front, Center, and Back raw camera views, one Center-mapped coordinate view, one CameraModel coordinate view, and a compact state panel.

## Drawing And Interaction

Normal canvases show only grids/axes, beacon shapes, car-lamp shapes, off-screen indicators, and selected-target highlighting. Persistent labels such as B0, F1, CAR, TARGET, coordinates, masks, and source text are not drawn.

All canvases maintain screen-space hit targets. Mouse tracking selects the closest target within an 8-12 pixel screen radius and displays one reusable `QToolTip`. Moving away immediately hides it. Tooltips include the semantic slot, coordinate system, raw values, validity, selection state, and Center camera-mask source where applicable.

Raw `CameraView` keeps the existing 188 x 120 center-origin mapping. Beacon radii are clamped to a stable screen range. Car lamps use angle and length with fixed visual thickness.

One reusable `CoordinateView` implements Center mapped and CameraModel modes. Slot colors are fixed and shared between modes. Selected slots receive an extra bright outer ring and cross without a text label. Center defaults to X -180..180 and Y -120..120. CameraModel defaults to X -120..120 and Y -90..90. Wheel input zooms around the cursor, dragging pans, and an icon reset button restores the default range. Neither view automatically rescales per frame. Off-screen objects are clamped to the canvas edge and rendered as direction indicators while retaining their real coordinates for tooltips.

## Verification

- Protocol tests cover 280/284-byte packets, invalid tails and old packet sizes, semantic field mapping, invalid sentinels, selected target values, and 70-column CSV round trips with realistic values.
- Build and CTest must pass.
- Desktop verification covers automatic planner-window opening, independent move/resize, hide/reopen behavior, stopped UDP state, selected-slot highlighting, tooltips, pan/zoom/reset, off-screen indicators, and main-window shutdown.
