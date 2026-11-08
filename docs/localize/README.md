# Item localization (Orin)

**Status:** built and tested on synthetic scans. Not yet fed by the real vision worker, camera or LiDAR.

`locatebot/localize/items.py`
- `localize(bearing_rad, scan, pose, LocalizeConfig) -> ItemFix | None`: looks along the camera bearing (from the camera's position on the car) in the LiDAR scan taken at the frame's capture time, uses the nearest return as the range, and places the point on the map with the pose at that time. Returns `None` with no return in the ray window or beyond `max_range_m`: **no range, no position**.
- `PoseHistory.at(t)`: pose at a frame's capture time (linear interpolation, heading through the short way round; refuses times outside the recorded span beyond a small hold).
- `ScanHistory.nearest(t)`: scan closest to the capture time, within `max_dt_s`.
- `ItemTracker`: merges sightings of the same item within `merge_radius_m` (inverse-variance weighted); separate tracks for far-apart sightings (second instance or false detection); `confirmed` after `min_sightings`; `best(item)` = most sightings.

Timing: the bearing is in the robot frame at the frame's capture time, but the ray is searched in the scan's frame. `ScanHistory` defaults to `max_dt_s = 0.06` (about half a 10 Hz scan period) to keep the turning error small; pass the pose at the frame capture time. Motion within one LiDAR revolution is not corrected.

Uncertainty per fix: `hypot(range_sigma_m, range * bearing_sigma_rad)`, so far items are less certain.

## Wiring (planned)
vision result (item, bearing, capture time) → `ScanHistory.nearest` + `PoseHistory.at` → `localize` → `ItemTracker.update` → pin on the map.

## Measure on the car
- `camera_xy`: camera position relative to the LiDAR centre (metres, robot frame).
- Camera/LiDAR heading alignment: point the car at a post and compare the vision bearing with the LiDAR bearing.
- `bearing_sigma_rad` from repeated detections of a fixed item.
- Camera and LiDAR clocks: capture times must be on the same clock as scan stamps.

Limits: items must be roughly at LiDAR height; positions inherit odometry drift.
