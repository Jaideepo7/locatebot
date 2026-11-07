# Locate Bot — project blueprint

## Goal
Build a Jetson Orin Nano Super RC-car research prototype that can identify a selected target and navigate toward a reachable stand-off position in a bounded indoor area. LiDAR constrains motion; asynchronous camera perception identifies targets. Inventory management is a later extension.

## Staged success criteria
| Stage | Deliverable | Required evidence |
|---|---|---|
| Now | Documentation only | Reviewable Markdown |
| Portable core | Four modules, fake vehicle, synthetic inputs | Unit/contract/integration tests |
| Sensor integration | Actual LiDAR acquisition, display, recording | Measured distances, orientation and fault tests |
| Supervised motion | Manual motion with safety overrides | Physical cutoff, MCU watchdog, calibrated limits |
| Navigation | Reach a known safe goal | Repeated bounded-area trials, blocked-path handling |
| Target finding | Select target and generate valid approach goal | Identity, spatial uncertainty, stand-off and loss handling |

A fake command log is not a physical simulator or proof of collision avoidance. Target bearing alone does not implement metric navigation.

## What to do right now
- Review and correct the architecture and contracts.
- Assign a human owner to each proposed lane.
- Inventory exact hardware and software below.
- Agree on one realistic hackathon demonstration.
- Keep implementation, agent execution, installs and hardware work paused until explicit approval.

## Future build order after approval
1. Integrator freezes immutable contracts, package/test structure and ownership rules.
2. Four lanes work independently with tests first: safety, control, perception, mission.
3. Review complete slices and merge into integration with a full test run after each merge.
4. Prove blocked inference does not block control using explicitly synthetic inputs.
5. Add actual LiDAR read/record/replay; validate driver timestamps and quality flags.
6. Implement independent microcontroller watchdog and calibrated actuator adapter, disabled by default.
7. Add measured vehicle geometry, encoders/IMU odometry and sensor transforms.
8. Choose a ROS 2 stack compatible with the installed JetPack/Ubuntu; integrate car-compatible mapping/navigation.
9. Navigate to a known AprilTag station with a reachable stand-off pose.
10. Benchmark LocateAnything on the actual Jetson, then integrate a supported asynchronous backend if useful.
11. Add barcode/RFID/database inventory functions only after reliable missions.

## 24-hour allocation proposal
| Window | Focus | Exit decision |
|---|---|---|
| 0–2 h | Inventory, setup, contracts | Eliminate unsupported stack choices |
| 2–7 h | Parallel portable core | Only tested slices enter review |
| 7–10 h | Integration/failure injection | Fix core before hardware |
| 10–16 h | Sensor replay and gated actuation | No hardware-safety shortcuts |
| 16–20 h | One bounded navigation/target demo | Freeze optional features |
| 20–24 h | Repeat trials, evidence, presentation | Declare limitations |
These are planning allocations, not measured completion promises. Cut custom quantization, exploration, inventory features and elaborate UI before cutting safety/testing. If navigation is not ready, show narrower verified behaviour rather than claiming warehouse autonomy.

## Hardware and setup worksheet
| Item | Exact model/version | Observed working evidence | Owner |
|---|---|---|---|
| Jetson/JetPack/Ubuntu | Unconfirmed | Unconfirmed | Unassigned |
| LiDAR/interface | Unconfirmed | Unconfirmed | Unassigned |
| Camera/depth/intrinsics | Unconfirmed | Unconfirmed | Unassigned |
| Chassis/wheelbase/footprint | Unconfirmed | Unconfirmed | Unassigned |
| ESC/steering servo/interface | Unconfirmed | Unconfirmed | Unassigned |
| MCU/watchdog/cutoff | Unconfirmed | Unconfirmed | Unassigned |
| Encoders/IMU | Unconfirmed | Unconfirmed | Unassigned |
| Power conversion/cables | Unconfirmed | Unconfirmed | Unassigned |
| Git remote/access | Unconfirmed | Unconfirmed | Unassigned |
| Demo area/obstacle types | Unconfirmed | Unconfirmed | Unassigned |

The reported 35 W USB-C power bank does not prove compatible Jetson power. Verify negotiated voltage, connector/polarity, conversion, peak current and motor interference. No runtime estimate is established.

## First-batch non-goals
No physical actuator output, ROS deployment, SLAM, actual LocateAnything inference, object-depth estimation, arrival detection, inventory counting or commercial readiness. Those remain explicit later tasks, not features to mark complete with placeholders.
