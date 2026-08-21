# uav_vision — Visual Perception Package

**Meta:** `catkin_make` workspace. Active ROS1 package. Depends on `cv_bridge`, `image_transport`, `roscpp`, `rospy`, `sensor_msgs`, `libapriltag-dev`.

## OVERVIEW

Active competition perception uses two message-gated USB cameras and two independent instances of the Python OpenVINO-board-detector plus multi-template classifier. The front camera recognizes the first board and the rear camera recognizes the second board without a yaw turn. The legacy C++ AprilTag landing implementation remains in the package for reference but is not included by `dual_target_vision.launch`.

## NODES

| Binary | Lang | Source | Role |
|--------|------|--------|------|
| `landing_tag_node` | C++14 | `src/landing_tag_node.cpp` | Legacy AprilTag detector; inactive in the competition runtime |
| `target_match_node` | Python | `scripts/target_match_node.py` | Front/rear OpenVINO board detection followed by square-board classification (plane/car/ship/house) |
| `v4l2_camera_node` | Python | `scripts/v4l2_camera_node.py` | V4L2 camera acquisition controlled by `/vision/control`; reapplies manual exposure/gain when opened |
| `landing_debug_web` | Python | `scripts/landing_debug_web.py` | MJPEG debug stream viewer |

## TOPICS

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| subscribe | `/vision/control` | `uav_vision_msgs/VisionControl` | Desired front/rear camera state; the legacy `down_camera_enabled` field controls rear |
| subscribe | `/vision/rear/image_raw` | `sensor_msgs/Image` | Rear target camera (1280x720 MJPG @ 30Hz) |
| subscribe | `/vision/front/image_raw` | `sensor_msgs/Image` | Target classification camera |
| publish | `/vision/target/result` | `uav_vision_msgs/TargetMatchArray` | Classification result with fused score + corners |
| publish | `/vision/target/front/debug_image` | `sensor_msgs/Image` | Front matched quad + class overlay |
| publish | `/vision/target/rear/debug_image` | `sensor_msgs/Image` | Rear matched quad + class overlay |

## TESTS

| File | Type | Count | Coverage |
|------|------|-------|----------|
| `test/test_landing_core.cpp` | GTest | 16 | OffsetEstimator, yaw-preserving roll/pitch stabilization, observed down-camera axis mapping, live-extrinsic matrix construction, detector and debug formatter |
| `test/test_target_matcher.py` | nosetests | 7 | corner ordering, template scoring, edge/white-board candidate detection, blank frame rejection, recording overlay |
| `test/test_temporal_vote.py` | nosetests | 5 | voting stability, tie handling, consecutive unknown reset, config validation |
| `test/test_camera_focus.py` | nosetests | 3 | sharpness metric, FPS meter |
| `test/test_v4l2_camera_node.py` | nosetests | 10 | settings, control readback, camera-role routing, idempotent enable state and guarded publish |
| `test/test_landing_debug_web.py` | nosetests | 7 | LatestJpegFrame, HTML/control visibility, extrinsic input validation, independent target web launch |

## NOTES

The following landing bullets document dormant legacy code only; they are not
part of the active front/rear target runtime and require no live extrinsic.

- **Landing center geometry**: ID 4 is the physical center tag. If it is visible, its 2D center is used directly. Without ID 4, four corner tags use the projected diagonal intersection, three corners use the longest-pair midpoint approximation, and one/two corners fall back to their visible mean. All outputs remain pixel-scale coordinates; no calibrated camera model is assumed.
- **Landing output frame**: the node builds `[dx/f, dy/f, 1]`, maps it through the live camera-to-body extrinsic, removes fused roll/pitch while retaining aircraft yaw, maps it back to camera axes, then reprojects. Thus pure yaw preserves signed image dx/dy and the output remains directly comparable to raw pixels. It is not metric ground reconstruction and does not compensate translation.
- **Live extrinsic tuning**: `pose/camera_to_body_rpy_deg` is `[roll, pitch, yaw]` in degrees, composed as `Rz*Ry*Rx` and polled at 5 Hz. The confirmed mounting is `[180,0,90]`: image `+X -> body +Y`, image `+Y -> body +X`, optical `+Z -> body -Z`. The landing web provides live controls and a matching preset.
- **`LandingOffset.valid`** semantics: after a valid estimate, 1-4 consecutive empty detections reuse the last filtered pixel offset and tag IDs; the 5th returns `valid=false` with NaN and clears the held center. Before the first detection, an empty frame is immediately invalid. Quality-gate and pixel-jump failures remain immediately invalid.
- **Pose fallback**: a fresh valid fused attitude enables world-plane rotation. If the pose has not arrived, is stale, or has an invalid quaternion, the node keeps `valid=true` and publishes the original image-plane pixel offset.
- **Temporal voting**: 5-frame sliding window, requires min 3 consecutive consistent labels. `target_lost_frames=3` clears history on consecutive unknowns.
- **Camera gating**: `front_camera_enabled` activates only the front camera/matcher. The legacy `down_camera_enabled` field activates only the rear camera/matcher. Both classifier nodes stay alive, reset temporal state on every enable transition, and share `/vision/target/result` because mission stages enable only one role at a time.
- **Camera config**: 1280x720 MJPG @ 30Hz, manual exposure 150, gain 5. Both UAVs map USB path `0:2` to `/dev/uav_front_camera`; UAV1 maps rear path `0:3` and UAV2 maps rear path `0:7` to `/dev/uav_rear_camera`.
- **Classification**: ORB/AKAZE geometry and Canny polygon quads are supplemented by an HSV white-board path tuned from the venue recording (`S<=60`, `V>=170`, 5x5 open/close). The recovered quad then uses the existing gray+HOG+color weighted match (0.5/0.3/0.2). Temporal voting rejects isolated white-region false candidates.
- **Detector cascade**: a 512x512 OpenVINO single-class detector first localizes the strongest board (`confidence>=0.10`). The box is expanded by `0.75*side`, rectified by the geometry matcher at up to 1280 px, then classified. Only when geometry fails does the node use the stricter direct crop gates (`score>=0.63`, `margin>=0.06`). The model is installed from `models/target_board_openvino/`.
- **Processing rate**: USB acquisition and ROS image publication remain 30 Hz. Each target node samples inference and its paired raw/result recording together at 10 Hz, matching the recorder configuration and reserving CPU for flight planning/control.
- **Thread limits**: `target_match.launch` fixes OpenBLAS/MKL/NumExpr to one worker and OpenMP to four. Do not remove these launch-level variables: NumPy's small template-score dot products otherwise create one worker per logical CPU on the 16-thread NUC.
- **OpenVINO runtime**: production ROS uses system Python 3.8. The NUC deployment provisions the existing OpenVINO 2024.4 target directory through a user-site `.pth` file, so `roslaunch` and the `jiangyin` competition shortcut require no extra environment prefix.
- **Recording**: each active matcher writes paired raw/result MP4 files asynchronously under `front_target/NNN` or `rear_target/NNN`; the bounded queue drops recording frames instead of delaying recognition.
- **Runtime launch**: use `dual_target_vision_uav1.launch` on UAV1 and generic `dual_target_vision.launch` on UAV2. `dual_target_debug_web.launch` serves front on port 8081 and rear on port 8083. No landing pose or camera extrinsic is needed.
- Custom msgs: `LandingOffset`, `TargetMatch`, `TargetMatchArray`, `VisionControl` defined in external `uav_vision_msgs` package.
- Full runtime guide: `docs/uav_vision_runtime_guide.md` (Chinese).
