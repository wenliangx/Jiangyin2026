# uav_vision — Visual Perception Package

**Meta:** `catkin_make` workspace. Active ROS1 package (8 C++, 13 Python, 5 launch, 4 config, 4 test). Depends on `cv_bridge`, `image_transport`, `roscpp`, `rospy`, `sensor_msgs`, `libapriltag-dev`.

## OVERVIEW

Visual perception for UAV landing and wall targets. Two independent chains: C++ landing-tag detector (down camera, AprilTag tag36h11) and Python multi-template target classifier (front camera, gray+HOG+color fusion).

## NODES

| Binary | Lang | Source | Role |
|--------|------|--------|------|
| `landing_tag_node` | C++14 | `src/landing_tag_node.cpp` | AprilTag detection + offset estimation (library: `uav_landing_core`) |
| `target_match_node` | Python | `scripts/target_match_node.py` | Multi-template classification (square board → plane/car/ship/house) |
| `v4l2_camera_node` | Python | `scripts/v4l2_camera_node.py` | V4L2 camera acquisition controlled by `/vision/control`; reapplies manual exposure/gain when opened |
| `landing_debug_web` | Python | `scripts/landing_debug_web.py` | MJPEG debug stream viewer |

## TOPICS

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| subscribe | `/vision/control` | `uav_vision_msgs/VisionControl` | 50Hz desired front/down camera state; consumed by camera nodes, not recognition nodes |
| subscribe | `/vision/down/image_raw` | `sensor_msgs/Image` | Landing tag camera (1280x720 MJPG @ 30Hz) |
| subscribe | `/vision/front/image_raw` | `sensor_msgs/Image` | Target classification camera |
| subscribe | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | PX4 fused body attitude used to remove roll/pitch while retaining aircraft yaw |
| publish | `/vision/landing/offset` | `uav_vision_msgs/LandingOffset` | Pixel-space landing deviation from the inferred 2D landing center |
| publish | `/vision/target/result` | `uav_vision_msgs/TargetMatchArray` | Classification result with fused score + corners |
| publish | `/vision/landing/debug_image` | `sensor_msgs/Image` | Tag boxes plus raw (magenta) and leveled/fallback (cyan/yellow) offset arrows, pose RPY/age, and camera-to-body matrix |
| publish | `/vision/target/debug_image` | `sensor_msgs/Image` | Matched quad + annotated class overlay |

## TESTS

| File | Type | Count | Coverage |
|------|------|-------|----------|
| `test/test_landing_core.cpp` | GTest | 16 | OffsetEstimator, yaw-preserving roll/pitch stabilization, observed down-camera axis mapping, live-extrinsic matrix construction, detector and debug formatter |
| `test/test_target_matcher.py` | nosetests | 5 | corner ordering, template scoring, square candidate detection, blank frame rejection |
| `test/test_temporal_vote.py` | nosetests | 5 | voting stability, tie handling, consecutive unknown reset, config validation |
| `test/test_camera_focus.py` | nosetests | 3 | sharpness metric, FPS meter |
| `test/test_v4l2_camera_node.py` | nosetests | 10 | settings, control readback, camera-role routing, idempotent enable state and guarded publish |
| `test/test_landing_debug_web.py` | nosetests | 7 | LatestJpegFrame, HTML/control visibility, extrinsic input validation, independent target web launch |

## NOTES

- **Landing center geometry**: ID 4 is the physical center tag. If it is visible, its 2D center is used directly. Without ID 4, four corner tags use the projected diagonal intersection, three corners use the longest-pair midpoint approximation, and one/two corners fall back to their visible mean. All outputs remain pixel-scale coordinates; no calibrated camera model is assumed.
- **Landing output frame**: the node builds `[dx/f, dy/f, 1]`, maps it through the live camera-to-body extrinsic, removes fused roll/pitch while retaining aircraft yaw, maps it back to camera axes, then reprojects. Thus pure yaw preserves signed image dx/dy and the output remains directly comparable to raw pixels. It is not metric ground reconstruction and does not compensate translation.
- **Live extrinsic tuning**: `pose/camera_to_body_rpy_deg` is `[roll, pitch, yaw]` in degrees, composed as `Rz*Ry*Rx` and polled at 5 Hz. The confirmed mounting is `[180,0,90]`: image `+X -> body +Y`, image `+Y -> body +X`, optical `+Z -> body -Z`. The landing web provides live controls and a matching preset.
- **`LandingOffset.valid`** semantics: after a valid estimate, 1-4 consecutive empty detections reuse the last filtered pixel offset and tag IDs; the 5th returns `valid=false` with NaN and clears the held center. Before the first detection, an empty frame is immediately invalid. Quality-gate and pixel-jump failures remain immediately invalid.
- **Pose fallback**: a fresh valid fused attitude enables world-plane rotation. If the pose has not arrived, is stale, or has an invalid quaternion, the node keeps `valid=true` and publishes the original image-plane pixel offset.
- **Temporal voting**: 5-frame sliding window, requires min 3 consecutive consistent labels. `target_lost_frames=3` clears history on consecutive unknowns.
- **Camera config**: 1280x720 MJPG @ 30Hz, manual exposure 150, gain 5 (see `config/camera_common.yaml` + `udev/99-uav-cameras.rules`). Camera nodes idempotently open/release capture from the repeated desired state; recognition nodes stay alive and process every arriving frame.
- **Classification**: Canny edge → polygon quad → gray+HOG+color weighted match (0.5/0.3/0.2). 13 augmentation tuples for template matching.
- Custom msgs: `LandingOffset`, `TargetMatch`, `TargetMatchArray`, `VisionControl` defined in external `uav_vision_msgs` package.
- Full runtime guide: `docs/uav_vision_runtime_guide.md` (Chinese).
