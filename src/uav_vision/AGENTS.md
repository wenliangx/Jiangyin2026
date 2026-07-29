# uav_vision — Visual Perception Package

**Meta:** `catkin_make` workspace. Active ROS1 package (8 C++, 13 Python, 5 launch, 4 config, 4 test). Depends on `cv_bridge`, `image_transport`, `roscpp`, `rospy`, `sensor_msgs`, `libapriltag-dev`.

## OVERVIEW

Visual perception for UAV landing and wall targets. Two independent chains: C++ landing-tag detector (down camera, AprilTag tag36h11) and Python multi-template target classifier (front camera, gray+HOG+color fusion).

## NODES

| Binary | Lang | Source | Role |
|--------|------|--------|------|
| `landing_tag_node` | C++14 | `src/landing_tag_node.cpp` | AprilTag detection + offset estimation (library: `uav_landing_core`) |
| `target_match_node` | Python | `scripts/target_match_node.py` | Multi-template classification (square board → plane/car/ship/house) |
| `v4l2_camera_node` | Python | `scripts/v4l2_camera_node.py` | V4L2 camera spawner with manual exposure/gain setup |
| `landing_debug_web` | Python | `scripts/landing_debug_web.py` | MJPEG debug stream viewer |

## TOPICS

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| subscribe | `/vision/down/image_raw` | `sensor_msgs/Image` | Landing tag camera (1280x720 MJPG @ 30Hz) |
| subscribe | `/vision/front/image_raw` | `sensor_msgs/Image` | Target classification camera |
| publish | `/vision/landing/offset` | `uav_vision_msgs/LandingOffset` | Pixel-space landing deviation, `valid` = all 5 tags present |
| publish | `/vision/target/result` | `uav_vision_msgs/TargetMatchArray` | Classification result with fused score + corners |
| publish | `/vision/landing/debug_image` | `sensor_msgs/Image` | Tag bounding boxes + offset line overlay |
| publish | `/vision/target/debug_image` | `sensor_msgs/Image` | Matched quad + annotated class overlay |

## TESTS

| File | Type | Count | Coverage |
|------|------|-------|----------|
| `test/test_landing_core.cpp` | GTest | 6 | OffsetEstimator (order-independence, invalid frames, median/jump reset), tag detector synthetic image, debug formatter |
| `test/test_target_matcher.py` | nosetests | 5 | corner ordering, template scoring, square candidate detection, blank frame rejection |
| `test/test_temporal_vote.py` | nosetests | 5 | voting stability, tie handling, consecutive unknown reset, config validation |
| `test/test_camera_focus.py` | nosetests | 3 | sharpness metric, FPS meter |
| `test/test_v4l2_camera_node.py` | nosetests | 4 | validate settings, control applier/readback |
| `test/test_landing_debug_web.py` | nosetests | 1 | LatestJpegFrame singleton |

## NOTES

- **`LandingOffset.valid`** semantics: `true` only when all 5 tags (IDs 0-4) are detected simultaneously with margin >= 20.0. Returns NaN otherwise.
- **Temporal voting**: 5-frame sliding window, requires min 3 consecutive consistent labels. `target_lost_frames=3` clears history on consecutive unknowns.
- **Camera config**: 1280x720 MJPG @ 30Hz, manual exposure 150, gain 5 (see `config/camera_common.yaml` + `udev/99-uav-cameras.rules`).
- **Classification**: Canny edge → polygon quad → gray+HOG+color weighted match (0.5/0.3/0.2). 13 augmentation tuples for template matching.
- Custom msgs: `LandingOffset`, `TargetMatch`, `TargetMatchArray` defined in external `uav_vision_msgs` package.
- Full runtime guide: `docs/uav_vision_runtime_guide.md` (Chinese).