# UAV Template Classifier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a ROS1 Noetic package that identifies one of four fixed wall targets with square-board localization and multi-template scoring, plus a browser-based USB camera focus viewer.

**Architecture:** `uav_vision_msgs` owns the ROS result contract. `uav_vision` contains ROS-independent Python modules for scoring, square-board matching, and temporal voting, with a thin ROS image subscriber. The focus viewer reads a selected V4L2 device directly and exposes an MJPEG stream over the local network.

**Tech Stack:** ROS1 Noetic, catkin, Python 3, rospy, OpenCV 4.2, NumPy, cv_bridge, stdlib HTTP server, unittest.

---

### Task 1: Preserve the existing branch state

**Files:**
- Do not modify existing files outside `docs/superpowers/plans`, `src/uav_vision_msgs`, and `src/uav_vision`.

- [ ] **Step 1: Confirm the branch**

Run:

```bash
cd /home/flag/Jiangyin2026
git branch --show-current
```

Expected: `plan_yaw`.

- [ ] **Step 2: Record unrelated changes**

Run:

```bash
git status --short
```

Expected: existing edits under `src/SUPER`, `src/fsm_ctrl`, and root diagnostic scripts remain untouched.

### Task 2: Define the result messages

**Files:**
- Create: `src/uav_vision_msgs/CMakeLists.txt`
- Create: `src/uav_vision_msgs/package.xml`
- Create: `src/uav_vision_msgs/msg/LandingOffset.msg`
- Create: `src/uav_vision_msgs/msg/TargetMatch.msg`
- Create: `src/uav_vision_msgs/msg/TargetMatchArray.msg`

- [ ] **Step 1: Add the three message definitions**

```text
# LandingOffset.msg
std_msgs/Header header
bool valid
float32 dx
float32 dy
float32 center_x
float32 center_y
uint8 tag_count
int32[] tag_ids

# TargetMatch.msg
string label
float32 score
float32 gray_score
float32 hog_score
float32 color_score
float32 margin
float32 sharpness
uint16 target_side_px
float32[8] corners

# TargetMatchArray.msg
std_msgs/Header header
bool valid
uav_vision_msgs/TargetMatch[] matches
```

- [ ] **Step 2: Configure message generation**

`CMakeLists.txt` must call `add_message_files`, `generate_messages(DEPENDENCIES std_msgs)`, and export `message_runtime`.

- [ ] **Step 3: Build and inspect**

Run:

```bash
cd /home/flag/Jiangyin2026
source /opt/ros/noetic/setup.zsh
catkin_make --pkg uav_vision_msgs
source devel/setup.zsh
rosmsg show uav_vision_msgs/TargetMatchArray
```

Expected: build succeeds and the nested `TargetMatch[] matches` field is visible.

### Task 3: Build the focus viewer test-first

**Files:**
- Create: `src/uav_vision/test/test_camera_focus.py`
- Create: `src/uav_vision/scripts/camera_focus_view.py`

- [ ] **Step 1: Write the failing unit test**

```python
class FocusMetricTest(unittest.TestCase):
    def test_uniform_image_has_zero_sharpness(self):
        frame = np.full((64, 64, 3), 127, np.uint8)
        self.assertEqual(camera_focus_view.measure_sharpness(frame), 0.0)

    def test_checkerboard_is_sharper_than_uniform_image(self):
        uniform = np.full((64, 64, 3), 127, np.uint8)
        checker = np.indices((64, 64)).sum(axis=0) % 2
        checker = cv2.cvtColor((checker * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        self.assertGreater(
            camera_focus_view.measure_sharpness(checker),
            camera_focus_view.measure_sharpness(uniform),
        )
```

- [ ] **Step 2: Verify RED**

Run:

```bash
python3 -m unittest src/uav_vision/test/test_camera_focus.py -v
```

Expected: import failure because `camera_focus_view.py` does not exist.

- [ ] **Step 3: Implement the viewer**

Implement:

```python
def measure_sharpness(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F, ksize=3).var())
```

The executable must:

- require `--device`;
- request MJPG, width, height, and FPS;
- overlay device, resolution, FPS, and sharpness;
- serve `/` and `/stream.mjpg` using `ThreadingHTTPServer`;
- fail with a clear message if the device cannot be opened.

- [ ] **Step 4: Verify GREEN**

Run:

```bash
python3 -m unittest src/uav_vision/test/test_camera_focus.py -v
python3 src/uav_vision/scripts/camera_focus_view.py --help
```

Expected: both unit tests pass and help lists `--device`, `--host`, `--port`, `--width`, `--height`, and `--fps`.

### Task 4: Implement scoring and matching test-first

**Files:**
- Create: `src/uav_vision/setup.py`
- Create: `src/uav_vision/src/uav_vision/__init__.py`
- Create: `src/uav_vision/src/uav_vision/target_matcher.py`
- Create: `src/uav_vision/test/test_target_matcher.py`

- [ ] **Step 1: Write failing tests**

Tests must cover:

```python
def test_orders_quad_corners_clockwise_from_top_left():
    points = np.float32([[90, 90], [10, 10], [90, 10], [10, 90]])
    ordered = order_quad(points)
    np.testing.assert_allclose(
        ordered,
        np.float32([[10, 10], [90, 10], [90, 90], [10, 90]]),
    )

def test_exact_template_scores_higher_than_other_class(tmp_path):
    templates = make_four_distinct_templates(tmp_path)
    matcher = TargetMatcher(test_config(), str(templates))
    result = matcher.classify_patch(cv2.imread(str(templates / "plane.png")))
    assert result.label == "plane"
    assert result.score > result.second_score

def test_rejects_ambiguous_patch(tmp_path):
    matcher = TargetMatcher(test_config(min_class_margin=0.20), str(templates))
    result = matcher.classify_patch(np.full((256, 256, 3), 127, np.uint8))
    assert not result.valid
```

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/uav_vision/src python3 -m unittest src/uav_vision/test/test_target_matcher.py -v
```

Expected: import failure for `uav_vision.target_matcher`.

- [ ] **Step 3: Implement the minimal matcher**

Implement:

- `MatcherConfig`;
- `order_quad`;
- shared `preprocess_patch`;
- deterministic template variants from explicit YAML tuples;
- grayscale `TM_CCOEFF_NORMED`;
- fixed OpenCV HOG cosine similarity;
- normalized H-S histogram correlation;
- convex quadrilateral candidate extraction;
- perspective warp;
- per-candidate class threshold and margin;
- cross-candidate `min_candidate_margin`;
- `unknown` rejection with a reason string.

- [ ] **Step 4: Verify GREEN**

Run:

```bash
PYTHONPATH=src/uav_vision/src python3 -m unittest src/uav_vision/test/test_target_matcher.py -v
```

Expected: all matcher tests pass.

### Task 5: Implement temporal voting test-first

**Files:**
- Create: `src/uav_vision/src/uav_vision/temporal_vote.py`
- Create: `src/uav_vision/test/test_temporal_vote.py`

- [ ] **Step 1: Write failing tests**

```python
def test_unknown_occupies_window_without_voting():
    voter = TemporalVoter(window_size=3, min_votes=2, lost_frames=3)
    assert voter.update("plane") is None
    assert voter.update(None) is None
    assert voter.update("plane") == "plane"

def test_current_frame_must_match_stable_label():
    voter = TemporalVoter(window_size=3, min_votes=2, lost_frames=3)
    voter.update("plane")
    voter.update("plane")
    assert voter.update(None) is None

def test_consecutive_unknown_clears_history():
    voter = TemporalVoter(window_size=5, min_votes=2, lost_frames=2)
    voter.update("car")
    voter.update("car")
    voter.update(None)
    voter.update(None)
    assert voter.update("car") is None
```

- [ ] **Step 2: Verify RED**

Run:

```bash
PYTHONPATH=src/uav_vision/src python3 -m unittest src/uav_vision/test/test_temporal_vote.py -v
```

Expected: import failure for `uav_vision.temporal_vote`.

- [ ] **Step 3: Implement `TemporalVoter`**

Use a fixed-length deque containing labels or `None`. Require a unique plurality, at least `min_votes`, and a current frame matching the winner. Clear the deque after `lost_frames` consecutive `None` values.

- [ ] **Step 4: Verify GREEN**

Run the same unittest command. Expected: all voter tests pass.

### Task 6: Add the ROS package and node

**Files:**
- Create: `src/uav_vision/CMakeLists.txt`
- Create: `src/uav_vision/package.xml`
- Create: `src/uav_vision/scripts/target_match_node.py`
- Create: `src/uav_vision/config/target_match.yaml`
- Create: `src/uav_vision/launch/target_match.launch`

- [ ] **Step 1: Add catkin Python packaging**

Declare dependencies on `rospy`, `std_msgs`, `sensor_msgs`, `cv_bridge`, `image_transport`, and `uav_vision_msgs`. Install both scripts with `catkin_install_python`.

- [ ] **Step 2: Implement the thin ROS adapter**

The node must:

- subscribe with queue size 1;
- convert `sensor_msgs/Image` to BGR;
- call `TargetMatcher.match_frame`;
- pass the accepted label or `None` to `TemporalVoter`;
- publish exactly one match when stable and current, otherwise an empty invalid result;
- publish an optional annotated debug image;
- inherit the input header;
- never reuse an old valid result.

- [ ] **Step 3: Add configuration**

The YAML must contain explicit template augmentation tuples and all geometry, quality, score, and temporal thresholds. Initial thresholds are provisional until real printed-target data exists.

- [ ] **Step 4: Add launch file**

Expose `image_topic`, `result_topic`, `debug_topic`, `templates_dir`, and `publish_debug_image` as arguments.

### Task 7: Add the four PDF-derived templates

**Files:**
- Create: `src/uav_vision/templates/plane.png`
- Create: `src/uav_vision/templates/car.png`
- Create: `src/uav_vision/templates/ship.png`
- Create: `src/uav_vision/templates/house.png`

- [ ] **Step 1: Crop the four image regions**

Crop from the high-resolution rendered PDF page, excluding the black square outline and English labels.

- [ ] **Step 2: Visually inspect**

Expected: each file contains only its image content, is square, and has no label text or black frame.

- [ ] **Step 3: Run a self-match test**

Load each template and verify its predicted label equals its filename stem.

### Task 8: Build and test on the NUC

- [ ] **Step 1: Run all unit tests**

```bash
cd /home/flag/Jiangyin2026
PYTHONPATH=src/uav_vision/src python3 -m unittest discover -s src/uav_vision/test -v
```

Expected: all tests pass.

- [ ] **Step 2: Build both packages**

```bash
source /opt/ros/noetic/setup.zsh
catkin_make --pkg uav_vision_msgs uav_vision
source devel/setup.zsh
```

Expected: build succeeds with no missing dependencies.

- [ ] **Step 3: Check Python syntax and ROS metadata**

```bash
python3 -m compileall -q src/uav_vision
rospack find uav_vision
rosmsg show uav_vision_msgs/TargetMatch
```

Expected: all commands succeed.

- [ ] **Step 4: Smoke-test the focus viewer**

```bash
python3 src/uav_vision/scripts/camera_focus_view.py \
  --device /dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0 \
  --host 0.0.0.0 --port 8080 --width 1280 --height 720 --fps 30
```

Open `http://10.1.77.193:8080` from the Windows browser. Expected: live front-camera video with an updating sharpness value. Physical USB path 3 was confirmed as the front camera; path 2 remains the provisional down-facing camera until the landing prop is available.

- [ ] **Step 5: Review repository scope**

Run:

```bash
git status --short
git diff -- src/uav_vision_msgs src/uav_vision docs/superpowers/plans
```

Expected: only new vision paths belong to this work; pre-existing user changes remain unstaged and untouched.
