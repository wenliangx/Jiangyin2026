# UAV Camera udev Rules Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create stable `/dev/uav_front_camera` and `/dev/uav_down_camera` names that bind the front and down cameras to their confirmed USB physical ports.

**Architecture:** Keep the authoritative rule in the `uav_vision` package, then install the same file into `/etc/udev/rules.d`. Match `video4linux`, the confirmed `ID_PATH`, and `ATTR{index}=="0"` so `/dev/videoX` enumeration changes and the secondary non-capture nodes do not affect the ROS configuration.

**Tech Stack:** Ubuntu 20.04 udev, V4L2, ROS1 Noetic workspace `/home/flag/Jiangyin2026`

---

### Task 1: Record a clean hardware preflight

**Files:**
- Inspect: `/home/flag/Jiangyin2026/src/uav_vision`
- Inspect: `/etc/udev/rules.d`

- [ ] **Step 1: Confirm the branch and preserve unrelated changes**

Run:

```bash
cd /home/flag/Jiangyin2026
git branch --show-current
git status --short
```

Expected: branch is `plan_yaw`. Record the existing modified and untracked files; do not stage, overwrite, or remove them.

- [ ] **Step 2: Confirm no process owns either capture device**

Run:

```bash
fuser \
  /dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0 \
  /dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0
```

Expected: no PIDs are printed. If a PID appears, identify it with `ps -fp <PID>` and stop only the temporary camera/ROS process created for this project.

- [ ] **Step 3: Reconfirm the front-camera match attributes**

Run:

```bash
udevadm info --query=property --name=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0
udevadm info --attribute-walk --name=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0
```

Expected properties:

```text
ID_PATH=pci-0000:00:14.0-usb-0:3:1.0
ID_V4L_CAPABILITIES=:capture:
ATTR{index}=="0"
```

- [ ] **Step 4: Reconfirm the down-camera match attributes**

Run:

```bash
udevadm info --query=property --name=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0
udevadm info --attribute-walk --name=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0
```

Expected properties:

```text
ID_PATH=pci-0000:00:14.0-usb-0:2:1.0
ID_V4L_CAPABILITIES=:capture:
ATTR{index}=="0"
```

- [ ] **Step 5: Confirm the runtime user can access video devices**

Run:

```bash
id -nG flag
```

Expected: the output contains `video`. If it does not, run `sudo usermod -aG video flag` and note that a new login is required before testing as `flag`.

### Task 2: Add the repository-owned rule file

**Files:**
- Create: `/home/flag/Jiangyin2026/src/uav_vision/udev/99-uav-cameras.rules`

- [ ] **Step 1: Create the udev directory**

Run:

```bash
mkdir -p /home/flag/Jiangyin2026/src/uav_vision/udev
```

Expected: the directory exists without changing any existing package files.

- [ ] **Step 2: Create the exact rule file**

Create `/home/flag/Jiangyin2026/src/uav_vision/udev/99-uav-cameras.rules` with:

```udev
# Stable camera roles for the Jiangyin UAV. Keep both cables on their assigned USB ports.
SUBSYSTEM=="video4linux", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:3:1.0", ATTR{index}=="0", SYMLINK+="uav_front_camera", GROUP="video", MODE="0660"
SUBSYSTEM=="video4linux", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:2:1.0", ATTR{index}=="0", SYMLINK+="uav_down_camera", GROUP="video", MODE="0660"
```

- [ ] **Step 3: Verify the repository file exactly**

Run:

```bash
sed -n '1,20p' /home/flag/Jiangyin2026/src/uav_vision/udev/99-uav-cameras.rules
grep -c '^SUBSYSTEM=="video4linux"' /home/flag/Jiangyin2026/src/uav_vision/udev/99-uav-cameras.rules
```

Expected: the two rules match the approved design and the count is `2`.

### Task 3: Install and activate the rules

**Files:**
- Create: `/etc/udev/rules.d/99-uav-cameras.rules`
- Source: `/home/flag/Jiangyin2026/src/uav_vision/udev/99-uav-cameras.rules`

- [ ] **Step 1: Verify the destination does not already exist**

Run:

```bash
sudo test ! -e /etc/udev/rules.d/99-uav-cameras.rules
```

Expected: exit code `0`. If the file exists, compare it with `sudo diff -u` and stop before overwriting it.

- [ ] **Step 2: Install with root ownership and read-only system permissions**

Run:

```bash
sudo install -o root -g root -m 0644 \
  /home/flag/Jiangyin2026/src/uav_vision/udev/99-uav-cameras.rules \
  /etc/udev/rules.d/99-uav-cameras.rules
```

Expected:

```text
-rw-r--r-- 1 root root ... /etc/udev/rules.d/99-uav-cameras.rules
```

- [ ] **Step 3: Reload and trigger video4linux rules**

Run:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
udevadm settle
```

Expected: all commands exit `0`.

- [ ] **Step 4: Check rule evaluation for the front device**

Run:

```bash
FRONT_SYSFS=$(udevadm info --query=path --name=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0)
sudo udevadm test "$FRONT_SYSFS" 2>&1 | grep -E '99-uav-cameras.rules|uav_front_camera'
```

Expected: output references `99-uav-cameras.rules` and `uav_front_camera`.

- [ ] **Step 5: Check rule evaluation for the down device**

Run:

```bash
DOWN_SYSFS=$(udevadm info --query=path --name=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0)
sudo udevadm test "$DOWN_SYSFS" 2>&1 | grep -E '99-uav-cameras.rules|uav_down_camera'
```

Expected: output references `99-uav-cameras.rules` and `uav_down_camera`.

### Task 4: Verify stable names and capture

**Files:**
- Inspect: `/dev/uav_front_camera`
- Inspect: `/dev/uav_down_camera`
- Use: `/home/flag/Jiangyin2026/src/uav_vision/scripts/camera_focus_view.py`

- [ ] **Step 1: Verify both symlinks exist**

Run:

```bash
ls -l /dev/uav_front_camera /dev/uav_down_camera
readlink -f /dev/uav_front_camera
readlink -f /dev/uav_down_camera
```

Expected: each link resolves to a `/dev/videoN` node; the two resolved nodes are different.

- [ ] **Step 2: Verify each link retains the intended physical path**

Run:

```bash
udevadm info --query=property --name=/dev/uav_front_camera | grep -E '^(ID_PATH|ID_V4L_CAPABILITIES)='
udevadm info --query=property --name=/dev/uav_down_camera | grep -E '^(ID_PATH|ID_V4L_CAPABILITIES)='
```

Expected:

```text
front: ID_PATH=pci-0000:00:14.0-usb-0:3:1.0 and :capture:
down:  ID_PATH=pci-0000:00:14.0-usb-0:2:1.0 and :capture:
```

- [ ] **Step 3: Smoke-test one frame from the front stable name**

Run:

```bash
cd /home/flag/Jiangyin2026
python3 -c 'import sys; sys.path.insert(0,"src/uav_vision/scripts"); from camera_focus_view import open_capture,measure_sharpness; c,f=open_capture("/dev/uav_front_camera",1280,720,30); print(f.shape,round(measure_sharpness(f),1)); c.release()'
```

Expected: shape is `(720, 1280, 3)` and sharpness is a non-negative number.

- [ ] **Step 4: Smoke-test one frame from the down stable name**

Run:

```bash
cd /home/flag/Jiangyin2026
python3 -c 'import sys; sys.path.insert(0,"src/uav_vision/scripts"); from camera_focus_view import open_capture,measure_sharpness; c,f=open_capture("/dev/uav_down_camera",1280,720,30); print(f.shape,round(measure_sharpness(f),1)); c.release()'
```

Expected: shape is `(720, 1280, 3)` and sharpness is a non-negative number.

### Task 5: Verify scope and leave the work uncommitted

**Files:**
- Inspect: `/home/flag/Jiangyin2026/src/uav_vision/udev/99-uav-cameras.rules`
- Inspect: `/home/flag/Jiangyin2026/docs/superpowers/specs/2026-07-26-uav-vision-design.md`

- [ ] **Step 1: Confirm only intended new files changed**

Run:

```bash
cd /home/flag/Jiangyin2026
git status --short
sed -n '1,20p' src/uav_vision/udev/99-uav-cameras.rules
git diff -- docs/superpowers/specs/2026-07-26-uav-vision-design.md
```

Expected: this plan adds only the udev rule. The previously updated design document and all unrelated user changes remain untouched during execution.

- [ ] **Step 2: Do not commit**

Per the user's instruction, leave the verified udev rule and documentation uncommitted on `plan_yaw` until the physical competition props are ready.

- [ ] **Step 3: Schedule the two remaining physical acceptance checks**

At the next hardware session:

1. Unplug and reconnect each camera one at a time; confirm both stable links recover and show the correct role.
2. Cold-start the NUC; confirm both stable links recover before any ROS launch starts.
