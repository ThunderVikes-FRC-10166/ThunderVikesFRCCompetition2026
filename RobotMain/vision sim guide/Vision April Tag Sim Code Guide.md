
# AprilTag Vision in SIM (RobotPy 2026 + MagicBot)  
### Goal
Make the simulator act like a **Limelight camera** that can see AprilTags on the 2026 field.

By the end of this:
- The **simulator** will detect AprilTags based on real 2026 field locations  
- Your robot code will read **Limelight-style data** (`tv`, `tid`, `tx`, pose info)  
- The console will print when a tag is visible  

Later, when you use a real Limelight, your robot code will stay almost the same.

---

## 0) How vision works on a real robot

A Limelight does **not** directly control the robot.  
Instead, it:
1. Detects AprilTags  
2. Publishes data to **NetworkTables**  
3. Your robot reads that data and decides what to do

We are going to copy that exact pattern in simulation.

---

## 1) What Limelight normally sends

These are the most common values teams use:

| Key | Meaning |
|-----|--------|
| `tv` | 1 = tag visible, 0 = none |
| `tid` | AprilTag ID number |
| `tx` | Horizontal angle offset (degrees) |
| `ty` | Vertical angle offset (degrees) |
| `botpose_wpiblue` | Robot pose estimate from tags |

We will simulate these exact keys.

---

## 2) Files we will modify

| File | Why |
|------|-----|
| `physics.py` | Simulates camera and publishes fake Limelight data |
| `robot.py` | Reads the vision data and prints it |

---

## 3) Add the official 2026 AprilTag field layout

We want tag positions to match the **real 2026 field**.

### Step 1 — Add the layout JSON
Download the WPILib AprilTag layout JSON for 2026 and put it in:

```

RobotMain/resources/2026-rebuilt-welded.json

````

This file contains:
- Tag ID numbers
- Exact field coordinates

---

## 4) Update `physics.py` to simulate a Limelight

### 4.1 Add these imports at the top

```python
from ntcore import NetworkTableInstance
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d
````

---

### 4.2 In `PhysicsEngine.__init__`, add:

```python
# Fake Limelight NetworkTables table
self.ll_table = NetworkTableInstance.getDefault().getTable("limelight")

# Load official AprilTag layout
self.tag_layout = AprilTagFieldLayout("resources/2026-rebuilt-welded.json")

# Camera position (front center of robot)
self.cam_forward_m = 0.30
self.cam_left_m = 0.00

# Visibility settings
self.max_tag_distance_m = 5.0
self.fov_half_angle_deg = 30.0
```

---

### 4.3 Add this helper function inside `PhysicsEngine`

```python
def _pick_visible_tag(self, robot_pose_2d):
    heading = robot_pose_2d.rotation()
    cosA = heading.cos()
    sinA = heading.sin()

    cam_dx = self.cam_forward_m * cosA - self.cam_left_m * sinA
    cam_dy = self.cam_forward_m * sinA + self.cam_left_m * cosA

    cam_x = robot_pose_2d.X() + cam_dx
    cam_y = robot_pose_2d.Y() + cam_dy

    best = None

    for tag in self.tag_layout.getTags():
        tag_id = tag.ID
        tag_pose: Pose3d = tag.pose

        dx = tag_pose.X() - cam_x
        dy = tag_pose.Y() - cam_y
        dist = math.hypot(dx, dy)

        if dist > self.max_tag_distance_m:
            continue

        rel_x = dx * cosA + dy * sinA
        rel_y = -dx * sinA + dy * cosA

        if rel_x <= 0.01:
            continue

        tx_deg = math.degrees(math.atan2(rel_y, rel_x))

        if abs(tx_deg) > self.fov_half_angle_deg:
            continue

        if best is None or dist < best[0]:
            best = (dist, tag_id, tx_deg)

    if best is None:
        return None

    dist, tag_id, tx_deg = best
    return (tag_id, tx_deg, dist)
```

---

### 4.4 At the end of `update_sim`, add:

```python
seen = self._pick_visible_tag(self.pose)

if seen is None:
    self.ll_table.putNumber("tv", 0)
    self.ll_table.putNumber("tid", -1)
    self.ll_table.putNumber("tx", 0.0)
    self.ll_table.putNumber("ty", 0.0)
else:
    tag_id, tx_deg, dist_m = seen
    self.ll_table.putNumber("tv", 1)
    self.ll_table.putNumber("tid", tag_id)
    self.ll_table.putNumber("tx", tx_deg)
    self.ll_table.putNumber("ty", 0.0)

    yaw_deg = math.degrees(self.yaw_rad)
    botpose = [self.pose.X(), self.pose.Y(), 0.0, 0.0, 0.0, yaw_deg, 11.0]
    self.ll_table.putNumberArray("botpose_wpiblue", botpose)

    if int(now * 5) != int((now - tm_diff) * 5):
        print(f"[FakeLimelight] tv=1 tid={tag_id} tx={tx_deg:.1f} dist={dist_m:.2f}m")
```

---

## 5) Read vision data in `robot.py`

### 5.1 Add import at the top

```python
from ntcore import NetworkTableInstance
```

---

### 5.2 In `createObjects`, add:

```python
self.ll = NetworkTableInstance.getDefault().getTable("limelight")
```

---

### 5.3 In `teleopPeriodic`, add:

```python
tv = self.ll.getNumber("tv", 0)
if tv >= 1:
    tid = int(self.ll.getNumber("tid", -1))
    tx = self.ll.getNumber("tx", 0.0)
    botpose = self.ll.getNumberArray("botpose_wpiblue", [])
    print(f"[Robot] sees tag tid={tid} tx={tx:.1f} botpose={botpose}")
```

---

## 6) How to test in the simulator

1. Run:

   ```
   robotpy sim
   ```
2. Drive the robot around in Teleop
3. Watch the console:

   * `[FakeLimelight]` lines show simulated camera detection
   * `[Robot] sees tag ...` shows robot reading the vision data

---

## 7) When you switch to a real Limelight later

Remove the fake vision code from `physics.py`.
The real Limelight will publish the same keys:

* `tv`
* `tid`
* `tx`
* pose arrays

Your `robot.py` code that reads the table can stay the same.

---

## 8) What students should understand

* The camera publishes data
* The robot reads that data
* Vision does not move the robot automatically
* Your code decides how to use the information

Once this works, you can build:

* Auto-align to tag (rotate until `tx ≈ 0`)
* Drive to field positions using AprilTag pose

