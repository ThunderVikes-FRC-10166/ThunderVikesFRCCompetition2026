# Guide 07: Autonomous Strategies

## What You'll Learn

This guide teaches you **how to think about autonomous mode** and gives you building blocks (code snippets) to design your own strategies.

Autonomous mode is the first 20 seconds of every FRC match. Your robot drives itself — no human control. What you do in these 20 seconds can decide the match.

This is YOUR chance to be creative. Every team's autonomous mode is different because every team has different strengths, hardware, and match strategies.

---

# Part 1: Understanding Autonomous Mode in MagicBot

## How MagicBot Autonomous Works

MagicBot automatically finds autonomous mode files in the `RobotMain/autonomous/` folder. Each file defines one strategy. The driver picks which strategy to use from the Driver Station before the match starts.

Every autonomous mode class needs:
- `MODE_NAME` — the name shown in the Driver Station
- `DEFAULT` — set to `True` for the strategy you want selected by default
- `on_enable()` — runs once when autonomous starts
- `on_iteration(time_elapsed)` — runs every 20ms (this is your main loop)
- `on_disable()` — runs once when autonomous ends

MagicBot **injects** your components automatically. If you declare a variable with the same name as a component in `robot.py`, MagicBot fills it in:

```python
class MyAuto:
    MODE_NAME = "My Strategy"
    DEFAULT = False

    swerve_drive: SwerveDrive          # MagicBot injects this automatically
    super_scorer: ThunderVikesSuperScorer  # And this too

    def on_enable(self):
        self.timer = wpilib.Timer()
        self.timer.start()

    def on_iteration(self, time_elapsed):
        # Your autonomous logic goes here
        pass
```

**Important**: The variable names must EXACTLY match the names in `robot.py`. Our components are `swerve_drive`, `intake`, `hopper`, `shooter`, and `super_scorer`.

---

# Part 2: Knowing Where You Are

The first big decision in autonomous is: **does your robot know where it is on the field?**

## Option A: Vision (Limelight with AprilTags)

If you completed Guide 06, your robot has a Pose Estimator that blends wheel odometry with AprilTag vision. This gives you accurate, drift-corrected positioning.

**Advantages:**
- Very accurate position — the robot can drive to exact field locations
- Self-correcting — if wheels slip, vision fixes the estimate
- Can align precisely to scoring locations

**Disadvantages:**
- Only works when the camera can see AprilTags
- Slight processing delay (latency)
- Camera must be mounted and configured

**Setting the starting position:**
At the start of autonomous, you should tell the robot where it's starting on the field:

```python
from wpimath.geometry import Pose2d, Rotation2d

def on_enable(self):
    # Tell the robot it's starting at a specific position
    # X=1.5m from the wall, Y=5.5m across, facing 180 degrees
    start_pose = Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180))
    self.swerve_drive.reset_odometry(start_pose)
```

The X and Y values depend on which starting position you're in. More on this in Part 4.

## Option B: Odometry Only (No Camera)

If you don't have a Limelight or it's not working, the robot still tracks its position using wheel encoders and the gyroscope. This works well for the 20-second autonomous period because there hasn't been much time for drift to accumulate.

**Advantages:**
- Always works — no camera needed
- No latency
- Simple to set up

**Disadvantages:**
- Drifts over time (small errors add up)
- If wheels slip (bumping another robot), position gets wrong
- Less precise for long-distance navigation

**Tip:** For simple autonomous routines (drive forward, shoot, drive back), odometry alone is often good enough.

---

# Part 3: Understanding set_drive_command() Speeds

Before we look at snippets, you need to understand how `set_drive_command()` works:

```python
self.swerve_drive.set_drive_command(x_speed, y_speed, rot, field_relative, rate_limit)
```

- **x_speed**: Forward/backward as a fraction of max speed (-1.0 to 1.0)
- **y_speed**: Left/right (strafe) as a fraction of max speed (-1.0 to 1.0)
- **rot**: Rotation as a fraction of max angular speed (-1.0 to 1.0)
- **field_relative**: `True` = "forward" means toward the far field wall (recommended)
- **rate_limit**: `True` = smooth acceleration, `False` = instant response

The key: **these are fractions of `kMaxSpeed` (4.0 m/s), not raw m/s values.** To convert from meters per second to the value you pass in:

```
value = desired_m_per_s / constants.kMaxSpeed
```

For example:
- To drive at **1 m/s forward**: pass `1.0 / 4.0 = 0.25`
- To drive at **2 m/s forward**: pass `2.0 / 4.0 = 0.50`
- To drive at **1 m/s left**: pass y_speed = `1.0 / 4.0 = 0.25`

**This matters for distance planning!** If you drive at 1 m/s for 5 seconds, you travel approximately 5 meters. That's much easier to reason about than "30% speed for 2 seconds."

We'll use a helper to make the code clearer:

```python
def mps(meters_per_second):
    """Convert meters per second to a set_drive_command value."""
    return meters_per_second / constants.kMaxSpeed
```

---

# Part 4: Building Blocks (Code Snippets)

Here are reusable pieces you can combine to build your autonomous strategy. Each one does one thing. Mix and match them.

## Building Block: Timer-Based Driving

The simplest approach — drive for a specific number of seconds:

```python
def on_enable(self):
    self.timer = wpilib.Timer()
    self.timer.start()
    self.state = "DRIVING"

def on_iteration(self, time_elapsed):
    if self.state == "DRIVING":
        # Drive forward at 1 m/s for 3 seconds = ~3 meters forward
        self.swerve_drive.set_drive_command(
            1.0 / constants.kMaxSpeed, 0.0, 0.0, True, False
        )
        if self.timer.get() > 3.0:
            self.state = "DONE"
    elif self.state == "DONE":
        self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, True, False)
```

**When to use:** Simple routines where you just need to move and don't care about exact positioning. Easy to plan: speed (m/s) times time (s) equals approximate distance (m).

## Building Block: Drive to a Position (Requires Vision or Good Odometry)

Drive toward a specific field coordinate:

```python
import math

def drive_toward(self, target_x, target_y, max_speed_mps=2.0):
    """Drive toward a target position. Returns True when close enough."""
    current = self.swerve_drive.get_pose()
    dx = target_x - current.X()
    dy = target_y - current.Y()
    distance = math.hypot(dx, dy)

    if distance < 0.15:  # Within 15cm — close enough
        self.swerve_drive.set_drive_command(0.0, 0.0, 0.0, True, False)
        return True

    # Speed in m/s: go faster when far, slow down near target
    # distance * 1.0 means at 2m away we go 2 m/s, at 0.5m we go 0.5 m/s
    speed_mps = min(distance * 1.0, max_speed_mps)

    # Convert from m/s to the fraction that set_drive_command expects
    x_speed = (dx / distance) * speed_mps / constants.kMaxSpeed
    y_speed = (dy / distance) * speed_mps / constants.kMaxSpeed

    self.swerve_drive.set_drive_command(x_speed, y_speed, 0.0, True, False)
    return False
```

**When to use:** When you want the robot to go to a specific spot on the field. Works best with vision, but works with odometry too for short distances. The `max_speed_mps` parameter lets you control how fast the robot approaches (in meters per second).

## Building Block: Rotate to a Heading

Turn the robot to face a specific direction:

```python
def rotate_toward(self, target_degrees, tolerance=3.0):
    """Rotate to face a specific heading. Returns True when aligned."""
    current_heading = self.swerve_drive.get_heading()
    error = target_degrees - current_heading

    # Wrap to [-180, 180]
    while error > 180:
        error -= 360
    while error < -180:
        error += 360

    if abs(error) < tolerance:
        return True

    rot_speed = error * 0.01  # Proportional control
    rot_speed = max(min(rot_speed, 0.3), -0.3)  # Clamp speed

    self.swerve_drive.set_drive_command(0.0, 0.0, rot_speed, True, False)
    return False
```

**When to use:** When you need the robot to face a specific direction before shooting or driving.

## Building Block: Intake a Ball

Use the super scorer to pick up a ball:

```python
def intake_for_duration(self, duration_seconds):
    """Run the intake for a specified duration. Call every cycle."""
    if self.intake_timer.get() < duration_seconds:
        self.super_scorer.intake_ball()
        return False  # Still intaking
    return True  # Done intaking
```

## Building Block: Shoot a Ball

Fire using the super scorer:

```python
def shoot_sequence(self):
    """Run the shooting sequence. Call every cycle. Returns True when done."""
    self.super_scorer.shoot_ball()
    # The super scorer handles spin-up and feeding automatically
    # Give it about 2 seconds for the flywheels to spin up and fire
    if self.shoot_timer.get() > 2.0:
        return True
    return False
```

## Building Block: Wait

Sometimes you need to pause between actions:

```python
def wait_for(self, seconds):
    """Wait for a duration. Returns True when time has elapsed."""
    return self.step_timer.get() > seconds
```

---

# Part 4: Alliance and Starting Position

The FRC field is symmetric. Blue alliance starts on one side, red on the other. Your autonomous strategy needs to account for this.

## The WPILib Blue Alliance Coordinate System

WPILib uses a coordinate system where:
- **Origin (0, 0)** is at the bottom-right corner of the blue alliance wall
- **X** increases toward the red alliance wall
- **Y** increases to the left (when facing the red wall)

This means the same field position has DIFFERENT coordinates depending on which alliance you're on. But `botpose_wpiblue` from the Limelight always uses the blue alliance origin, regardless of your actual alliance.

## Handling Alliance Color

You can check which alliance you're on and flip coordinates:

```python
import wpilib

def get_alliance_adjusted_x(self, x_blue):
    """Convert a blue-alliance X coordinate to the correct alliance."""
    alliance = wpilib.DriverStation.getAlliance()
    if alliance == wpilib.DriverStation.Alliance.kRed:
        field_length = 16.54  # 2025 field length in meters, check for 2026
        return field_length - x_blue
    return x_blue
```

**Simpler approach:** Write separate autonomous modes for blue and red, or use a selector:

```python
class BlueLeftAuto:
    MODE_NAME = "Blue - Left Start"
    DEFAULT = False
    # ... blue-specific positions

class RedLeftAuto:
    MODE_NAME = "Red - Left Start"
    DEFAULT = False
    # ... red-specific positions (mirrored)
```

This is more code, but much easier to understand and debug than trying to flip coordinates at runtime.

## Common Starting Positions

Most FRC fields have 2-3 common starting positions per alliance. You should plan a strategy for each one:

1. **Left position** — closer to one side of the field
2. **Center position** — in the middle
3. **Right position** — closer to the other side

Each starting position offers different scoring opportunities and paths. Think about:
- What scoring elements are closest?
- Which path avoids traffic from other robots?
- Can you pick up game pieces along the way?

---

# Part 5: Tips

1. **Start simple.** Get the robot driving forward and stopping first. Then add one thing at a time.

2. **Use state machines.** The `self.state = "DOING_SOMETHING"` pattern keeps your code organized. Each state does one thing.

3. **Add timeouts.** Never let a state run forever. If something takes longer than expected, move on.

4. **Test one state at a time.** Comment out everything after the first state, get that working, then add the next.

5. **Log what's happening.** Use `print()` to show your current state and position. This makes debugging much easier.

6. **Always have a simple fallback.** If your complex strategy breaks at competition, you need something basic that works.

---

# Part 6: Creating Your Autonomous File

To add a new autonomous mode:

1. Create a new file in `RobotMain/autonomous/` (e.g., `my_strategy_auto.py`)
2. Define a class with `MODE_NAME` and either `on_enable`/`on_iteration` or MagicBot's `AutonomousStateMachine`
3. Declare any components you need as class-level type annotations
4. MagicBot finds it automatically — no changes to `robot.py` needed

**File naming:** The file name doesn't matter to MagicBot, but use something descriptive like `blue_left_score_auto.py` or `sweep_and_leave_auto.py`.

**Multiple strategies:** You can have as many autonomous mode files as you want. The driver picks which one to use from the Driver Station software before the match starts.

---

# Checklist Before Competition

- [ ] "Just Leave" autonomous works reliably (this is your backup)
- [ ] At least one scoring autonomous is tested
- [ ] Starting positions are correct for both blue and red alliance
- [ ] All autonomous modes have timeouts (no infinite loops)
- [ ] You've tested each autonomous mode at least 5 times
- [ ] You have a plan for which autonomous to use based on field position
- [ ] Your alliance partners know what your robot will do in autonomous

---

# Now It's Your Turn

You have the building blocks. You know how the autonomous system works. Sketch your strategy on paper, figure out the states, and build it. You have 20 seconds — make them count.

Good luck, and have fun with it.
