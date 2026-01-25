
# RobotPy + MagicBot Try/Except Cheat Sheet (FRC 2026)  
### How to handle errors safely in SIM and on the real robot (Beginner-friendly)

Robots are real-time systems. When your code crashes:
- In **SIM**: the program stops and you lose control
- On the **real robot**: the robot can stop responding (and it’s stressful + unsafe)

This cheat sheet teaches how to use:
- `try`
- `except`
- `else`
- `finally`

…so your robot can:
✅ keep running safely  
✅ stop motors if something fails  
✅ show clear debug messages  
✅ avoid “mystery crashes”

---

## 0) The #1 rule: safety first

If something goes wrong, do one of these:
1) **Stop all motion** (`swerve.drive(0,0,0)`)
2) **Skip the bad update** and keep running
3) **Disable a feature** (example: turn off vision alignment if Limelight is missing)

Never keep sending random motor commands if sensors are broken.

---

## 1) The 4 pieces of try/except (what they mean)

### Basic form
```python
try:
    # risky code
except SomeError:
    # what to do if it fails
````

### Best-practice form

```python
try:
    # risky code
except SomeError as e:
    # handle it safely
else:
    # runs only if no exception happened
finally:
    # ALWAYS runs, even if exception happened
```

**`finally` is great for safety** because it always runs.

---

## 2) The BEST places to use try/except in RobotPy/MagicBot

### ✅ Place A: in periodic loops (teleop/autonomous iteration)

Bad things happen here:

* controller disconnects
* NetworkTables value missing
* math errors (divide by zero)
* `NoneType` (object not injected / not created)

**Goal:** don’t crash the whole robot loop.

**Pattern:**

* wrap only the “risky” part
* if it fails: stop robot OR skip

---

### ✅ Place B: vision / NetworkTables reads (Limelight)

Common issue:

* Limelight not connected
* wrong table name
* key missing (`tx`, `tv`, etc.)

**Goal:** if vision breaks, robot should still drive normally.

---

### ✅ Place C: initialization of optional hardware

In SIM:

* navX may not exist
* Limelight may not exist
* real motor controllers don’t exist

**Goal:** allow SIM to run even if real hardware isn’t present.

---

### ✅ Place D: autonomous mode code

Auto should fail safely:

* stop motion
* exit the routine (switch to DONE)

---

## 3) What NOT to do with try/except

### ❌ Don’t hide every error

This is bad:

```python
try:
    do_everything()
except:
    pass
```

Why?

* it hides bugs
* you won’t know what broke

✅ Always print/log something useful.

---

### ❌ Don’t wrap your entire robot loop unless you stop safely

If you must catch a broad error, you should **stop the robot**.

---

## 4) The “3 levels” of exceptions to catch

### Level 1 (most common for beginners): `AttributeError`

This happens when:

* something is `None`
* you typed the name wrong
* injection didn’t happen

Example error:

```
AttributeError: 'NoneType' object has no attribute 'drive'
```

✅ Great for SIM when something isn’t wired yet.

---

### Level 2: `KeyError` / `TypeError` / `ValueError`

Common causes:

* NetworkTables key missing → `KeyError` (sometimes)
* math conversion gets wrong type → `TypeError`
* converting invalid data → `ValueError`

---

### Level 3 (last resort): `Exception`

Catches everything, including unknown problems.

✅ Use only when you also do safety behavior (STOP robot).

---

## 5) SIM-safe driving: try/except in teleopPeriodic

In `robot.py`, a safe pattern is:

```python
def teleopPeriodic(self):
    try:
        x = -self.driver.getLeftY()
        y = -self.driver.getLeftX()
        rot = -self.driver.getRightX()

        # ...deadband, scaling...

        self.swerve.drive(vx, vy, omega)

    except AttributeError as e:
        # Probably injection issue: swerve/driver missing
        wpilib.reportError(f"Teleop AttributeError: {e}", False)

    except Exception as e:
        # Something unexpected - stop robot for safety
        wpilib.reportError(f"Teleop Error: {e}", True)
        self.swerve.drive(0.0, 0.0, 0.0)
```

### Why this is good

* If a joystick method fails, you won’t crash the sim
* If something unexpected happens, it stops driving (safe)

---

## 6) SIM-safe autonomous: stop safely if something fails

In your auto file:

```python
def on_iteration(self, time_elapsed: float):
    try:
        # your state machine logic
        self.robot.swerve.drive(1.0, 0.0, 0.0)

    except AttributeError as e:
        # robot or swerve missing
        wpilib.reportError(f"Auto missing robot/swerve: {e}", False)
        self.state = "DONE"

    except Exception as e:
        wpilib.reportError(f"Auto unexpected error: {e}", True)
        self.state = "DONE"
        self.robot.swerve.drive(0.0, 0.0, 0.0)
```

---

## 7) Vision (Limelight) try/except pattern (recommended)

Vision should be optional.
If it fails, disable it and keep driving.

```python
def get_limelight_tx(self):
    try:
        table = self.limelight_table
        tv = table.getNumber("tv", 0)
        if tv < 1:
            return None
        return table.getNumber("tx", 0.0)
    except Exception as e:
        wpilib.reportError(f"Limelight read failed: {e}", False)
        return None
```

Then in driving code:

```python
tx = self.get_limelight_tx()
if tx is not None:
    # do auto-aim math
    pass
else:
    # normal driving
    pass
```

✅ If limelight breaks, your robot still drives.

---

## 8) Using `finally` for motor safety

`finally` ALWAYS runs.

This is very useful when:

* you might crash midway
* you always want to stop motors or release resources

Example:

```python
try:
    # some risky autonomous step
    self.robot.swerve.drive(1.0, 0.0, 0.0)
    do_some_math()
except Exception as e:
    wpilib.reportError(f"Auto step failed: {e}", True)
finally:
    # guarantee we won't keep moving forever
    if self.robot is not None:
        self.robot.swerve.drive(0.0, 0.0, 0.0)
```

✅ Good for “if anything goes wrong, stop.”

---

## 9) A recommended “robot-wide” emergency stop helper

Make a helper method (in robot.py or drivetrain):

```python
def safe_stop(self):
    try:
        self.swerve.drive(0.0, 0.0, 0.0)
    except Exception:
        pass
```

Then in any except/finally:

```python
self.safe_stop()
```

---

## 10) Debug messages: best practice

Use WPILib’s reporting so it shows in console + Driver Station logs:

```python
wpilib.reportWarning("Something odd happened")
wpilib.reportError("Something broke", True)
```

**Tip:** Include the exception message:

```python
except Exception as e:
    wpilib.reportError(f"Error: {e}", True)
```

---

## 11) “Common errors” and what to do

### Error: Controller not connected

Symptom:

* joystick axes missing
  Fix:
* catch it, treat joystick input as 0
* don’t crash

### Error: `NoneType` from injection

Symptom:

* `self.swerve` is None somewhere
  Fix:
* stop driving
* show a warning
* check injection names

### Error: Limelight missing

Symptom:

* NetworkTables keys are missing
  Fix:
* return None from vision method
* disable auto-aim

---

## 12) The golden pattern students can copy (short)

**Teleop/Auto loops:**

```python
try:
    # read sensors/input
    # compute command
    # command robot
except AttributeError as e:
    # missing hardware / injection
    wpilib.reportError(f"Missing object: {e}", False)
    # stop or skip
except Exception as e:
    # unexpected
    wpilib.reportError(f"Unexpected error: {e}", True)
    # STOP for safety
finally:
    # optional safety guarantee
    pass
```

---

## Final advice

* Use try/except around **small risky sections**, not giant blocks.
* Always log something useful.
* When unsure, STOP the robot.

In SIM, this makes your work sessions smoother.
On a real robot, this keeps people safe.
