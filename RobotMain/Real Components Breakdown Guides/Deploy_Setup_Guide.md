# Deploying RobotPy + MagicBot to the roboRIO (FRC 2026)
## Student Setup Guide (Switching from Java in 2025 to Python in 2026)

This is the step-by-step guide for:
- installing RobotPy on your laptop
- making sure the roboRIO gets Python + required packages
- updating `pyproject.toml` correctly for our swerve code (REV + navX + MagicBot)
- deploying from the terminal (PyCharm friendly)

---

# 0) Big picture: what goes where?

There are TWO computers involved:

### 💻 Your laptop
You install:
- Python (on Windows/macOS)
- RobotPy tools
- Your code (edited in PyCharm)

### 🤖 The roboRIO
RobotPy deploy tools will install:
- the correct Python runtime for roboRIO
- RobotPy (WPILib bindings)
- vendor libraries (REV, navX, etc.)
- your project code

**Important:** You do NOT manually “install Python” on the roboRIO.  
`robotpy sync` + `robotpy deploy` handle it.

---

# 1) Install Python + RobotPy on your laptop

## Step 1.1 — Install Python
Install Python from python.org.

On Windows, during install, check:
- ✅ “Add Python to PATH”

## Step 1.2 — Install/upgrade RobotPy tools
Open Command Prompt or PowerShell and run:

```bash
py -3 -m pip install --upgrade robotpy
```

---

# 2) PyCharm setup (so you don’t fight the interpreter)

## Step 2.1 — Open the project folder
Open the folder that contains your robot project (the one with `RobotMain/`).

## Step 2.2 — Choose the Python interpreter
PyCharm:
- File → Settings → Project → Python Interpreter
- Select the Python you installed in step 1

## Step 2.3 — Quick import test
In PyCharm Python Console:

```python
import wpilib
import magicbot
```

If both imports work, your laptop is ready.

---

# 3) Fixing your `pyproject.toml` (THIS IS THE IMPORTANT PART)

You showed this file:

```toml
[tool.robotpy]
robotpy_version = "2026.2.1"

components = [
    "all",
    "apriltag",
    "commands2",
    "cscore",
    "romi",
    "sim",
    "xrp",
    "rev",
]

requires = [
    "wpilib",
]
```

### What needs to change (summary)
1) ✅ Keep `robotpy_version = "2026.2.1"`
2) ✅ `components` are RobotPy *extras* like `all/sim/commands2/...`
3) ❌ `"rev"` is NOT a RobotPy extra (it does not appear in RobotPy extras list)
4) ✅ REV and navX should be installed via `requires` using vendor package names:
   - `robotpy-rev`
   - `robotpy-navx`
5) ❌ You do NOT need `"wpilib"` in `requires` (RobotPy installs WPILib as part of `robotpy_version`)

---

## Step 3.1 — Correct `pyproject.toml` for our swerve robot

Replace your file with this:

```toml
#
# RobotPy configuration (FRC 2026)
# This controls what gets installed on the roboRIO when we deploy
#

[tool.robotpy]

# RobotPy version for the season (you already have this right)
robotpy_version = "2026.2.1"

# Optional RobotPy extras (these are real extras provided by the robotpy meta package)
# Pick what you actually use. "all" is fine for beginners.
components = [
    "all",
    # OR if you prefer explicit instead of "all", you could do:
    # "sim",
    # "commands2",
    # "cscore",
    # "apriltag",
    # "romi",
    # "xrp",
]

# Third-party / vendor packages (PyPI package names)
requires = [
    "robotpy-wpilib-utilities",  # MagicBot framework
    "robotpy-rev",               # REV Spark MAX support
    "robotpy-navx",              # navX2 gyro support
]
```

### Why this is correct
- `components` must match RobotPy “extras” names like: `apriltag`, `commands2`, `cscore`, `romi`, `sim`, `xrp`, `all`
- REV and navX are separate vendor packages installed via `requires`
- MagicBot comes from `robotpy-wpilib-utilities`

---

# 4) Run `robotpy sync` (do this whenever pyproject.toml changes)

From the folder that contains `pyproject.toml`, run:

```bash
py -3 -m robotpy sync
```

What sync does:
- installs the right Python packages locally
- downloads roboRIO-compatible wheels for deployment
- prepares your environment so deploy works

You need internet for this step.

---

# 5) Connect to the robot

You can deploy over:
- USB to roboRIO
- Ethernet
- robot radio Wi‑Fi

Beginner tip: USB is usually the most reliable when you’re starting.

---

# 6) Deploy to the roboRIO (the command you’ll use most)

Before deploying, make sure:
- the file you want to run is named `robot.py`
- `pyproject.toml` is in the same folder as `robot.py`

Then deploy with netconsole output:

```bash
py -3 -m robotpy deploy --nc
```

What deploy does:
- installs Python on roboRIO (if missing)
- installs packages from `pyproject.toml`
- copies your code
- starts the robot program

If it works, you should see:

```
SUCCESS: Deploy was successful!
```

---

# 7) Switching between SIM and REAL robot files

You are using this workflow:
- `robot_sim.py` = simulation version
- `robot_real.py` = real robot version

Before deploying, rename the one you want to run into `robot.py`.

Example (REAL):
- rename `robot_real.py` → `robot.py`
- deploy

Example (SIM):
- rename `robot_sim.py` → `robot.py`
- run sim / tests

---

# 8) Common errors (and fast fixes)

## “ModuleNotFoundError: rev” or “navx” or “magicbot”
Fix:
1) Verify `requires` contains:
   - `robotpy-rev`
   - `robotpy-navx`
   - `robotpy-wpilib-utilities`
2) Run:
   ```bash
   py -3 -m robotpy sync
   ```
3) Deploy again:
   ```bash
   py -3 -m robotpy deploy --nc
   ```

## Deploy cannot find roboRIO
Fix checklist:
- Driver Station sees robot?
- correct team number / IP config?
- try USB

## No console output
Fix:
- deploy with `--nc`

## Deploy fails due to tests (temporary workaround)
```bash
py -3 -m robotpy deploy --skip-tests
```

---

# 9) Student “deploy checklist” (print this)

Before deploy:
- [ ] I renamed the correct file to `robot.py`
- [ ] `pyproject.toml` is next to `robot.py`
- [ ] I ran `robotpy sync` after changes

Deploy:
- [ ] `py -3 -m robotpy deploy --nc`

After deploy:
- [ ] I see “SUCCESS: Deploy was successful!”
- [ ] Driver Station can enable without code crashing
- [ ] Console output shows our prints / errors

---

If students follow this guide, the robot will have:
- RobotPy WPILib
- MagicBot
- REV Spark MAX support
- navX2 support

…which is exactly what our swerve module + drivetrain guides require.
