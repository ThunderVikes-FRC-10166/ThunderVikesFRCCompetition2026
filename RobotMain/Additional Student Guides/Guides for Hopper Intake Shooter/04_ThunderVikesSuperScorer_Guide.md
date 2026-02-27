# ThunderVikes Super Scorer - Student Guide

This guide walks you through building the **ThunderVikesSuperScorer** component, which is a "super component" that coordinates the intake, hopper, and shooter into one unified scoring system.

---

## What is a Super Component?

Think of a factory:
- The **Intake** is the receiving dock (brings materials in)
- The **Hopper** is the conveyor belt (moves materials through the factory)
- The **Shooter** is the shipping dock (sends products out)
- The **Super Scorer** is the **factory manager** (coordinates everything)

Without the Super Scorer, `robot.py` would have to manually tell each component what to do every cycle. That would be messy and error-prone. Instead, the Super Scorer handles all the coordination internally. Robot.py just says "intake a ball" or "shoot a ball" and the Super Scorer figures out the rest.

---

## What is a State Machine?

A state machine is just a fancy way of saying **"I'm in one mode at a time, and I follow rules about when to switch modes."**

Think of a traffic light:
- It can be **RED**, **YELLOW**, or **GREEN**
- It follows rules: GREEN -> YELLOW -> RED -> GREEN
- It's never two colors at once

Our Super Scorer has 4 states:

| State | What's happening |
|-------|-----------------|
| **IDLE** | Not doing anything, waiting for commands |
| **INTAKING** | Opening the intake arm, running roller and hopper to collect a ball |
| **LOADED** | A ball is in the hopper, ready to shoot |
| **SHOOTING** | Spinning up flywheels and feeding the ball to shoot it |

Here's how the states flow:

```
         intake button pressed
IDLE ─────────────────────────────> INTAKING
  ^                                    │
  │                                    │ intake button released
  │                                    v
  │     shoot button pressed        LOADED
  │ <───────────────────────────────   │
  │                                    │ shoot button pressed
  │                                    v
  └──── shoot button released ──── SHOOTING
```

The **stop** button can jump back to IDLE from any state (emergency stop).

---

## What is MagicBot Injection?

MagicBot has a feature called **dependency injection**. Here's how it works:

1. In `robot.py`, you declare components like `intake = Intake()` and `hopper = Hopper()`
2. In the Super Scorer, you declare type hints like `intake: Intake` and `hopper: Hopper`
3. MagicBot **automatically connects them** - it sees that the Super Scorer needs an `Intake`, finds the one declared in `robot.py`, and provides it

The variable names in the Super Scorer **must match** the variable names in `robot.py`. If `robot.py` says `intake = Intake()`, then the Super Scorer must say `intake: Intake` (same name: `intake`).

This is powerful because:
- The Super Scorer doesn't create the intake/hopper/shooter itself
- MagicBot wires everything together automatically
- If you rename a component in `robot.py`, you must rename it in the Super Scorer too

---

## Step 1: Create the File

Create a new file at:

```
RobotMain/components/thunder_vikes_super_scorer.py
```

---

## Step 2: Imports

```python
from magicbot import will_reset_to
from components.intake import Intake
from components.hopper import Hopper
from components.shooter import Shooter
```

What each import does:
- `will_reset_to` - MagicBot's safety feature that automatically resets variables each cycle
- `Intake`, `Hopper`, `Shooter` - the three sub-components we're coordinating

We don't import `rev` or `constants` here because this component doesn't directly control any hardware. It only talks to the intake, hopper, and shooter, and those components handle the hardware.

---

## Step 3: Class Definition and Injection

```python
class ThunderVikesSuperScorer:

    intake: Intake
    hopper: Hopper
    shooter: Shooter
```

These three lines are **MagicBot injection declarations**. We're saying "this component needs an Intake, a Hopper, and a Shooter." MagicBot will automatically provide them from `robot.py`.

Notice there are no `= None` or `= Intake()` assignments. Just the type hints. MagicBot handles the rest.

---

## Step 4: State Constants

```python
    IDLE = "idle"
    INTAKING = "intaking"
    LOADED = "loaded"
    SHOOTING = "shooting"
```

These are our state names. We use string constants so:
- We can't accidentally misspell a state name (the editor would catch `self.IDLE` vs `self.IDEL`)
- We can print the current state for debugging and it's human-readable
- If we need to change a state name, we change it in one place

---

## Step 5: Will-Reset-To Variables

```python
    _want_intake = will_reset_to(False)
    _want_shoot = will_reset_to(False)
    _want_stop = will_reset_to(False)
```

These are the "requests" from `robot.py`. Each cycle (every 20ms):
1. `robot.py` calls methods like `intake_ball()` which set these to `True`
2. `execute()` reads them and acts accordingly
3. After `execute()`, MagicBot **automatically resets them all to `False`**

This is a safety feature! If `robot.py` crashes or the operator lets go of a button, these reset to `False` and the Super Scorer knows to stop what it was doing.

The underscore `_` prefix is a Python convention meaning "this is private - only this class should use it directly."

---

## Step 6: setup()

```python
    def setup(self) -> None:
        self.state = self.IDLE
```

`setup()` is called once when the robot starts. We initialize the state machine to `IDLE` (not doing anything).

This is simple because the Super Scorer doesn't have any hardware to configure. The intake, hopper, and shooter handle their own setup.

---

## Step 7: Command Methods (Called from robot.py)

These are the methods that `robot.py` calls when the operator presses buttons:

```python
    def intake_ball(self) -> None:
        self._want_intake = True

    def shoot_ball(self) -> None:
        self._want_shoot = True

    def stop_all(self) -> None:
        self._want_stop = True
```

Notice how simple these are! They just set a flag. The actual logic happens in `execute()`.

Why not do the logic here? Because MagicBot's pattern is:
1. **Command methods** just record what was requested
2. **execute()** does the actual work

This keeps the timing consistent - everything happens in `execute()`, which MagicBot calls at a predictable rate (every 20ms).

---

## Step 8: get_state() Helper

```python
    def get_state(self) -> str:
        return self.state
```

A simple helper that returns the current state. Useful for debugging or displaying on the dashboard.

---

## Step 9: execute() - The State Machine Brain

This is the most important method. It runs every 20ms and contains all the state machine logic:

```python
    def execute(self) -> None:
```

### Part 1: Emergency Stop Check

```python
        if self._want_stop:
            self.state = self.IDLE
            self.intake.stop()
            self.hopper.stop()
            self.shooter.stop()
            return
```

The emergency stop takes priority over everything. If the operator presses the stop button:
1. Jump back to IDLE state
2. Tell all three sub-components to stop immediately
3. `return` - skip the rest of the logic this cycle

### Part 2: IDLE State Logic

```python
        if self.state == self.IDLE:
            if self._want_intake:
                self.state = self.INTAKING
            elif self._want_shoot:
                self.state = self.SHOOTING
```

When idle, we're just waiting for a command:
- If the operator presses intake -> switch to INTAKING
- If the operator presses shoot -> switch to SHOOTING
- If neither is pressed -> stay in IDLE (do nothing)

### Part 3: INTAKING State Logic

```python
        if self.state == self.INTAKING:
            self.intake.open_arm()
            self.intake.run_roller()
            self.hopper.feed_from_intake()

            if not self._want_intake:
                self.state = self.LOADED
```

While intaking:
1. Keep the intake arm open
2. Keep the roller spinning to grab balls
3. Run the hopper to pull balls from the intake

When the operator releases the intake button (`not self._want_intake`):
1. Transition to LOADED state (we assume we got a ball)

Notice we use `if` (not `elif`) for the INTAKING check. This means if we just transitioned from IDLE to INTAKING on this same cycle, we'll start intaking immediately without waiting for the next cycle.

### Part 4: LOADED State Logic

```python
        elif self.state == self.LOADED:
            if not self.intake.is_closed():
                self.intake.close_arm()

            if self._want_shoot:
                self.state = self.SHOOTING
            elif self._want_intake:
                self.state = self.INTAKING
```

When loaded (ball is ready):
- **Close the arm** — Since MagicBot's `will_reset_to` resets every cycle, we need to keep calling `close_arm()` every cycle until the arm is fully closed. We check `is_closed()` first so we stop calling it once the limit switch says "I'm closed."
- If the operator presses shoot -> switch to SHOOTING
- If the operator presses intake -> go back to INTAKING (maybe they want another ball)
- Otherwise -> just sit there holding the ball, continuing to close the arm if needed

### Part 5: SHOOTING State Logic

```python
        elif self.state == self.SHOOTING:
            self.shooter.spin_up()
            self.hopper.feed_to_shooter()

            if self.shooter.is_at_speed():
                self.shooter.feed()

            if not self._want_shoot:
                self.state = self.IDLE
```

While shooting:
1. Tell the flywheels to spin up
2. Run the hopper to push balls toward the shooter
3. **Only feed the ball into the flywheels once they're up to speed** - this is important! If we feed too early, the ball won't shoot far enough
4. When the operator releases the shoot button, go back to IDLE

The `is_at_speed()` check is what makes the shooter reliable. The flywheels need time to reach full speed before we push a ball into them.

---

## The Complete File

Here is the full `thunder_vikes_super_scorer.py` file with all sections together:

```python
from magicbot import will_reset_to
from components.intake import Intake
from components.hopper import Hopper
from components.shooter import Shooter


class ThunderVikesSuperScorer:

    intake: Intake
    hopper: Hopper
    shooter: Shooter

    IDLE = "idle"
    INTAKING = "intaking"
    LOADED = "loaded"
    SHOOTING = "shooting"

    _want_intake = will_reset_to(False)
    _want_shoot = will_reset_to(False)
    _want_stop = will_reset_to(False)

    def setup(self) -> None:
        self.state = self.IDLE

    def intake_ball(self) -> None:
        self._want_intake = True

    def shoot_ball(self) -> None:
        self._want_shoot = True

    def stop_all(self) -> None:
        self._want_stop = True

    def get_state(self) -> str:
        return self.state

    def execute(self) -> None:
        if self._want_stop:
            self.state = self.IDLE
            self.intake.stop()
            self.hopper.stop()
            self.shooter.stop()
            return

        if self.state == self.IDLE:
            if self._want_intake:
                self.state = self.INTAKING
            elif self._want_shoot:
                self.state = self.SHOOTING

        if self.state == self.INTAKING:
            self.intake.open_arm()
            self.intake.run_roller()
            self.hopper.feed_from_intake()

            if not self._want_intake:
                self.state = self.LOADED

        elif self.state == self.LOADED:
            if not self.intake.is_closed():
                self.intake.close_arm()

            if self._want_shoot:
                self.state = self.SHOOTING
            elif self._want_intake:
                self.state = self.INTAKING

        elif self.state == self.SHOOTING:
            self.shooter.spin_up()
            self.hopper.feed_to_shooter()

            if self.shooter.is_at_speed():
                self.shooter.feed()

            if not self._want_shoot:
                self.state = self.IDLE
```

---

## How It All Connects

Here's the big picture of how the Super Scorer fits into the robot:

```
  Xbox Controller (Operator)
         │
         v
    robot.py (teleopPeriodic)
    ├── A button held → super_scorer.intake_ball()
    ├── B button held → super_scorer.shoot_ball()
    └── X button held → super_scorer.stop_all()
         │
         v
    ThunderVikesSuperScorer (execute)
    ├── State machine decides what to do
    ├── Calls intake.open_arm(), intake.run_roller(), etc.
    ├── Calls hopper.feed_from_intake(), hopper.feed_to_shooter()
    └── Calls shooter.spin_up(), shooter.feed()
         │
         v
    Individual Components (execute)
    ├── Intake: controls arm and roller motors
    ├── Hopper: controls 3 conveyor motors
    └── Shooter: controls feeder and flywheel motors
         │
         v
    Physical Motors (SparkMax controllers)
```

MagicBot calls `execute()` on every component every 20ms, in the right order. Components that are depended upon (intake, hopper, shooter) execute **after** the Super Scorer, so the commands from the Super Scorer take effect in the same cycle.

---

## Key Concepts to Remember

1. **Super components don't touch hardware directly** - they only call methods on other components
2. **State machines keep things organized** - one state at a time, clear transition rules
3. **will_reset_to is a safety net** - if nothing calls a method, the request resets to False
4. **MagicBot injection wires it all together** - just declare the type hints, MagicBot does the rest
5. **Command methods just set flags** - the real work happens in execute()
6. **Emergency stop overrides everything** - always check it first in execute()

---

## Testing Your Understanding

Before moving on, make sure you can answer these questions:

1. What happens if the operator presses the intake button and then immediately lets go? (Answer: the Super Scorer transitions IDLE -> INTAKING, then on the next cycle where `_want_intake` is False, it goes to LOADED. In the LOADED state, it keeps calling `close_arm()` every cycle until the limit switch says the arm is fully closed.)

2. Why do we check `is_at_speed()` before calling `feed()`? (Answer: the flywheels need time to spin up. Feeding the ball before they're ready means a weak shot)

3. What happens if `robot.py` crashes mid-cycle? (Answer: `will_reset_to` resets all flags to False, so the Super Scorer stops requesting actions from sub-components. The sub-components also have `will_reset_to` on their own flags, so motors stop too)

4. Why does the INTAKING state use `if` instead of `elif`? (Answer: so that when we transition from IDLE to INTAKING in the same cycle, we immediately start intaking without waiting for the next execute() call)
