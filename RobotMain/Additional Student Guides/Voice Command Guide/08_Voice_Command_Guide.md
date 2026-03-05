# Guide 08: Voice Command System

## What Are We Building?

A voice command system that lets you **speak commands** to your robot! You say things like "Viking forward three" and the robot drives forward 3 meters. This is an optional bonus feature — it does not replace the normal joystick controls.

### How It Works (Big Picture)

```
Your voice → Microphone → Laptop script → NetworkTables → Robot code → Wheels move
```

There are **two separate pieces**:

1. **Voice Commander** (`voice_command/voice_commander.py`) — Runs on the **driver station laptop**. Listens to your microphone, recognizes what you said, and sends the command to the robot over NetworkTables.

2. **Voice Receiver** (`components/voice_receiver.py`) — Runs on the **robot**. Reads commands from NetworkTables so `robot.py` can act on them.

The robot only accepts voice commands when the driver is **holding the left bumper** on their Xbox controller. This prevents accidental commands from crowd noise.

---

## Is This Legal in FRC?

**Yes.** Vosk runs 100% offline (no internet needed), software on the driver station laptop is allowed, and NetworkTables is the standard FRC communication method. We recommend submitting a Q&A to FIRST before your first competition to get written confirmation.

---

## Part 1: Setup

### Download the Vosk Speech Model

Vosk is an **offline speech recognition library** — unlike Siri or Google, it runs entirely on your laptop with no internet needed.

1. Go to: https://alphacephei.com/vosk/models
2. Download: **vosk-model-small-en-us-0.15** (about 50 MB)
3. Unzip it
4. Place it inside `voice_command/model/` so it looks like:

```
voice_command/
    voice_commander.py
    model/
        am/
        conf/
        graph/
        ivector/
```

If you end up with an extra subfolder (like `model/vosk-model-small-en-us-0.15/am/...`), that's fine — the script detects this automatically.

### Install Python Packages

On your **driver station laptop**, run:

```bash
pip install vosk sounddevice inputs pyntcore
```

- `vosk` — Speech recognition engine
- `sounddevice` — Reads audio from your microphone
- `inputs` — Reads gamepad buttons for push-to-talk
- `pyntcore` — The same NetworkTables library the robot uses. Sends commands to the robot (optional for offline testing)

---

## Part 2: Constants

Add these to your `constants.py` (both copies must match):

```python
kVoiceTeamNumber = 10166
kVoiceNTTable = f"/voice/{kVoiceTeamNumber}"
kVoiceDriveSpeed = 1.0
kVoiceTurnSpeed = math.pi / 4
```

| Constant | What It Does |
|---|---|
| `kVoiceTeamNumber` | Your team number — keeps your commands separate from other teams |
| `kVoiceNTTable` | The NetworkTables path where commands are published |
| `kVoiceDriveSpeed` | Movement speed in meters per second (1.0 = walking speed) |
| `kVoiceTurnSpeed` | Turn speed in radians per second (pi/4 = 45 degrees/sec) |

---

## Part 3: Building the Voice Commander Script

Create the file `voice_command/voice_commander.py`. We'll build it up step by step so you understand what each piece does. Add each section in order.

### Step 1: The Docstring and Imports

Start by adding the file description and importing the libraries we need:

```python
"""
voice_commander.py — Voice Command System for Team 10166 ThunderVikes
=====================================================================

This script runs on the DRIVER STATION LAPTOP (not the robot).
It listens for voice commands through a microphone and sends them
to the robot via NetworkTables.

PUSH-TO-TALK: The microphone only processes audio when you hold the
left bumper on your gamepad. If no gamepad is detected, it falls
back to always-listening mode.

REQUIREMENTS (install on driver station laptop):
    pip install vosk sounddevice inputs pyntcore

VOSK MODEL:
    Download from: https://alphacephei.com/vosk/models
    Get: vosk-model-small-en-us-0.15 (~50MB)
    Unzip into: voice_command/model/

USAGE:
    python voice_commander.py                      (voice + push-to-talk)
    python voice_commander.py --ip 10.101.66.2     (connect to real robot)
    python voice_commander.py --text               (type commands instead)
    python voice_commander.py --button BTN_TR      (use right bumper)

COMMANDS (say "viking" first, then the command):
    viking forward 3    — Drive forward 3 meters
    viking reverse 2    — Drive backward 2 meters
    viking left 1       — Strafe left 1 meter
    viking right 2      — Strafe right 2 meters
    viking turn 90      — Turn right 90 degrees
    viking turn negative 45 — Turn left 45 degrees
    viking reset        — Face field-forward
    viking stop         — Stop all movement
"""

import json
import sys
import time
import argparse
import os
import threading
```

These are all standard Python libraries — `json` for parsing Vosk results, `time` for timestamps, `argparse` for command-line options, `os` for file paths, and `threading` for reading the gamepad in the background.

### Step 2: Configuration

Next, add the settings you can customize. These go right after the imports:

```python
TEAM_NUMBER = 10166
NT_TABLE = f"/voice/{TEAM_NUMBER}"

WAKE_WORD = "viking"

COMMANDS = ["reset", "forward", "reverse", "left", "right", "turn", "stop"]

DEFAULT_PTT_BUTTON = "BTN_TL"
```

- `TEAM_NUMBER` — Must match your `constants.py`. Used in the NetworkTables path so your commands don't interfere with other teams at a competition.
- `WAKE_WORD` — The word you say before every command. This prevents random speech from triggering commands. You could change it to `"thunder"`, `"robot"`, or anything you want.
- `COMMANDS` — The list of words the robot understands as commands.
- `DEFAULT_PTT_BUTTON` — Which gamepad button activates push-to-talk. `"BTN_TL"` is the left bumper on most Xbox controllers.

### Step 3: Number Words

Vosk sometimes hears "three" as the word and sometimes as "3" the digit. This dictionary lets us handle both:

```python
NUMBER_WORDS = {
    "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4,
    "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
    "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
    "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17,
    "eighteen": 18, "nineteen": 19, "twenty": 20,
    "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60,
    "seventy": 70, "eighty": 80, "ninety": 90,
}
```

So if you say "viking forward three", the script can turn "three" into the number 3. Compound numbers work too — "forty five" becomes 40 + 5 = 45.

### Step 4: The Restricted Vocabulary

This is one of the most important parts of the whole system:

```python
VOCAB_LIST = (
    [WAKE_WORD]
    + COMMANDS
    + ["negative"]
    + list(NUMBER_WORDS.keys())
    + [str(i) for i in range(0, 21)]
    + ["[unk]"]
)
```

We give Vosk a **restricted vocabulary** — it can ONLY recognize these specific words. Why does this matter? At a competition, the crowd is screaming, music is blasting, and other teams are yelling. If we let Vosk listen for all English words, it would constantly try to match that noise to random words and might accidentally trigger commands.

With a restricted vocabulary, Vosk can only hear our command words. Everything else gets caught by `[unk]` (the "unknown" token), which we simply ignore. This makes the system much more reliable in noisy environments.

### Step 5: Optional Imports

These libraries aren't strictly required — the script can work without them in limited modes:

```python
NT_AVAILABLE = False
try:
    from ntcore import NetworkTableInstance
    NT_AVAILABLE = True
except ImportError:
    pass

INPUTS_AVAILABLE = False
try:
    import inputs
    INPUTS_AVAILABLE = True
except ImportError:
    pass
```

We wrap these in `try/except` so the script doesn't crash if they're not installed:
- Without `pyntcore` — The script runs in **offline mode** (prints commands to the console instead of sending to the robot). Great for testing that your mic and model work.
- Without `inputs` — The script runs without push-to-talk (always listening). Still works, just no button control.

### Step 6: Command Parsing Functions

Now we need functions to understand what you said. First, a helper to turn spoken number words into actual numbers:

```python
def parse_number(words):
    value = 0
    negative = False
    for w in words:
        if w == "negative":
            negative = True
        elif w in NUMBER_WORDS:
            value += NUMBER_WORDS[w]
        else:
            try:
                value += int(w)
            except ValueError:
                pass
    if negative:
        value = -value
    return value
```

This loops through the words after the command and adds up the numbers. If it hears "negative", it makes the result negative. Examples:
- `["three"]` → 3
- `["forty", "five"]` → 45 (40 + 5)
- `["negative", "ninety"]` → -90

Next, the main parsing function that takes a full sentence and extracts the command and value:

```python
def parse_command(text):
    words = text.lower().split()

    try:
        wake_idx = words.index(WAKE_WORD)
    except ValueError:
        return None, 0

    after_wake = words[wake_idx + 1:]
    if not after_wake:
        return None, 0

    cmd = after_wake[0]
    if cmd not in COMMANDS:
        return None, 0

    if cmd in ("stop", "reset"):
        return cmd, 0

    number_words = after_wake[1:]
    if not number_words:
        return cmd, 1
    value = parse_number(number_words)
    return cmd, value
```

Here's what it does step by step:
1. Finds the wake word ("viking") in the sentence — if it's not there, ignore everything
2. Takes the word right after "viking" as the command
3. Checks that word is in our `COMMANDS` list
4. For "stop" and "reset", no number is needed
5. For movement commands, it parses the remaining words as a number (defaults to 1 if none given)

Examples:
- "viking forward three" → command="forward", value=3
- "viking stop" → command="stop", value=0
- "viking turn negative forty five" → command="turn", value=-45
- "viking left" → command="left", value=1 (default)

### Step 7: Publishing Commands

This function sends a recognized command to the robot via NetworkTables:

```python
def publish_command(table, command, value):
    if table is not None:
        table.putString("command", command)
        table.putNumber("value", float(value))
        table.putNumber("timestamp", time.time())
        print(f"  >> Sent to robot: command='{command}', value={value}")
    else:
        print(f"  >> [OFFLINE] Recognized: command='{command}', value={value}")
```

Three values are published:
- `command` — The command name (e.g., "forward")
- `value` — The number (e.g., 3.0)
- `timestamp` — The current time. The robot checks this to know when a **new** command arrives (if the timestamp hasn't changed, it's the same old command)

If we're in offline mode (`table is None`), it just prints to the console instead.

### Step 8: Model Path Helper

When you unzip the Vosk model, sometimes you end up with an extra folder layer. This function handles that automatically:

```python
def resolve_model_path(model_path):
    if not os.path.exists(model_path):
        return model_path

    expected_items = {"am", "conf", "graph", "ivector"}
    contents = set(os.listdir(model_path))

    if contents & expected_items:
        return model_path

    subdirs = [
        d for d in contents
        if os.path.isdir(os.path.join(model_path, d)) and not d.startswith(".")
    ]
    if len(subdirs) == 1:
        candidate = os.path.join(model_path, subdirs[0])
        sub_contents = set(os.listdir(candidate))
        if sub_contents & expected_items:
            return candidate

    return model_path
```

It looks for the expected model folders (`am/`, `conf/`, `graph/`, `ivector/`). If they're not in the directory you pointed to, it checks if there's a single subfolder that contains them. So both of these layouts work:
- `model/am/` (direct)
- `model/vosk-model-small-en-us-0.15/am/` (extra subfolder)

### Step 9: Gamepad Push-to-Talk

This is the class that reads your gamepad's left bumper in the background:

```python
class GamepadPTT:
    def __init__(self, button_code):
        self.button_code = button_code
        self._pressed = False
        self._running = True

    def start(self):
        thread = threading.Thread(target=self._read_loop, daemon=True)
        thread.start()

    def stop(self):
        self._running = False

    @property
    def is_pressed(self):
        return self._pressed

    def _read_loop(self):
        while self._running:
            try:
                events = inputs.get_gamepad()
                for event in events:
                    if event.ev_type == "Key" and event.code == self.button_code:
                        self._pressed = event.state == 1
            except inputs.UnpluggedError:
                self._pressed = False
                time.sleep(1.0)
            except Exception:
                time.sleep(0.01)
```

How this works:
- `_read_loop` runs in a **background thread** (a separate task running at the same time as the main voice code)
- It constantly reads gamepad events using `inputs.get_gamepad()`
- When it sees a button press event (`ev_type == "Key"`) matching our configured button code, it updates `_pressed`
- `event.state == 1` means pressed, `0` means released
- If the gamepad gets unplugged, it catches the error and waits before retrying

Now add the helper function that sets up the gamepad:

```python
def init_gamepad(button_code):
    if not INPUTS_AVAILABLE:
        print("No gamepad library — always-listening mode. Install: pip install inputs")
        return None

    gamepads = inputs.devices.gamepads
    if not gamepads:
        print("No gamepad detected — always-listening mode.")
        return None

    print(f"Gamepad: {gamepads[0].name}")
    print(f"Push-to-talk: hold {button_code} to speak\n")

    ptt = GamepadPTT(button_code)
    ptt.start()
    return ptt
```

This checks if the `inputs` library is installed and if a gamepad is actually plugged in. If either is missing, it returns `None` and the script falls back to always-listening mode. If everything is good, it creates a `GamepadPTT` object and starts listening for button presses.

### Step 10: Text Mode

Text mode lets you test the command system by typing instead of speaking:

```python
def run_text_mode(table):
    print("\n=== TEXT INPUT MODE ===")
    if table is None:
        print("(OFFLINE — commands will be printed only)")
    print(f"Type: {WAKE_WORD} forward 3")
    print(f"Type: {WAKE_WORD} stop")
    print("Type 'quit' to exit.\n")

    while True:
        try:
            text = input("Command> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if text.lower() == "quit":
            break
        if not text:
            continue

        cmd, val = parse_command(text)
        if cmd:
            publish_command(table, cmd, val)
        else:
            print(f"  (not recognized — start with '{WAKE_WORD}' then a command)")
```

This is a simple loop: it asks you to type a command, parses it using the same `parse_command()` function the voice mode uses, and sends it. This is really useful for testing because you don't need a microphone — just type `viking forward 3` and see if the robot responds.

### Step 11: Voice Mode

This is the big one — the actual voice recognition loop. It ties everything together:

```python
def run_voice_mode(table, model_path, ptt_button):
    try:
        import sounddevice as sd
        from vosk import Model, KaldiRecognizer
    except ImportError as e:
        print(f"Missing package: {e}")
        print("Install: pip install vosk sounddevice")
        print("Falling back to text mode...\n")
        run_text_mode(table)
        return

    model_path = resolve_model_path(model_path)

    if not os.path.exists(model_path):
        print(f"Vosk model not found at '{model_path}'")
        print("Download from: https://alphacephei.com/vosk/models")
        print("Falling back to text mode...\n")
        run_text_mode(table)
        return

    print(f"Loading Vosk model from '{model_path}'...")
    try:
        model = Model(model_path)
    except Exception:
        print("Failed to load model. Make sure model/ contains am/, conf/, graph/ folders.")
        print("Falling back to text mode...\n")
        run_text_mode(table)
        return

    grammar = json.dumps(VOCAB_LIST)
    recognizer = KaldiRecognizer(model, 16000, grammar)

    ptt = init_gamepad(ptt_button)
    has_ptt = ptt is not None

    if table is None:
        print("(OFFLINE — commands will be printed only)")
    print(f"Say '{WAKE_WORD}' followed by a command. Press Ctrl+C to stop.\n")

    was_active = False

    try:
        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16", channels=1) as stream:
            if has_ptt:
                print("Ready! Waiting for push-to-talk button...")
            else:
                print("Listening... (speak clearly)")

            while True:
                active = ptt.is_pressed if has_ptt else True

                if active and not was_active:
                    recognizer.Reset()
                    print("\n  [LISTENING]")
                elif not active and was_active:
                    text = json.loads(recognizer.FinalResult()).get("text", "").strip()
                    if text:
                        print(f"  Heard: '{text}'")
                        cmd, val = parse_command(text)
                        if cmd:
                            publish_command(table, cmd, val)
                    print("  [RELEASED]")

                was_active = active

                if active:
                    data, _ = stream.read(8000)
                    if recognizer.AcceptWaveform(bytes(data)):
                        text = json.loads(recognizer.Result()).get("text", "").strip()
                        if text:
                            print(f"  Heard: '{text}'")
                            cmd, val = parse_command(text)
                            if cmd:
                                publish_command(table, cmd, val)
                else:
                    time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping voice commander.")
    except Exception as e:
        print(f"Microphone error: {e}")
        print("Falling back to text mode...\n")
        run_text_mode(table)
    finally:
        if ptt is not None:
            ptt.stop()
```

There's a lot here, so let's break it down:

**Setup phase** (the first ~30 lines):
1. Import `sounddevice` and `vosk` — if they're missing, fall back to text mode
2. Find the Vosk model — if it's not there, fall back to text mode
3. Load the model and create a `KaldiRecognizer` with our restricted `VOCAB_LIST`
4. Set up the gamepad for push-to-talk

**The main loop** (inside `while True`):
- `active` is `True` when the push-to-talk button is held (or always `True` if no gamepad)
- When the button is **first pressed** (`active and not was_active`): reset the recognizer and print `[LISTENING]`
- When the button is **released** (`not active and was_active`): get the final result from Vosk, parse it, and send the command. Print `[RELEASED]`
- While the button is held: read audio from the microphone and feed it to Vosk. If Vosk has enough audio to recognize something mid-speech, it processes that too
- While the button is NOT held: just sleep briefly to avoid burning CPU

**Error handling**: If something goes wrong with the microphone, the script falls back to text mode so you can still test.

### Step 12: The Main Function

Finally, the entry point that ties everything together:

```python
def main():
    parser = argparse.ArgumentParser(description="ThunderVikes Voice Commander")
    # Robot IP addresses:
    #   127.0.0.1    — Simulator running on your laptop (default)
    #   172.22.11.2  — Robot connected via USB
    #   10.101.66.2  — Robot over WiFi/radio (pattern: 10.TE.AM.2)
    parser.add_argument("--ip", default="127.0.0.1", help="Robot IP (default: 127.0.0.1)")
    parser.add_argument("--text", action="store_true", help="Type commands instead of speaking")
    parser.add_argument("--model", default=None, help="Path to Vosk model directory")
    parser.add_argument("--button", default=DEFAULT_PTT_BUTTON, help="Gamepad button for push-to-talk (default: BTN_TL)")
    args = parser.parse_args()

    model_path = args.model
    if model_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "model")

    print(f"ThunderVikes Voice Commander — Team {TEAM_NUMBER}\n")

    table = None
    nt_inst = None

    if NT_AVAILABLE:
        print(f"Connecting to NetworkTables at {args.ip}...")
        nt_inst = NetworkTableInstance.create()
        nt_inst.setServer(args.ip)
        nt_inst.startClient4("voice_commander")

        start = time.time()
        while not nt_inst.isConnected() and time.time() - start < 5:
            time.sleep(0.1)

        if nt_inst.isConnected():
            print("Connected!")
            table = nt_inst.getTable(NT_TABLE)
            table.putString("command", "none")
            table.putNumber("value", 0.0)
            table.putNumber("timestamp", 0.0)
        else:
            print("Could not connect — running in OFFLINE mode.\n")
    else:
        print("OFFLINE mode (install pyntcore to send commands to robot).\n")

    if args.text:
        run_text_mode(table)
    else:
        run_voice_mode(table, model_path, args.button)


if __name__ == "__main__":
    main()
```

What this does:
1. **Parse command-line arguments** — `argparse` lets you customize behavior when running the script:
   - `--ip` sets the robot's IP address. The comments in the code list the three IPs you'll use:

     | Scenario | IP Address | When to use |
     |---|---|---|
     | Simulator | `127.0.0.1` (default) | Testing on your laptop without a robot |
     | USB connection | `172.22.11.2` | Robot plugged into your laptop via USB cable |
     | WiFi / radio | `10.101.66.2` | Robot connected over WiFi or the FRC radio at competition |

     The WiFi IP follows the FRC pattern `10.TE.AM.2` — for team 10166 that's `10.101.66.2`.
   - `--text` switches to text mode (type commands instead of speaking)
   - `--model` points to a custom Vosk model path
   - `--button` changes the push-to-talk button (defaults to `BTN_TL`, the left bumper)
2. **Connect to NetworkTables** — We create a NetworkTables instance using `NetworkTableInstance.create()`, point it at the robot's IP with `setServer()`, and start it as a client with `startClient4("voice_commander")`. This uses the same NT4 protocol the robot uses. If it can't connect within 5 seconds, it switches to offline mode.
3. **Initialize the table** — Sets the command, value, and timestamp to their defaults so the robot doesn't see stale data.
4. **Start voice or text mode** — Based on the `--text` flag.

### Running the Script

```bash
# Default (voice mode, push-to-talk, connects to simulator):
python voice_commander.py

# Connect to robot via USB cable:
python voice_commander.py --ip 172.22.11.2

# Connect to robot over WiFi / radio:
python voice_commander.py --ip 10.101.66.2

# Use right bumper instead of left:
python voice_commander.py --button BTN_TR

# Text mode (type instead of speak):
python voice_commander.py --text
```

If the robot/simulator isn't running, the script automatically runs in **offline mode** and prints commands to the console so you can test recognition without a robot.

---

## Part 4: The Voice Receiver Component

Now we switch to the **robot side**. This MagicBot component runs on the robot and reads commands from NetworkTables.

### File: `components/voice_receiver.py`

```python
import wpilib
from ntcore import NetworkTableInstance
import constants


class VoiceReceiver:

    _last_timestamp = 0.0
    _current_command = ""
    _current_value = 0.0
    _command_consumed = True

    def setup(self) -> None:
        nt = NetworkTableInstance.getDefault()
        self.voice_table = nt.getTable(constants.kVoiceNTTable)
```

The `setup()` method connects to the NetworkTables table where the voice commander publishes commands. We use `constants.kVoiceNTTable` instead of hardcoding the path — if the team number changes, you only update it in one place.

The four class variables track state:
- `_last_timestamp` — The timestamp of the last command we saw
- `_current_command` — The command string (e.g., "forward")
- `_current_value` — The number value (e.g., 3.0)
- `_command_consumed` — Has `robot.py` already acted on this command?

```python
    def get_command(self) -> str:
        return self._current_command

    def get_value(self) -> float:
        return self._current_value

    def has_new_command(self) -> bool:
        return not self._command_consumed and self._current_command != "" and self._current_command != "none"

    def clear_command(self) -> None:
        self._command_consumed = True
```

These methods let `robot.py` interact with voice commands:
- `has_new_command()` — Returns `True` only when there's a command that hasn't been acted on yet
- `get_command()` — Returns the command string (e.g., "forward")
- `get_value()` — Returns the number value (e.g., 3.0)
- `clear_command()` — Marks the command as handled (so it doesn't repeat every 20ms)

```python
    def execute(self) -> None:
        timestamp = self.voice_table.getNumber("timestamp", 0.0)
        if timestamp > self._last_timestamp:
            self._last_timestamp = timestamp
            self._current_command = self.voice_table.getString("command", "none")
            self._current_value = self.voice_table.getNumber("value", 0.0)
            self._command_consumed = False
```

Every 20ms, MagicBot calls `execute()`. It checks if the timestamp in NetworkTables has changed — if it has, that means the voice commander sent a new command. The component reads the command and value, and sets `_command_consumed = False` so `robot.py` knows to act on it.

---

## Part 5: Robot.py Changes

### Step 1: Add the Import

At the top of `robot.py`, add:

```python
from components.voice_receiver import VoiceReceiver
```

### Step 2: Declare the Component

In the component declarations section (with the other component type hints):

```python
voice_receiver: VoiceReceiver
```

MagicBot will automatically create it and call its `setup()` and `execute()` methods.

### Step 3: Initialize Voice State in teleopInit

```python
def teleopInit(self) -> None:
    self._voice_active = False
    self._voice_start_time = 0.0
    self._voice_duration = 0.0
    self._voice_x = 0.0
    self._voice_y = 0.0
    self._voice_rot = 0.0
```

These variables track whether a voice command is currently being executed:
- `_voice_active` — Is the robot currently moving because of a voice command?
- `_voice_start_time` — When did the current movement start?
- `_voice_duration` — How long should the movement last (in seconds)?
- `_voice_x`, `_voice_y`, `_voice_rot` — The drive command values for the current movement

### Step 4: Add Voice Command Handling in teleopPeriodic

This goes after the joystick control section. Let's go through it piece by piece.

**First, the safety check:**

```python
        bumper_held = self.driver_controller.getLeftBumper()

        if not bumper_held and self._voice_active:
            self._voice_active = False
```

If the driver releases the left bumper while a voice command is executing, movement stops immediately. This is your emergency brake for voice commands.

**Next, execute the active voice movement:**

```python
        if self._voice_active:
            elapsed = time.monotonic() - self._voice_start_time
            if elapsed < self._voice_duration:
                self.swerve_drive.set_drive_command(
                    self._voice_x, self._voice_y, self._voice_rot, True, False,
                )
            else:
                self._voice_active = False
```

If a voice command is active, check how much time has passed. If we haven't hit the target duration yet, keep driving. Once the time is up, stop.

**Finally, check for new commands:**

```python
        if bumper_held and self.voice_receiver.has_new_command():
            cmd = self.voice_receiver.get_command()
            val = self.voice_receiver.get_value()
            self.voice_receiver.clear_command()

            speed_frac = constants.kVoiceDriveSpeed / constants.kMaxSpeed
            turn_frac = constants.kVoiceTurnSpeed / constants.kMaxAngularSpeed
            max_distance = 10.0
            max_turn_degrees = 360.0

            if cmd == "stop":
                self._voice_active = False
                self.swerve_drive.set_drive_command(0, 0, 0, True, False)

            elif cmd == "reset":
                self._voice_active = False
                self.swerve_drive.zero_heading()

            elif cmd == "forward":
                distance = min(val if val > 0 else 1, max_distance)
                self._voice_active = True
                self._voice_start_time = time.monotonic()
                self._voice_duration = distance / constants.kVoiceDriveSpeed
                self._voice_x = speed_frac
                self._voice_y = 0.0
                self._voice_rot = 0.0

            elif cmd == "reverse":
                distance = min(val if val > 0 else 1, max_distance)
                self._voice_active = True
                self._voice_start_time = time.monotonic()
                self._voice_duration = distance / constants.kVoiceDriveSpeed
                self._voice_x = -speed_frac
                self._voice_y = 0.0
                self._voice_rot = 0.0

            elif cmd == "left":
                distance = min(val if val > 0 else 1, max_distance)
                self._voice_active = True
                self._voice_start_time = time.monotonic()
                self._voice_duration = distance / constants.kVoiceDriveSpeed
                self._voice_x = 0.0
                self._voice_y = speed_frac
                self._voice_rot = 0.0

            elif cmd == "right":
                distance = min(val if val > 0 else 1, max_distance)
                self._voice_active = True
                self._voice_start_time = time.monotonic()
                self._voice_duration = distance / constants.kVoiceDriveSpeed
                self._voice_x = 0.0
                self._voice_y = -speed_frac
                self._voice_rot = 0.0

            elif cmd == "turn":
                degrees = val if val != 0 else 90
                degrees = max(-max_turn_degrees, min(degrees, max_turn_degrees))
                duration = abs(math.radians(degrees)) / constants.kVoiceTurnSpeed
                direction = 1.0 if degrees > 0 else -1.0
                self._voice_active = True
                self._voice_start_time = time.monotonic()
                self._voice_duration = duration
                self._voice_x = 0.0
                self._voice_y = 0.0
                self._voice_rot = -direction * turn_frac
```

New commands are only accepted when the bumper is held (`bumper_held and self.voice_receiver.has_new_command()`). For each movement command, we calculate:
- **How long to drive** — `distance / speed = seconds` (e.g., 3 meters / 1.0 m/s = 3 seconds)
- **What fraction of max speed** — `set_drive_command()` takes -1.0 to 1.0 values (fractions of max speed), not raw m/s. So `speed_frac = 1.0 / 4.0 = 0.25` means 25% of max speed.

### How Timer-Based Movement Works

When you say "viking forward three", the robot drives at `kVoiceDriveSpeed` (1.0 m/s) for `distance / speed` seconds (3 / 1.0 = 3 seconds). It's not perfectly precise (wheels can slip, the floor might be uneven), but it works well enough for voice-commanded movement.

### Safety Features

1. **Bumper release = emergency stop** — Let go of the bumper and movement stops immediately
2. **Distance cap** — Max 10 meters per command, max 360 degrees per turn
3. **Bumper required** — New commands only accepted while the bumper is held

---

## Part 6: How to Customize

### Change the Activation Button (robot.py)

Replace `getLeftBumper()` with any of these:

| Method | Button |
|---|---|
| `getRightBumper()` | Right bumper |
| `getAButton()` | A button |
| `getBButton()` | B button |
| `getYButton()` | Y button |

### Change the Wake Word (voice_commander.py)

```python
WAKE_WORD = "viking"   # Change to "thunder", "robot", etc.
```

Keep it to a single distinct word that doesn't sound like common crowd noise.

### Change the Speed (constants.py, both copies!)

```python
kVoiceDriveSpeed = 1.0        # Higher = faster movement
kVoiceTurnSpeed = math.pi / 4 # Higher = faster turning
```

### Add a New Command

1. In `voice_commander.py`, add the word to the `COMMANDS` list
2. In `robot.py`, add an `elif` block to handle it

Example — "viking spin" for a full 360:

```python
elif cmd == "spin":
    self._voice_active = True
    self._voice_start_time = time.monotonic()
    self._voice_duration = (2 * math.pi) / constants.kVoiceTurnSpeed
    self._voice_x = 0.0
    self._voice_y = 0.0
    self._voice_rot = -turn_frac
```

---

## Part 7: Testing

### Text Mode (no microphone needed)

1. Start the robot simulation
2. In a separate terminal:
   ```bash
   cd RobotMain && python voice_command/voice_commander.py --text
   ```
3. Type: `viking forward 3` and press Enter
4. Hold the left bumper in the simulator and watch the robot move

### Voice Mode

1. Make sure the Vosk model is downloaded (Part 1)
2. Plug in your gamepad
3. Run:
   ```bash
   cd RobotMain && python voice_command/voice_commander.py
   ```
4. Hold the left bumper and say: "Viking forward three"
5. Release the bumper — the command processes and sends

### Troubleshooting

| Problem | Solution |
|---|---|
| "Vosk model not found" | Download the model and place it in `voice_command/model/` |
| "PortAudio library not found" | Mac: `brew install portaudio`, Linux: `sudo apt install portaudio19-dev` |
| Robot doesn't move | Make sure teleop is enabled AND you're holding the left bumper |
| Always listening (no push-to-talk) | Install the `inputs` library: `pip install inputs` |

---

## Part 8: Competition Tips

1. **Use a headset microphone** — Clip-on or gaming headsets work much better than laptop mics in a noisy arena
2. **Speak clearly with pauses** — "VIKING... FORWARD... THREE" works better than rushing
3. **Always have joystick backup** — Voice commands are cool but joysticks are more reliable
4. **Test with loud music** — Simulate competition noise to see how well it handles background sound

---

## Summary

| File | Location | Purpose |
|---|---|---|
| `voice_commander.py` | `voice_command/` | Laptop script — listens to mic, sends commands via NetworkTables |
| `voice_receiver.py` | `components/` | Robot component — reads commands from NetworkTables |
| `robot.py` | `RobotMain/` | Checks left bumper + executes voice movement |
| `constants.py` | Both copies | Speed and NetworkTables configuration |

**The flow:** You speak -> Vosk recognizes -> Script sends to NetworkTables -> Voice receiver reads it -> robot.py checks left bumper -> swerve drive moves the robot.
