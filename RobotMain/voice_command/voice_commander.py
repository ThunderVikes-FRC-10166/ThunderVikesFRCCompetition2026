# Voice_Commander.py-Voice Command system for Team 10166
""""
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

TEAM_NUMBER = 10166
NT_TABLE = f"/voice/{TEAM_NUMBER}"

WAKE_WORD = "viking"

COMMANDS = ["reset", "foward", "reverse", "left", "right", "turn", "stop"]

DEFAULT_PTT_BUTTON = "BTN_TL"

NUMBER_WORDS = {  "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4,
    "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
    "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
    "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17,
    "eighteen": 18, "nineteen": 19, "twenty": 20,
    "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60,
    "seventy": 70, "eighty": 80, "ninety": 90,
    }
VOCAB_LIST = (
    ["viking"]
    +COMMANDS
    +["negative"]
    +list(NUMBER_WORDS.keys())
    +[str(i) for i in range(0, 21)]
    +["[unk]"]
)
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
def parse_command(text):
    words = text.lower().split()

    try:
        wake_idx = words.index(WAKE_WORD)
    except ValueError:
        return None, 0

    after_wake = words[wake_idx + 1]
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
def publish_command(table, command,value):
    if table is not None:
        table.putString("command", command)
        table.putNumber("value", float(value))
        table.putNumber("timestamp", time.time())
        print(f" >> Sent to robot: command= '{command}', value={value}")
    else:
        print(f" >>[OFFLINE] Recognized: command= '{command}', value={value}")
def resolve_model_path(model_path):
    if not os.path.exists(model_path):
        return model_path

    expected_items= {"am", "conf","graph","ivector"}
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
class GamepadPTT:
    def __inti__(self, button_code):
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
                    if event.ev_type == "key" and event.code == self.button_code:
                        self._pressed = event.state == 1
            except inputs.UnpluggedError:
                self._pressed = False
                time.sleep(0.1)
            except Exception:
                time.sleep(0.01)

def init_gamepad(button_code):
        if not INPUTS_AVAILABLE:
            print("No gamepad libary - always-listening mode. Install: pip install inputs")
            return None

        gamepads = inputs.devices.gamepads
        if not gamepads:
             print("No gamepad detected - always listening mode.")
             return None

        print(f"Gamepad: {gamepads[0].name}")
        print(f"Push-to-talk: hold {button_code} to speak\n")

        ptt = GamepadPTT(button_code)
        ptt.start()
        return ptt


def run_text_mode(table):
    print("\n== TEXT INPUT MODE ===")
    if table is None:
        print("OFFLINE - commands will be printed only)")
    print(f"Type: {WAKE_WORD} foward 3")
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
            print(f"  (not recognized - start with '{WAKE_WORD}' them a command)")


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

    print(f"Loading model from '{model_path}'...")
    try:
        model = Model(model_path)
    except Exception:
        print("Failed to load model. Make sure model/ contains am/, graph/ folders.")
        print("Falling back to text mode...\n")
        run_text_mode(table)
        return

    grammer = json.dumps(VOCAB_LIST)
    recognizer = KaldiRecognizer(model, 16000, grammer)

    ptt = init_gamepad(ptt_button)
    has_ptt = ptt is not None

    if table is None:
        print("OFFLINE - commands will be printed only)")
    print(f"Say '{WAKE_WORD}' followed by a command. Press Ctrl+C to stop.\n")

    was_active = False

    try:
        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16", channels=1) as stream:
            if has_ptt:
                print("Ready! Waiting for push-to-talk button...")
            else:
                print("Listening...(speak clearly)")

            while True:
                active = ptt.is_pressed if has_ptt else False

                if active and not was_active:
                    recognizer.Reset()
                    print("\n [LISTENING]")
                elif not active and was_active:
                    text = json.loads(recognizer.FinalResult()).get("text","").strip()
                    if text:
                        print(f" Heard: '{text}'")
                        cmd, val = parse_command(text)
                        if cmd:
                            publish_command(table, cmd, val)
                    print(" [REALEASED]")

                was_active = active

                if active:
                    data, _= stream.read(8000)
                    if recognizer.AcceptWaveform(bytes(data)):
                        text = json.loads(recognizer.Result()).get("text","").strip()
                        if text:
                            print(f" Heard: '{text}'")
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
    finally:
        if ptt is not None:
            ptt.stop()
def main():
    parser = argparse.ArgumentParser(description="ThunderVikes Voice Commander" )
    # Robot IP adresses:
    #   127.0.0.1     -Simulator running on your laptop (default)
    #   172.22.11.2   -Robot connected via USb
    #   10.101.66.2   -Robot over Wi-Fi/radio (pattern: 10.TE.AM.2)
    parser.add_argument("--ip", defualt="127.0.0.1", help="Robot IP (defualt: 127.0.0.1)")
    parser.add_argument("--text", action="store_true", help="Type commands instead of speaking")
    parser.add_argument("--model", defualt=None, help="Path to Vosk model directory")
    parser.add_argument("--button", defualt=DEFAULT_PTT_BUTTON, help="Gamepad button for push-to-talk (defualt: BTN_TL)")
    args = parser.parse_args()

    model_path = parser.parse_args()
    if model_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "model")

    print(f"ThunderVikes Voice Commander - Team {TEAM_NUMBER}\n")

    table = None
    nt_inst = None

    if NT_AVAILABLE:
        print(f"Connecting to NetworkTables at {args.ip}...")
        nt_inst = NetworkTableInstance.create()
        nt_inst.setServer(args.ip)
        nt_inst.startClient4("voice_commander")

        start = time.time()
        while not nt_inst.isConnected() and time.time() -start < 5:
            time.sleep(0.1)

        if nt_inst.isConnected():
            print("Connected!")
            table = nt_inst.getTable(NT_TABLE)
            table.putString("command", "none")
            table.putString("command"

