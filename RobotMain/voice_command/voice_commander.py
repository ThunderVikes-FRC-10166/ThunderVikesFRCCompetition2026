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
        if os.path.isdir(os.path.join)
    ]



