# Python Basics for FRC Students (RobotPy + MagicBot) — Beginner Guide
**Audience:** High school students who are new to Python  
**Goal:** Learn core Python concepts *and* how they connect to your FRC robot code.

> You do **not** need to memorize everything. You need to understand the *ideas* and practice typing small examples.

---

## 1) Variables (Python vs Java)

### 1.1 What is a variable?
A **variable** is a named box that stores a value.

In Python, you create a variable by **assigning** a value:

```python
speed = 1.5
name = "ThunderVikes"
enabled = True
```

### 1.2 How Python differs from Java
In Java, you usually write the type first:

```java
double speed = 1.5;
String name = "ThunderVikes";
boolean enabled = true;
```

In Python, you usually **do not** write the type:

```python
speed = 1.5
name = "ThunderVikes"
enabled = True
```

Python figures out the type automatically at runtime.

### 1.3 Common data types
| Type | Example | What it means |
|------|---------|---------------|
| `int` | `count = 3` | whole numbers |
| `float` | `speed = 1.5` | decimal numbers |
| `str` | `msg = "hello"` | text |
| `bool` | `ready = True` | true/false |
| `None` | `target = None` | “no value yet” |

Quick test in Python:
```python
x = 3
print(type(x))   # <class 'int'>
```

### 1.4 Type hints (NOT like Java types)
Python has something called **type hints**:

```python
speed: float = 1.5
count: int = 3
name: str = "ThunderVikes"
```

**Important:** Type hints do NOT force the type at runtime.
They are mainly for:
- readability
- catching mistakes in your editor (PyCharm)
- helping MagicBot injection (for robot hardware)

Example: you can still do this (Python will allow it):
```python
speed: float = 1.5
speed = "fast"   # This is allowed, but it's a bad idea.
```

### 1.5 Lists (like Java arrays, but easier)
A **list** stores many items in order:

```python
numbers = [1, 2, 3]
names = ["Ava", "Leo", "Kai"]
```

Useful in FRC when you have:
- a list of motors
- a list of modules
- a list of autonomous steps

Example:
```python
modules = ["FL", "FR", "BL", "BR"]
print(modules[0])  # FL
```

### 1.6 Dictionaries (like Java HashMap)
A **dictionary** stores key → value pairs:

```python
robot_info = {"team": 10166, "name": "ThunderVikes"}
print(robot_info["team"])  # 10166
```

Useful in FRC when you want:
- lookup tables (tag id → goal pose)
- tracking sensor readings by name
- remembering which AprilTags you've seen

Example:
```python
seen_tags = {}
seen_tags[19] = "goal tag"
print(seen_tags)
```

---

## 2) Functions / Methods

### 2.1 What is a function?
A **function** is a named set of instructions you can run.

```python
def say_hi():
    print("Hi!")
```

Call it like:
```python
say_hi()
```

### 2.2 Functions with inputs (parameters)
```python
def add(a, b):
    return a + b
```

Call it:
```python
result = add(2, 5)
print(result)  # 7
```

### 2.3 Type hints for functions
Type hints help PyCharm catch mistakes:

```python
def add(a: int, b: int) -> int:
    return a + b
```

- `a: int` means “a should be an int”
- `-> int` means “this function returns an int”

Again: type hints are guidance, not enforcement.

### 2.4 Returning values
If a function returns something, you can store it:

```python
def get_speed() -> float:
    return 1.5

speed = get_speed()
```

If a function returns nothing, it returns `None`:

```python
def print_speed(speed: float) -> None:
    print(speed)
```

---

## 3) Classes, Objects, and Imports

### 3.1 Why classes exist
A **class** groups:
- data (variables)
- behavior (functions)

Together, this becomes a “thing” (an object).

In FRC:
- `SwerveDrive` is a class
- `SwerveModule` is a class
- `XboxController` is a class

### 3.2 Imports (using libraries)
Python uses `import` to use code from other files or libraries:

```python
import math
print(math.pi)
```

RobotPy libraries you will import often:
```python
import wpilib
import rev
```

### 3.3 A simple class example
```python
class Counter:
    def __init__(self):
        self.value = 0

    def increment(self) -> None:
        self.value += 1

    def get(self) -> int:
        return self.value
```

Using the object:
```python
c = Counter()
c.increment()
print(c.get())  # 1
```

### 3.4 Class properties vs methods
- **Property:** data stored on the object (`self.value`)
- **Method:** function on the object (`increment()`)

### 3.5 Type hints on class properties (important for MagicBot)
MagicBot uses type hints to inject robot hardware into components:

```python
class MySubsystem:
    motor: rev.SparkMax   # MagicBot injects this if robot.py creates it
```

This is why we **type-hint** hardware in components.

---

## 4) Logic Flow (Conditionals + Loops)

### 4.1 Conditionals (if/elif/else)
```python
score = 15

if score >= 20:
    print("Great!")
elif score >= 10:
    print("Good!")
else:
    print("Keep trying!")
```

Python runs the first True block and skips the rest.

### 4.2 Challenge: FizzBuzz (classic logic practice)
**Goal:** For numbers 1 to 20:
- print “Fizz” if divisible by 3
- print “Buzz” if divisible by 5
- print “FizzBuzz” if divisible by both
- otherwise print the number

Starter code:
```python
for i in range(1, 21):
    # TODO: your logic here
    print(i)
```

Hint:
- “divisible by 3” means `i % 3 == 0`
- check “both” first

### 4.3 For loops (counting)
```python
for i in range(5):
    print(i)
```

Output:
0,1,2,3,4

### 4.4 While loops (repeat until condition changes)
```python
count = 0
while count < 5:
    print(count)
    count += 1
```

In FRC, while loops can be dangerous if they never end.
Robots must update at ~50 times per second. If you block the loop, the robot “freezes.”

So in robot code, we usually use **state machines** instead of long while loops.

### 4.5 Loops with lists (append)
```python
values = []
for i in range(3):
    values.append(i * 10)
print(values)  # [0, 10, 20]
```

### 4.6 Using dictionaries with conditionals
Common pattern: “If key exists, update it; otherwise add it.”

```python
seen = {}

tag_id = 19
if tag_id not in seen:
    seen[tag_id] = 1
else:
    seen[tag_id] += 1

print(seen)  # {19: 1} then {19: 2}
```

This is useful for tracking which AprilTags have been seen.

---

## 5) Try / Except (Handling Errors Safely)

### 5.1 Why try/except matters in robot code
Robots run live on the field. Errors happen:
- sensor unplugged
- NetworkTables value missing
- None values because a device didn’t inject correctly

Without handling, code can crash or behave dangerously.

### 5.2 Basic pattern
```python
try:
    x = 10 / 0
except Exception as e:
    print("Error happened:", e)
```

### 5.3 Where to use it in FRC
Good places:
- reading vision/network values
- reading encoders/gyro
- any place you might get `None`

Example (Limelight read):
```python
try:
    tv = self.ll.getNumber("tv", 0)
    if tv < 1:
        return
    tx = self.ll.getNumber("tx", 0.0)
except Exception as e:
    wpilib.reportError(f"Limelight read failed: {e}", True)
```

### 5.4 Using `finally`
`finally` always runs, even if there’s an error.

This is good for safety (like stopping motors):

```python
try:
    # do something risky
    pass
except Exception as e:
    print("Error:", e)
finally:
    # safety stop
    self.swerve.drive(0.0, 0.0, 0.0)
```

### 5.5 The big rule: don’t hide errors forever
Try/except should help you:
- print the error
- safely stop
- keep the robot running if possible

But you should still fix the real bug after.

---

## How this applies to our FRC robot project
- Variables store joystick inputs, speeds, angles
- Functions organize repeated math and tasks
- Classes represent robot subsystems (swerve, shooter, vision)
- Logic flow (if/loops) powers teleop + autonomous state machines
- Try/except keeps the robot safe and easier to debug

---

### Suggested student exercises (quick)
1) Write a function that converts joystick value (-1..1) into m/s using a max speed.
2) Write a dict that tracks AprilTag sightings and counts them.
3) Write a class `Shooter` with a method `shoot()` that prints “shoot”.
4) Solve FizzBuzz.

---
