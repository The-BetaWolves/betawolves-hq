# Robot Subsystems - Design & Implementation Guide

> **Purpose**: This document maps out exactly how we want everything programmed so that all team members understand the code structure and logic. If something goes sideways in any file, we'll know where it is and what kind of logic should be there.

---

## Complex Problems to Solve (Priority Order)

1. **Turret** - Auto-aiming system
2. **Shooter** - Hood control, stationary shooting, shooting from anywhere, shooting while moving
3. **Climber** - Automation
4. **Autonomous Paths** - Path planning and execution

---

# Turret Subsystem

## Design Philosophy: "Always Be Pointing"

The turret should **always** have a default command running that keeps it aimed at a target. This means:
- The turret never sits idle
- It continuously tracks based on robot position
- Target selection happens automatically based on field position

## Physical Configuration

### Turret Zero Position
The turret is **zeroed to the back of the robot** (not the front) due to:
- Electrical constraints
- Potential obstruction issues at the front

This means in code, we need to subtract π (PI) from the angle to convert it back to robot-relative "front zero".

---

## Math Concepts for Students

### Understanding Radians

**Why radians over degrees?**
- All WPILib and Java math functions use radians
- Radians represent how far around a circle we've traveled, compared to the radius
- They make the math cleaner for robotics calculations

**Key conversions:**
- 1 radian ≈ 57 degrees
- π (PI) radians = 180° = half circle
- 2π radians = 360° = full circle

**Practice problems:**
- How many degrees is 2.1 radians? → ~120°
- How many degrees is -0.5 radians? → ~-29°

### Mental Math for Robot Directions

Think of the robot from a top-down view:

```
                0 radians (FRONT)
                     ↑
                     |
-π/2 radians ←───────┼───────→ +π/2 radians
(RIGHT)              |            (LEFT)
                     ↓
              ±π radians (BACK)
```

| Direction | Radians | Degrees |
|-----------|---------|---------|
| Front     | 0       | 0°      |
| Left      | π/2     | 90°     |
| Back      | π       | 180°    |
| Right     | -π/2    | -90°    |

### What is `angleModulus`?

The `MathUtil.angleModulus()` function keeps angles within the range of -π to +π radians (-180° to +180°).

**Why do we need this?**
- Without it, angles can grow unbounded (720°, 1080°, etc.)
- It ensures we always take the shortest path to a target angle
- Prevents the turret from spinning multiple full rotations

**Example:**
- Input: 3.5 radians (~200°) → Output: ~-2.78 radians (~-160°)
- Both represent the same direction, but the output is in our standard range

### Vectors in Robotics Context

A **vector** describes both a direction and a magnitude (distance). For turret aiming:
- We create a vector from the robot to the target
- The direction tells us where to point
- The magnitude tells us how far away the target is (useful for shooter speed)

---

## How to Aim: The Math

We need **two pieces of information**:
1. The (x, y) position of the target on the field
2. The pose (x, y, heading) of the robot

### Step-by-Step Calculation

```
Given:
  - Robot position: (x_robot, y_robot)
  - Robot heading: robotHeading
  - Target position: (x_target, y_target)

Step 1: Field Relative - Get vector from robot to target
  dx = x_target - x_robot
  dy = y_target - y_robot

Step 2: Field Relative - Calculate angle to target
  targetFieldAngle = Math.atan2(dy, dx)

Step 3: Robot Relative - Subtract robot's heading
  aimRobotRelative = MathUtil.angleModulus(targetFieldAngle - robotHeading)

Step 4: Turret Relative - Reverse for back-mounted turret
  aimTurretRelative = MathUtil.angleModulus(aimRobotRelative - Math.PI)

Step 5: Apply physical limits
  turretSetpoint = MathUtil.clamp(aimTurretRelative, -limit, +limit)
```

### Practice Problem

**Given:**
- Robot is at position (2, 3)
- Target is at position (8, 7)
- What is the turret setpoint?

**Solution:**
1. dx = 8 - 2 = 6
2. dy = 7 - 3 = 4
3. targetFieldAngle = atan2(4, 6) = atan(4/6) ≈ 0.588 radians (~33.7°)
4. (Continue based on robot heading...)

*Note: To do atan2 on a calculator, use: atan(dy/dx)*

---

## Code Structure

### Constants File
```
- Blue hub position (x, y)
- Blue pass point positions (x, y)
- Red hub position (x, y)
- Red pass point positions (x, y)
- Turret rotation limits
```

### Robot Container
```java
// Instantiate Turret Subsystem
private final TurretSubsystem turretSubsystem = new TurretSubsystem();

// Set default command - ALWAYS be pointing
turretSubsystem.setDefaultCommand(
    new AimTurretCommand(turretSubsystem, drive::getPose)
);
```

### Turret Folder Structure

```
turret/
├── TurretSubsystem.java      # Hardware setup, motor methods
├── AimTurretCommand.java     # Default command, execute loop
├── CalculateAimSetpoint.java # Pure math class
└── TargetSelecter.java       # Chooses target based on position
```

### CalculateAimSetpoint Class
**Purpose:** Pure math - calculates aim setpoint from pose and target

```java
public class CalculateAimSetpoint {
    /**
     * @param robotPose The current pose of the robot
     * @param targetPosition Translation2d with target x,y
     * @return Turret setpoint in radians
     */
    public static double calculate(Pose2d robotPose, Translation2d targetPosition) {
        // All the math from above goes here
    }
}
```

### TargetSelecter Class
**Purpose:** Decides which target to aim at based on robot position

```java
public class TargetSelecter {
    /**
     * @param robotPose The current pose of the robot
     * @return Translation2d with x,y position of selected target
     */
    public static Translation2d selectTarget(Pose2d robotPose) {
        // Logic to choose between hub or pass points
        // Based on which zone the robot is in
    }
}
```

### TurretSubsystem
**Purpose:** Hardware abstraction

```java
public class TurretSubsystem extends SubsystemBase {
    // Motor controller setup
    // Encoder setup
    // AdvantageScope logging infrastructure

    public void setPosition(double setpointRadians) { ... }
    public double getPosition() { ... }
    public boolean atSetpoint() { ... }
}
```

### AimTurretCommand (Default Command)
**Purpose:** Execute loop that ties everything together

```java
@Override
public void execute() {
    // 1. Get target position from TargetSelecter
    Translation2d target = TargetSelecter.selectTarget(poseSupplier.get());

    // 2. Calculate turret setpoint
    double setpoint = CalculateAimSetpoint.calculate(poseSupplier.get(), target);

    // 3. Command the turret
    turretSubsystem.setPosition(setpoint);
}
```

---

# Shooter Subsystem

## Development Progression

### Phase 1: Stationary Shooting (No Hood)
- Fixed shooter angle
- Variable wheel speed based on distance
- Robot must be stopped to shoot

### Phase 2: Stationary Shooting with Moving Hood
- Adjustable hood angle
- Create lookup table: distance → (wheel speed, hood angle)
- Still requires robot to be stopped

### Phase 3: Shooting from Anywhere
- Use odometry to determine distance
- Interpolate between known good shots
- Account for different field positions

### Phase 4: Shooting While Moving
- Lead the target based on robot velocity
- Adjust for ball flight time
- Most complex - requires velocity compensation

---

# Climber Subsystem

## Automation Goals
- Reduce driver cognitive load during endgame
- Sequence-based climbing with single button activation
- Safety interlocks to prevent damage

## Implementation Notes
*(To be filled in after mentor provides detailed plan)*

---

# Autonomous Paths

## Planning Approach
*(To be filled in after mentor provides detailed plan)*

---

# Learning Resources

For students who want to understand these concepts better, try asking AI tools questions like:
- "What is a radian and why do we use radians over degrees in FRC robotics?"
- "What is the angular modulus method and why use -π to +π instead of 0° to 360°?"
- "How do vectors work for calculating aim angles in robotics?"
- "Explain atan2 and why it's better than atan for robotics"

The mentor will also cover these topics in person during team meetings.
