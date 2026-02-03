# Robot Subsystems - Design & Implementation Guide

> **Purpose**: This document maps out exactly how we want everything programmed so that all team members understand the code structure and logic. If something goes sideways in any file, we'll know where it is and what kind of logic should be there.

---

## Related Documentation

- [Turret Strategy (Google Doc)](https://docs.google.com/document/d/1WeEUyfwCS4K-zV9DrKdq06biDVZyRlgV0zLkby5eSyU/edit?tab=t.0#heading=h.8uu09bemwrf8) - Original mentor planning document
- [Firmware Updates](01-updating-firmware-to-latest-frc-versions.md) - Pre-season hardware setup
- [AprilTag Vision](02-reading-april-tags.md) - Vision localization with PhotonVision
- [Virtual Field Setup](03-setup-virtual-game-field.md) - Simulation and testing

---

## WPILib 2026 Notes

Key changes affecting our code:

| Change | Impact |
|--------|--------|
| `Subsystem.idle()` | New method for default subsystem behavior |
| `Command.schedule()` deprecated | Use `CommandScheduler.getInstance().schedule(cmd)` instead |
| 2D `MathUtil.applyDeadband()` | Better circular joystick deadband handling |
| Shuffleboard deprecated | Use AdvantageScope, Glass, or Elastic |
| PathWeaver deprecated | Use Choreo or PathPlanner |

---

# Hardware Inventory

## Current Robot State (2026-02-02)

The robot is currently assembled and undergoing maintenance.

> **Note:** All photos below were taken with the robot **on its side** while installing the underbelly. Images are not oriented to normal upright position.

### Robot Overview Photos

![Robot Full View](photos/subsystems/robot-full-view.jpeg)
*Robot on side - shows intake roller (blue), shooter wheels (green/black), and internal electronics*

![Robot Side Overview](photos/subsystems/robot-side-overview.jpeg)
*Robot on side - shooter assembly with hood, intake system, and chassis visible*

![Robot Underside](photos/subsystems/robot-underside-electronics.jpeg)
*Robot on side - battery, electronics panel, and wiring (underbelly being installed)*

### Turret Assembly Photos

![Turret Motor Closeup](photos/subsystems/turret-motor-closeup.jpeg)
*Robot on side - Turret motor (NEO) with through-bore encoder on polycarbonate base*

![Turret Mount](photos/subsystems/turret-mount-angle.jpeg)
*Robot on side - Turret mounting assembly, motor position and encoder wiring*

---

## Motor & Sensor Inventory

| Subsystem | Component | Motor Type | Quantity | Encoder |
|-----------|-----------|------------|----------|---------|
| **Chassis** | Swerve Drive | NEO | 8 | Integrated |
| **Intake** | Rotator | NEO | 1 | External Encoder |
| **Intake** | Rollers | NEO | 1 | - |
| **Indexer** | Feed | NEO | 1 | - |
| **Kicker** | Feed to Shooter | NEO Vortex | 1 | - |
| **Shooter** | Turret Rotation | NEO | 1 | Through-Bore Encoder |
| **Shooter** | Flywheel | NEO Vortex | 2 | Integrated |
| **Shooter** | Flywheel | NEO | 1 | Integrated |
| **Shooter** | Hood | NEO | 1 | - |
| **Climber** | Extend/Retract | NEO Vortex | 1 | - |

### Motor Summary

| Motor Type | Count | Notes |
|------------|-------|-------|
| **NEO (REV-21-1650)** | 14 | Brushless, integrated encoder |
| **NEO Vortex (REV-21-1652)** | 4 | Higher power, integrated encoder |
| **Total Motors** | 18 | |

### Encoder Summary

| Encoder Type | Count | Location |
|--------------|-------|----------|
| Through-Bore Encoder | 1 | Shooter Turret |
| External Encoder | 1 | Intake Rotator |
| Integrated (NEO/Vortex) | 18 | All motors |

---

## CAN Bus Device IDs

*TODO: Document actual CAN IDs once finalized*

| Device | CAN ID | Notes |
|--------|--------|-------|
| Swerve FL Drive | TBD | |
| Swerve FL Steer | TBD | |
| Swerve FR Drive | TBD | |
| Swerve FR Steer | TBD | |
| Swerve BL Drive | TBD | |
| Swerve BL Steer | TBD | |
| Swerve BR Drive | TBD | |
| Swerve BR Steer | TBD | |
| Intake Rotator | TBD | |
| Intake Rollers | TBD | |
| Indexer | TBD | |
| Kicker | TBD | |
| Turret | TBD | |
| Shooter Flywheel 1 | TBD | Leader |
| Shooter Flywheel 2 | TBD | Follower |
| Shooter Flywheel 3 | TBD | |
| Shooter Hood | TBD | |
| Climber | TBD | |

---

## Complex Problems to Solve (Priority Order)

1. **Turret** - Auto-aiming system
2. **Shooter** - Hood control, stationary shooting, shooting from anywhere, shooting while moving
3. **Climber** - Automation
4. **Autonomous Paths** - Path planning and execution
5. **Vision Integration** - AprilTag localization (see [02-reading-april-tags.md](02-reading-april-tags.md))

---

# Turret Subsystem

## Implementation Todo List

### Phase 1: Hardware Foundation
- [ ] Create `TurretSubsystem.java` with motor and encoder setup
- [ ] Configure PID controller for position control
- [ ] Add soft limits to prevent over-rotation
- [ ] Implement AdvantageScope logging
- [ ] Test motor responds to manual commands

### Phase 2: Aim Calculation
- [ ] Create `TurretConstants.java` with field positions (hub, pass points)
- [ ] Create `CalculateAimSetpoint.java` with vector math
- [ ] Create `TargetSelecter.java` for target selection logic
- [ ] Unit test aim calculation with known positions
- [ ] Verify math with AdvantageScope visualization

### Phase 3: Default Command
- [ ] Create `AimTurretCommand.java`
- [ ] Wire up pose supplier from drive subsystem
- [ ] Set as default command in RobotContainer
- [ ] Test "always pointing" behavior
- [ ] Add target override for driver control

### Phase 4: Integration
- [ ] Coordinate with shooter for "ready to fire" state
- [ ] Add vision-based target refinement (if applicable)
- [ ] Tune for competition performance

---

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

### Folder Structure

```
frc/robot/
├── Constants.java              # All robot constants
├── RobotContainer.java         # Subsystem instantiation, command binding
└── subsystems/
    └── turret/
        ├── TurretSubsystem.java
        ├── TurretConstants.java
        ├── CalculateAimSetpoint.java
        ├── TargetSelecter.java
        └── commands/
            └── AimTurretCommand.java
```

---

## Implementation Notes

> **Pattern Note:** All subsystems follow the IO interface pattern from our chassis bot for simulation support and AdvantageKit logging.

### Turret Folder Structure
```
subsystems/turret/
├── TurretConstants.java       # Configuration constants
├── TurretIO.java              # Interface + @AutoLog inputs
├── TurretIOReal.java          # SparkMax + Through-Bore hardware
├── TurretIOSim.java           # Simulation implementation
├── TurretSubsystem.java       # Pure logic, receives IO
├── CalculateAimSetpoint.java  # Pure math (unchanged)
├── TargetSelecter.java        # Target selection (unchanged)
└── commands/
    └── AimTurretCommand.java
```

### Key Implementation Details

**TurretConstants.java** should include:
- Hardware IDs (motor CAN ID, encoder DIO port)
- Physical configuration (gear ratio, position conversion)
- Soft limits for rotation range
- PID gains for position control
- Field positions for targets (hub, pass points)

**TurretIO.java** interface should define:
- Input class with position, velocity, voltage, current, temperature
- Methods: `updateInputs()`, `setPosition()`, `setVoltage()`, `stop()`

**TurretSubsystem.java** should:
- Take TurretIO in constructor (dependency injection)
- Log inputs and outputs to AdvantageScope
- Provide `setPosition()`, `getPosition()`, `atSetpoint()`, `stop()` methods

**CalculateAimSetpoint.java** should:
- Be a pure math class with no hardware dependencies
- Implement the 5-step calculation described above
- Include distance calculation for shooter speed

**AimTurretCommand.java** should:
- Run forever as default command
- Get pose from drive subsystem via supplier
- Support target override for manual control

---

# Shooter Subsystem

## Implementation Todo List

### Phase 1: Basic Shooter
- [ ] Create `ShooterSubsystem.java` with flywheel motor(s)
- [ ] Implement velocity control (PID or feedforward)
- [ ] Add "at speed" detection for shooter readiness
- [ ] Test fixed-speed shooting at known distance
- [ ] Log flywheel RPM to AdvantageScope

### Phase 2: Hood Control (if applicable)
- [ ] Create `HoodSubsystem.java` or add to ShooterSubsystem
- [ ] Implement position control for hood angle
- [ ] Define hood presets (close, mid, far)
- [ ] Test hood movement and position accuracy

### Phase 3: Distance-Based Shooting
- [ ] Create `ShooterConstants.java` with lookup tables
- [ ] Implement `InterpolatingTreeMap` for distance → (speed, angle)
- [ ] Integrate with turret distance calculation
- [ ] Tune at multiple distances and populate lookup table

### Phase 4: Shooting While Moving
- [ ] Calculate target lead based on robot velocity
- [ ] Adjust for ball flight time
- [ ] Test and tune velocity compensation
- [ ] Add enable/disable toggle for moving shots

---

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

## Implementation Notes

### Shooter Folder Structure
```
subsystems/shooter/
├── ShooterConstants.java      # Configuration constants
├── ShooterIO.java             # Interface + @AutoLog inputs
├── ShooterIOReal.java         # SparkFlex/SparkMax hardware
├── ShooterIOSim.java          # Simulation implementation
└── ShooterSubsystem.java      # Pure logic, receives IO
```

### Key Implementation Details

**ShooterConstants.java** should include:
- Hardware IDs for flywheel motors (2x Vortex + 1x NEO) and hood motor
- Current limits for each motor type
- PID/feedforward gains for velocity control
- Hood gear ratio and position limits
- Shot table: distance → (RPM, hood angle) mappings

**ShooterIO.java** interface should define:
- Flywheel inputs: velocity, voltage, current, temperature
- Hood inputs: position, velocity, voltage, current
- Methods: `setFlywheelVelocity()`, `setHoodPosition()`, `stop()`

**ShooterSubsystem.java** should:
- Use `InterpolatingDoubleTreeMap` for distance-based shooting
- Provide `setForDistance()` method that sets both flywheel and hood
- Include `readyToShoot()` check combining flywheel speed and hood position

---

# Intake Subsystem

## Implementation Todo List

### Phase 1: Basic Intake
- [ ] Create `IntakeSubsystem.java` with rotator and roller motors
- [ ] Implement deploy/retract methods for rotator
- [ ] Implement run/stop methods for rollers
- [ ] Add encoder feedback for rotator position
- [ ] Test intake sequence

### Phase 2: Automation
- [ ] Create `IntakeCommand.java` for automatic game piece acquisition
- [ ] Add game piece detection (sensor or current sensing)
- [ ] Integrate with indexer handoff

---

## Hardware

| Component | Motor | Encoder |
|-----------|-------|---------|
| Rotator | 1x NEO | External Encoder |
| Rollers | 1x NEO | - |

## Implementation Notes

### Intake Folder Structure
```
subsystems/intake/
├── IntakeConstants.java       # Configuration constants
├── IntakeIO.java              # Interface + @AutoLog inputs
├── IntakeIOReal.java          # SparkMax hardware
├── IntakeIOSim.java           # Simulation implementation
└── IntakeSubsystem.java       # Pure logic, receives IO
```

### Key Implementation Details

**IntakeConstants.java** should include:
- Hardware IDs for rotator and roller motors
- External encoder DIO port
- Gear ratio and position conversion
- Deploy/retract positions in radians
- Roller speeds for intake/eject

**IntakeSubsystem.java** should provide:
- `deploy()` and `retract()` for rotator position
- `intake()` and `eject()` for roller control
- `isDeployed()` and `isRetracted()` position checks

---

# Indexer & Kicker Subsystem

## Implementation Todo List

### Phase 1: Basic Indexer
- [ ] Create `IndexerSubsystem.java` with indexer and kicker motors
- [ ] Implement feed methods
- [ ] Add game piece detection sensors
- [ ] Test indexing sequence

### Phase 2: Shooter Integration
- [ ] Create `FeedCommand.java` for automated feeding to shooter
- [ ] Coordinate with shooter "ready" state
- [ ] Add kicker timing for consistent shots

---

## Hardware

| Component | Motor |
|-----------|-------|
| Indexer | 1x NEO |
| Kicker | 1x NEO Vortex |

## Implementation Notes

### Indexer Folder Structure
```
subsystems/indexer/
├── IndexerConstants.java       # Configuration constants
├── IndexerIO.java              # Interface + @AutoLog inputs
├── IndexerIOReal.java          # SparkMax/SparkFlex hardware
├── IndexerIOSim.java           # Simulation implementation
└── IndexerSubsystem.java       # Pure logic, receives IO
```

### Key Implementation Details

**IndexerConstants.java** should include:
- CAN IDs for indexer (NEO) and kicker (Vortex) motors
- Game piece sensor DIO port
- Motor speeds for feed/reverse operations
- Kicker timing delay

**IndexerSubsystem.java** should provide:
- `feed()` and `reverse()` for indexer
- `kick()` for launching into shooter
- `hasGamePiece()` sensor check

---

# Climber Subsystem

## Implementation Todo List

### Phase 1: Basic Movement
- [ ] Create `ClimberSubsystem.java` with motor(s)
- [ ] Implement extend/retract methods
- [ ] Add limit switches or position limits
- [ ] Manual control for testing
- [ ] Log position to AdvantageScope

### Phase 2: Position Control
- [ ] Add encoder feedback
- [ ] Implement position presets (stowed, extended, climbing)
- [ ] Test position accuracy and repeatability

### Phase 3: Automated Sequence
- [ ] Create `ClimbSequenceCommand.java`
- [ ] Define state machine for climb stages
- [ ] Add safety interlocks (e.g., can't extend while driving fast)
- [ ] Test full sequence with robot on practice climb bars

### Phase 4: Driver Integration
- [ ] Single-button climb initiation
- [ ] Emergency stop override
- [ ] Status indication on dashboard

---

## Automation Goals
- Reduce driver cognitive load during endgame
- Sequence-based climbing with single button activation
- Safety interlocks to prevent damage

---

## Implementation Notes

### Climber Folder Structure
```
subsystems/climber/
├── ClimberConstants.java       # Configuration constants
├── ClimberIO.java              # Interface + @AutoLog inputs
├── ClimberIOReal.java          # SparkFlex hardware
├── ClimberIOSim.java           # Simulation implementation
├── ClimberSubsystem.java       # Pure logic, receives IO
└── commands/
    └── ClimbSequenceCommand.java
```

### Key Implementation Details

**ClimberConstants.java** should include:
- CAN ID for climber motor (NEO Vortex)
- High current limit for climbing load
- Drum diameter for position conversion
- Position presets (stowed, extended)
- Soft limits for safety

**ClimberSubsystem.java** should:
- Track state (STOWED, EXTENDING, EXTENDED, RETRACTING, CLIMBED)
- Provide `extend()` and `retract()` position control
- Provide `manualExtend()` and `manualRetract()` for testing
- Include `getCurrentAmps()` for load detection

**ClimbSequenceCommand.java** should:
- Implement state machine: EXTENDING → WAIT_FOR_HOOK → RETRACTING → COMPLETE
- Run as single-button automated climb
- Include hook detection or driver confirmation between stages

---

# Autonomous Paths

## Implementation Todo List

### Phase 1: Infrastructure
- [ ] Install PathPlanner or Choreo
- [ ] Configure swerve/drive kinematics
- [ ] Create test paths in path editor
- [ ] Verify path following with simple path

### Phase 2: Game Paths
- [ ] Design auto routines for 2026 REBUILT game
- [ ] Create named commands for game actions
- [ ] Build path groups for different starting positions
- [ ] Test paths in simulation

### Phase 3: Integration
- [ ] Add auto selector to dashboard
- [ ] Integrate with vision for initial pose
- [ ] Test on practice field
- [ ] Tune PID and feedforward values

---

## Planning Approach

**Recommended Tools (2026):**
- **Choreo** - Trajectory optimization with physics constraints
- **PathPlanner** - User-friendly path creation with auto builder

*Note: PathWeaver is deprecated as of 2026*

---

## Implementation Notes

### AutoBuilder Configuration

For PathPlanner, configure `AutoBuilder.configureHolonomic()` in your drive subsystem with:
- Pose supplier and reset consumer
- ChassisSpeeds supplier and consumer
- Translation and rotation PID constants
- Max module speed and drive base radius
- Alliance flip supplier for red/blue paths

### Auto Selector

Use `AutoBuilder.buildAutoChooser()` to create a `SendableChooser<Command>` that automatically populates with all PathPlanner autos.

---

# Vision Integration

See [02-reading-april-tags.md](02-reading-april-tags.md) for full vision setup.

## Implementation Todo List

- [ ] PhotonVision co-processor flashed (see [01-updating-firmware-to-latest-frc-versions.md](01-updating-firmware-to-latest-frc-versions.md))
- [ ] Camera(s) mounted and calibrated
- [ ] PhotonVision configured with correct camera transforms
- [ ] Pose estimator consuming vision measurements
- [ ] AdvantageScope visualization working
- [ ] Vision data integrated with turret aiming

---

# Learning Resources

For students who want to understand these concepts better, try asking AI tools questions like:
- "What is a radian and why do we use radians over degrees in FRC robotics?"
- "What is the angular modulus method and why use -π to +π instead of 0° to 360°?"
- "How do vectors work for calculating aim angles in robotics?"
- "Explain atan2 and why it's better than atan for robotics"

The mentor will also cover these topics in person during team meetings.

---

## Change Log

| Date | Author | Changes |
|------|--------|---------|
| 2026-02-02 | BetaWolves | Initial documentation - Turret strategy and math |
| 2026-02-02 | BetaWolves | Added todo lists, Java code stubs, WPILib 2026 notes |
| 2026-02-02 | BetaWolves | Added hardware inventory (18 motors), robot photos, Intake/Indexer/Kicker stubs |
| 2026-02-02 | BetaWolves | Removed full code implementations - students will learn by programming |
