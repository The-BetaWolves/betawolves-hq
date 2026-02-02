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

## Java Code Stubs

### TurretConstants.java

```java
package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
    // Hardware: 1x NEO motor + 1x Through-Bore Encoder
    // See: photos/subsystems/turret-motor-closeup.jpeg
    public static final int TURRET_MOTOR_ID = 20;  // NEO (SPARK MAX)
    public static final int TURRET_ENCODER_DIO = 0; // Through-Bore Encoder DIO port

    // Physical limits (radians from center)
    public static final double MAX_ROTATION_RAD = Units.degreesToRadians(270);
    public static final double MIN_ROTATION_RAD = Units.degreesToRadians(-270);

    // PID gains
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Position tolerance
    public static final double POSITION_TOLERANCE_RAD = Units.degreesToRadians(2);

    // Field positions (meters) - BLUE ALLIANCE
    // TODO: Update these for 2026 REBUILT field
    public static final Translation2d BLUE_HUB = new Translation2d(0.0, 0.0);
    public static final Translation2d BLUE_PASS_POINT_1 = new Translation2d(0.0, 0.0);
    public static final Translation2d BLUE_PASS_POINT_2 = new Translation2d(0.0, 0.0);

    // RED ALLIANCE (mirrored)
    public static final Translation2d RED_HUB = new Translation2d(0.0, 0.0);
    public static final Translation2d RED_PASS_POINT_1 = new Translation2d(0.0, 0.0);
    public static final Translation2d RED_PASS_POINT_2 = new Translation2d(0.0, 0.0);
}
```

### TurretSubsystem.java

```java
package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// TODO: Import your motor controller library (REV, CTRE, etc.)

public class TurretSubsystem extends SubsystemBase {
    // Hardware
    // private final CANSparkMax turretMotor;  // REV example
    // private final TalonFX turretMotor;       // CTRE example

    // Control
    private final PIDController pidController;
    private double setpointRadians = 0.0;

    // Logging (AdvantageScope compatible)
    private final DoublePublisher positionPub;
    private final DoublePublisher setpointPub;
    private final DoublePublisher errorPub;

    public TurretSubsystem() {
        // Initialize motor controller
        // turretMotor = new CANSparkMax(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
        // turretMotor.setIdleMode(IdleMode.kBrake);

        // Initialize PID
        pidController = new PIDController(
            TurretConstants.kP,
            TurretConstants.kI,
            TurretConstants.kD
        );
        pidController.setTolerance(TurretConstants.POSITION_TOLERANCE_RAD);
        pidController.enableContinuousInput(-Math.PI, Math.PI);

        // Initialize logging
        var nt = NetworkTableInstance.getDefault();
        positionPub = nt.getDoubleTopic("Turret/Position").publish();
        setpointPub = nt.getDoubleTopic("Turret/Setpoint").publish();
        errorPub = nt.getDoubleTopic("Turret/Error").publish();
    }

    @Override
    public void periodic() {
        // Run PID control
        double currentPosition = getPosition();
        double output = pidController.calculate(currentPosition, setpointRadians);

        // Apply output to motor
        // turretMotor.set(output);

        // Log data
        positionPub.set(currentPosition);
        setpointPub.set(setpointRadians);
        errorPub.set(setpointRadians - currentPosition);
    }

    /**
     * Set the turret position setpoint.
     * @param radians Target position in radians (clamped to limits)
     */
    public void setPosition(double radians) {
        this.setpointRadians = MathUtil.clamp(
            radians,
            TurretConstants.MIN_ROTATION_RAD,
            TurretConstants.MAX_ROTATION_RAD
        );
    }

    /**
     * Get the current turret position.
     * @return Position in radians
     */
    public double getPosition() {
        // TODO: Return encoder position in radians
        // return turretMotor.getEncoder().getPosition() * CONVERSION_FACTOR;
        return 0.0;
    }

    /**
     * Check if turret is at setpoint.
     * @return true if within tolerance
     */
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    /**
     * Stop the turret motor.
     */
    public void stop() {
        // turretMotor.set(0);
    }

    /**
     * WPILib 2026: Define default idle behavior
     */
    @Override
    public Command idle() {
        return run(this::stop);
    }
}
```

### CalculateAimSetpoint.java

```java
package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Pure math class - calculates turret aim setpoint.
 * No hardware dependencies, easy to unit test.
 */
public class CalculateAimSetpoint {

    /**
     * Calculate turret setpoint to aim at a target.
     *
     * @param robotPose Current robot pose (x, y, heading)
     * @param targetPosition Target position on field (x, y)
     * @return Turret setpoint in radians (turret-relative)
     */
    public static double calculate(Pose2d robotPose, Translation2d targetPosition) {
        // Step 1: Get vector from robot to target (field relative)
        double dx = targetPosition.getX() - robotPose.getX();
        double dy = targetPosition.getY() - robotPose.getY();

        // Step 2: Calculate angle to target (field relative)
        double targetFieldAngle = Math.atan2(dy, dx);

        // Step 3: Convert to robot relative by subtracting robot heading
        double robotHeading = robotPose.getRotation().getRadians();
        double aimRobotRelative = MathUtil.angleModulus(targetFieldAngle - robotHeading);

        // Step 4: Convert to turret relative (turret zeroed to back)
        double aimTurretRelative = MathUtil.angleModulus(aimRobotRelative - Math.PI);

        // Step 5: Clamp to turret limits
        double turretSetpoint = MathUtil.clamp(
            aimTurretRelative,
            TurretConstants.MIN_ROTATION_RAD,
            TurretConstants.MAX_ROTATION_RAD
        );

        return turretSetpoint;
    }

    /**
     * Calculate distance to target (useful for shooter speed).
     *
     * @param robotPose Current robot pose
     * @param targetPosition Target position
     * @return Distance in meters
     */
    public static double distanceToTarget(Pose2d robotPose, Translation2d targetPosition) {
        return robotPose.getTranslation().getDistance(targetPosition);
    }
}
```

### TargetSelecter.java

```java
package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Selects which target to aim at based on robot position.
 */
public class TargetSelecter {

    public enum TargetType {
        HUB,
        PASS_POINT_1,
        PASS_POINT_2
    }

    /**
     * Select the best target based on robot position.
     *
     * @param robotPose Current robot pose
     * @return Translation2d of the selected target
     */
    public static Translation2d selectTarget(Pose2d robotPose) {
        // Default to hub
        return getHubPosition();
    }

    /**
     * Select a specific target type.
     *
     * @param targetType The type of target to get
     * @return Translation2d of the target
     */
    public static Translation2d getTarget(TargetType targetType) {
        boolean isRed = DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red;

        return switch (targetType) {
            case HUB -> isRed ? TurretConstants.RED_HUB : TurretConstants.BLUE_HUB;
            case PASS_POINT_1 -> isRed ? TurretConstants.RED_PASS_POINT_1 : TurretConstants.BLUE_PASS_POINT_1;
            case PASS_POINT_2 -> isRed ? TurretConstants.RED_PASS_POINT_2 : TurretConstants.BLUE_PASS_POINT_2;
        };
    }

    /**
     * Get the hub position for the current alliance.
     */
    public static Translation2d getHubPosition() {
        return getTarget(TargetType.HUB);
    }

    /**
     * Advanced: Select target based on field zones.
     * TODO: Implement zone-based target selection
     */
    public static Translation2d selectTargetByZone(Pose2d robotPose) {
        // Example logic:
        // - If in offensive zone, aim at hub
        // - If in defensive zone, aim at pass point
        // - If near specific field elements, adjust target

        double x = robotPose.getX();
        double y = robotPose.getY();

        // TODO: Define zones based on 2026 REBUILT field layout
        // For now, always return hub
        return getHubPosition();
    }
}
```

### AimTurretCommand.java

```java
package frc.robot.subsystems.turret.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.CalculateAimSetpoint;
import frc.robot.subsystems.turret.TargetSelecter;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Default command that continuously aims the turret at a target.
 * Runs forever - designed to be set as the turret's default command.
 */
public class AimTurretCommand extends Command {
    private final TurretSubsystem turret;
    private final Supplier<Pose2d> poseSupplier;
    private Supplier<Translation2d> targetOverride = null;

    /**
     * Create an aim command with automatic target selection.
     *
     * @param turret The turret subsystem
     * @param poseSupplier Supplier for robot pose (typically drive::getPose)
     */
    public AimTurretCommand(TurretSubsystem turret, Supplier<Pose2d> poseSupplier) {
        this.turret = turret;
        this.poseSupplier = poseSupplier;
        addRequirements(turret);
    }

    /**
     * Create an aim command with a fixed target.
     *
     * @param turret The turret subsystem
     * @param poseSupplier Supplier for robot pose
     * @param targetSupplier Supplier for target position
     */
    public AimTurretCommand(
            TurretSubsystem turret,
            Supplier<Pose2d> poseSupplier,
            Supplier<Translation2d> targetSupplier) {
        this(turret, poseSupplier);
        this.targetOverride = targetSupplier;
    }

    @Override
    public void execute() {
        // 1. Get current robot pose
        Pose2d pose = poseSupplier.get();

        // 2. Get target position (override or auto-select)
        Translation2d target;
        if (targetOverride != null) {
            target = targetOverride.get();
        } else {
            target = TargetSelecter.selectTarget(pose);
        }

        // 3. Calculate turret setpoint
        double setpoint = CalculateAimSetpoint.calculate(pose, target);

        // 4. Command the turret
        turret.setPosition(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        // Never finishes - runs as default command
        return false;
    }
}
```

### RobotContainer.java (Turret Wiring)

```java
// In RobotContainer.java

import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.commands.AimTurretCommand;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem drive = new DriveSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        // Turret: ALWAYS be pointing at target
        turret.setDefaultCommand(
            new AimTurretCommand(turret, drive::getPose)
        );
    }

    private void configureButtonBindings() {
        // Example: Override to aim at pass point while button held
        // controller.leftBumper().whileTrue(
        //     new AimTurretCommand(turret, drive::getPose,
        //         () -> TargetSelecter.getTarget(TargetType.PASS_POINT_1))
        // );
    }
}
```

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

## Java Code Stubs

### ShooterConstants.java

```java
package frc.robot.subsystems.shooter;

public final class ShooterConstants {
    // Hardware: 2x NEO Vortex + 1x NEO for flywheel, 1x NEO for hood
    // See: photos/subsystems/robot-side-overview.jpeg (green/black wheels)
    public static final int FLYWHEEL_VORTEX_1_ID = 30;  // NEO Vortex (SPARK Flex)
    public static final int FLYWHEEL_VORTEX_2_ID = 31;  // NEO Vortex (SPARK Flex)
    public static final int FLYWHEEL_NEO_ID = 32;       // NEO (SPARK MAX)
    public static final int HOOD_MOTOR_ID = 33;         // NEO (SPARK MAX)

    // Flywheel PID
    public static final double FLYWHEEL_kP = 0.0;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.0;
    public static final double FLYWHEEL_kS = 0.0; // Static feedforward
    public static final double FLYWHEEL_kV = 0.0; // Velocity feedforward

    // Flywheel tolerances
    public static final double FLYWHEEL_TOLERANCE_RPM = 50.0;

    // Hood limits
    public static final double HOOD_MIN_ANGLE_DEG = 20.0;
    public static final double HOOD_MAX_ANGLE_DEG = 60.0;

    // Shot presets (distance meters → RPM)
    // TODO: Populate from testing
    public static final double[][] SHOT_TABLE = {
        // {distance_m, flywheel_rpm, hood_angle_deg}
        {1.0, 2000.0, 25.0},
        {2.0, 2500.0, 35.0},
        {3.0, 3000.0, 45.0},
        {4.0, 3500.0, 55.0},
    };
}
```

### ShooterSubsystem.java

```java
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // Hardware
    // private final CANSparkMax flywheelLeader;
    // private final CANSparkMax flywheelFollower;

    // Control
    private final SimpleMotorFeedforward feedforward;
    private double targetRPM = 0.0;

    // Interpolation tables for distance-based shooting
    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

    // Logging
    private final DoublePublisher actualRPMPub;
    private final DoublePublisher targetRPMPub;

    public ShooterSubsystem() {
        // Initialize hardware
        // flywheelLeader = new CANSparkMax(...);
        // flywheelFollower = new CANSparkMax(...);
        // flywheelFollower.follow(flywheelLeader, true); // Inverted

        feedforward = new SimpleMotorFeedforward(
            ShooterConstants.FLYWHEEL_kS,
            ShooterConstants.FLYWHEEL_kV
        );

        // Populate interpolation tables from constants
        for (double[] row : ShooterConstants.SHOT_TABLE) {
            rpmTable.put(row[0], row[1]);
            hoodTable.put(row[0], row[2]);
        }

        // Logging
        var nt = NetworkTableInstance.getDefault();
        actualRPMPub = nt.getDoubleTopic("Shooter/ActualRPM").publish();
        targetRPMPub = nt.getDoubleTopic("Shooter/TargetRPM").publish();
    }

    @Override
    public void periodic() {
        actualRPMPub.set(getVelocityRPM());
        targetRPMPub.set(targetRPM);
    }

    /**
     * Set flywheel to a specific RPM.
     */
    public void setVelocity(double rpm) {
        this.targetRPM = rpm;
        // double ff = feedforward.calculate(rpm / 60.0); // Convert to RPS
        // flywheelLeader.setVoltage(ff);
    }

    /**
     * Set flywheel based on distance to target.
     */
    public void setVelocityForDistance(double distanceMeters) {
        double rpm = rpmTable.get(distanceMeters);
        setVelocity(rpm);
    }

    /**
     * Get recommended hood angle for distance.
     */
    public double getHoodAngleForDistance(double distanceMeters) {
        return hoodTable.get(distanceMeters);
    }

    /**
     * Get current flywheel velocity.
     */
    public double getVelocityRPM() {
        // return flywheelLeader.getEncoder().getVelocity();
        return 0.0;
    }

    /**
     * Check if flywheel is at target speed.
     */
    public boolean atTargetVelocity() {
        return Math.abs(getVelocityRPM() - targetRPM) < ShooterConstants.FLYWHEEL_TOLERANCE_RPM;
    }

    /**
     * Stop the shooter.
     */
    public void stop() {
        targetRPM = 0.0;
        // flywheelLeader.set(0);
    }

    @Override
    public Command idle() {
        return run(this::stop);
    }
}
```

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

## Java Code Stubs

### IntakeSubsystem.java

```java
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // Hardware: 1x NEO (rotator) + 1x NEO (rollers) + External Encoder
    public static final int ROTATOR_MOTOR_ID = 50;  // NEO (SPARK MAX)
    public static final int ROLLER_MOTOR_ID = 51;   // NEO (SPARK MAX)
    public static final int ROTATOR_ENCODER_DIO = 1; // External encoder DIO port

    // Hardware
    // private final CANSparkMax rotatorMotor;
    // private final CANSparkMax rollerMotor;

    public IntakeSubsystem() {
        // Initialize motors
    }

    public void deploy() {
        // Rotate intake to deployed position
    }

    public void retract() {
        // Rotate intake to retracted position
    }

    public void runRollers(double speed) {
        // Run intake rollers (positive = intake, negative = eject)
    }

    public void stopRollers() {
        // Stop rollers
    }

    public boolean isDeployed() {
        // Check encoder position
        return false;
    }

    @Override
    public Command idle() {
        return run(this::stopRollers);
    }
}
```

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

## Java Code Stubs

### IndexerSubsystem.java

```java
package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    // Hardware: 1x NEO (indexer) + 1x NEO Vortex (kicker)
    public static final int INDEXER_MOTOR_ID = 52;  // NEO (SPARK MAX)
    public static final int KICKER_MOTOR_ID = 53;   // NEO Vortex (SPARK Flex)

    // Hardware
    // private final CANSparkMax indexerMotor;
    // private final CANSparkFlex kickerMotor;

    public IndexerSubsystem() {
        // Initialize motors
    }

    public void runIndexer(double speed) {
        // Run indexer belt/wheels
    }

    public void stopIndexer() {
        // Stop indexer
    }

    public void kick() {
        // Run kicker to feed game piece to shooter
    }

    public void stopKicker() {
        // Stop kicker
    }

    public boolean hasGamePiece() {
        // Check sensor for game piece presence
        return false;
    }

    @Override
    public Command idle() {
        return run(() -> {
            stopIndexer();
            stopKicker();
        });
    }
}
```

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

## Java Code Stubs

### ClimberSubsystem.java

```java
package frc.robot.subsystems.climber;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    // Hardware: 1x NEO Vortex
    public static final int CLIMBER_MOTOR_ID = 40; // NEO Vortex (SPARK Flex)

    public enum ClimberState {
        STOWED,
        EXTENDING,
        EXTENDED,
        CLIMBING,
        CLIMBED
    }

    // Hardware
    // private final CANSparkFlex climberMotor; // Vortex uses SPARK Flex

    private ClimberState state = ClimberState.STOWED;

    // Logging
    private final DoublePublisher positionPub;

    public ClimberSubsystem() {
        // climberMotor = new CANSparkMax(...);

        var nt = NetworkTableInstance.getDefault();
        positionPub = nt.getDoubleTopic("Climber/Position").publish();
    }

    @Override
    public void periodic() {
        positionPub.set(getPosition());
    }

    public void extend() {
        state = ClimberState.EXTENDING;
        // climberMotor.set(1.0);
    }

    public void retract() {
        state = ClimberState.CLIMBING;
        // climberMotor.set(-1.0);
    }

    public void stop() {
        // climberMotor.set(0);
    }

    public double getPosition() {
        // return climberMotor.getEncoder().getPosition();
        return 0.0;
    }

    public ClimberState getState() {
        return state;
    }

    public boolean isExtended() {
        // TODO: Check position or limit switch
        return false;
    }

    public boolean isStowed() {
        // TODO: Check position or limit switch
        return false;
    }

    @Override
    public Command idle() {
        return run(this::stop);
    }
}
```

### ClimbSequenceCommand.java

```java
package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberState;

/**
 * Automated climbing sequence.
 * Single button press initiates full climb.
 */
public class ClimbSequenceCommand extends Command {
    private final ClimberSubsystem climber;
    private ClimbStage currentStage = ClimbStage.IDLE;

    private enum ClimbStage {
        IDLE,
        EXTENDING,
        WAIT_FOR_HOOK,
        RETRACTING,
        COMPLETE
    }

    public ClimbSequenceCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        currentStage = ClimbStage.EXTENDING;
    }

    @Override
    public void execute() {
        switch (currentStage) {
            case EXTENDING:
                climber.extend();
                if (climber.isExtended()) {
                    currentStage = ClimbStage.WAIT_FOR_HOOK;
                    climber.stop();
                }
                break;

            case WAIT_FOR_HOOK:
                // Wait for driver confirmation or sensor
                // TODO: Add hook detection or button press
                // For now, auto-advance after brief pause
                currentStage = ClimbStage.RETRACTING;
                break;

            case RETRACTING:
                climber.retract();
                if (climber.isStowed()) {
                    currentStage = ClimbStage.COMPLETE;
                    climber.stop();
                }
                break;

            case COMPLETE:
            case IDLE:
                climber.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return currentStage == ClimbStage.COMPLETE;
    }
}
```

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

## Java Code Stubs

### AutoBuilder Setup (PathPlanner)

```java
// In DriveSubsystem constructor or separate AutoConfig class

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(
        this::getPose,              // Pose supplier
        this::resetPose,            // Pose reset consumer
        this::getChassisSpeeds,     // ChassisSpeeds supplier
        this::driveRobotRelative,   // ChassisSpeeds consumer
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0),  // Translation PID
            new PIDConstants(5.0, 0.0, 0.0),  // Rotation PID
            4.5,                               // Max module speed m/s
            0.4,                               // Drive base radius m
            new ReplanningConfig()
        ),
        () -> {
            // Flip path for red alliance
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == Alliance.Red;
        },
        this                         // Subsystem requirement
    );
}
```

### Auto Selector

```java
// In RobotContainer

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

private final SendableChooser<Command> autoChooser;

public RobotContainer() {
    // Build auto chooser from PathPlanner paths
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
}

public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}
```

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
