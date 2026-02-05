# 2026 Chassis Bot - Architecture & Code Flow

This document walks through how the robot code is organized and how data flows
from the moment the robot boots up through teleop driving and autonomous routines.

---

## File Map

```
src/main/java/frc/robot/
│
├── Main.java                          ← JVM entry point (don't touch)
├── Robot.java                         ← Mode lifecycle (auto, teleop, disabled)
├── RobotContainer.java                ← Wires everything together
├── Constants.java                     ← All hardware IDs, speeds, PID values
│
├── commands/
│   ├── TeleopDriveCommand.java        ← Reads joystick → drives the robot
│   └── TestBangBang.java              ← Simple test auto (drive forward)
│
├── subsystems/
│   ├── swerve/
│   │   ├── SwerveDrive.java           ← Main drive subsystem
│   │   ├── SwerveModule/
│   │   │   ├── SwerveModule.java      ← One wheel module (×4)
│   │   │   ├── SwerveModuleIO.java    ← Interface for real/sim
│   │   │   ├── SwerveModuleIOReal.java← SparkMax + CANcoder hardware
│   │   │   ├── SwerveModuleIOSim.java ← Physics simulation
│   │   │   ├── SwerveModuleIOInputs.java ← Logged sensor data
│   │   │   └── SwerveModuleConstants.java ← Per-module config
│   │   ├── Gyro/
│   │   │   ├── GyroIO.java            ← Interface for heading sensor
│   │   │   ├── GyroPigeon.java        ← Real Pigeon2 IMU
│   │   │   └── GyroSim.java           ← Simulated heading
│   │   └── Odometry/
│   │       ├── OdometryIO.java        ← Interface for pose tracking
│   │       ├── OdometryReal.java      ← Real odometry
│   │       ├── OdometrySim.java       ← Simulated odometry
│   │       └── OdometryIOInputs.java  ← Logged pose data
│   │
│   └── vision/
│       ├── Vision.java                ← Filters camera data, sends to drive
│       ├── VisionConstants.java       ← Camera positions, field layout
│       ├── VisionIO.java              ← Interface + data records
│       ├── VisionIOPhotonVision.java  ← Real PhotonVision camera
│       ├── VisionIOLimelight.java     ← Real Limelight camera
│       └── VisionIOPhotonVisionSim.java ← Simulated camera
│
└── util/
    ├── SwerveModuleAngleOptimizer.java ← Minimizes wheel steering rotation
    └── OptimizeModuleState.java        ← (Duplicate of above)
```

---

## Startup Flow

This is what happens when the robot powers on:

```
JVM starts
  │
  ▼
Main.main()
  │  Just calls RobotBase.startRobot(Robot::new)
  │
  ▼
Robot() constructor
  │  Sets up AdvantageKit logging
  │  Creates RobotContainer
  │
  ▼
RobotContainer() constructor ◄── THIS IS WHERE EVERYTHING GETS BUILT
  │
  ├── Creates SwerveDrive subsystem
  │     ├── Picks real or sim Gyro
  │     ├── Picks real or sim Odometry
  │     ├── Creates 4 SwerveModules (FL, FR, BL, BR)
  │     │     └── Each picks real or sim IO
  │     └── Configures PathPlanner for auto paths
  │
  ├── Creates Vision subsystem
  │     ├── Picks real or sim camera IO (1-2 cameras)
  │     └── Loads custom field layout (Betawolves2026LabField.json)
  │
  ├── Sets TeleopDriveCommand as the default command for SwerveDrive
  │
  └── Loads PathPlanner auto chooser (for selecting autos on dashboard)
```

**Key idea:** `RobotContainer` is the "wiring diagram." It builds every
subsystem and connects them together. If you want to know what the robot
*has*, look here.

---

## The Main Loop (runs every 20ms)

Once the robot is initialized, WPILib's `CommandScheduler` runs a loop
every 20 milliseconds (50 times per second). Here is what happens each cycle:

```
Every 20ms, CommandScheduler.run() is called
  │
  ├─── 1. Run all COMMANDS ──────────────────────────────────┐
  │    (things the robot is actively doing)                   │
  │                                                           │
  │    During TELEOP:                                         │
  │      TeleopDriveCommand.execute()                         │
  │        ├── Read joystick X, Y, Twist                      │
  │        ├── Apply cubic curve (x³) for smooth control      │
  │        ├── Multiply by max speed (3.75 m/s)               │
  │        └── Call SwerveDrive.drive(vx, vy, omega)          │
  │                                                           │
  │    During AUTO:                                           │
  │      PathPlanner FollowPathCommand.execute()              │
  │        ├── Read current pose from SwerveDrive             │
  │        ├── Calculate speeds to follow the path            │
  │        └── Call SwerveDrive.driveRobotRelative(speeds)    │
  │                                                           │
  ├─── 2. Run all SUBSYSTEM periodic() methods ──────────────┤
  │                                                           │
  │    SwerveDrive.periodic()                                 │
  │      ├── Convert desired speeds → 4 wheel states          │
  │      ├── Send states to each SwerveModule                 │
  │      │     └── Each module → motor controllers            │
  │      ├── Read gyro + encoder positions                    │
  │      ├── Update pose estimator                            │
  │      └── Log everything                                   │
  │                                                           │
  │    Vision.periodic()                                      │
  │      ├── Read latest camera frames                        │
  │      ├── Detect AprilTags → estimate robot pose           │
  │      ├── Filter bad data (too ambiguous, out of bounds)   │
  │      └── Send good poses → SwerveDrive pose estimator     │
  │                                                           │
  └──────────────────────────────────────────────────────────┘
```

---

## Teleop Drive - Step by Step

This is the most important flow for understanding how driver input
becomes wheel motion:

```
 DRIVER                        COMMAND                         SUBSYSTEM                        HARDWARE
┌──────────┐            ┌─────────────────────┐         ┌──────────────────┐            ┌─────────────────┐
│          │  X, Y,     │ TeleopDriveCommand   │         │   SwerveDrive    │            │  4 Swerve       │
│ Joystick │──Twist───▶ │                     │         │                  │            │  Modules        │
│          │            │ 1. Read inputs       │         │                  │            │                 │
└──────────┘            │ 2. Cube them (x³)   │  drive()│                  │            │ ┌─────────────┐ │
                        │ 3. Scale to m/s     │────────▶│ 1. Store speeds  │            │ │ FL: Drive + │ │
                        │                     │         │                  │  setStates  │ │   Steer     │ │
                        └─────────────────────┘         │ 2. periodic():   │───────────▶│ ├─────────────┤ │
                                                        │    Kinematics    │            │ │ FR: Drive + │ │
                                                        │    converts to   │            │ │   Steer     │ │
                                                        │    4 module      │            │ ├─────────────┤ │
                                                        │    states        │            │ │ BL: Drive + │ │
                                                        │    (speed+angle) │            │ │   Steer     │ │
                                                        │                  │            │ ├─────────────┤ │
                                                        │ 3. Update pose   │            │ │ BR: Drive + │ │
                                                        │    estimator     │            │ │   Steer     │ │
                                                        └──────────────────┘            │ └─────────────┘ │
                                                                                        └─────────────────┘
```

**What "Kinematics" does:** Takes the robot's desired movement (forward,
sideways, rotation) and calculates what speed and angle each of the 4
wheels needs to be at. This is the math that makes swerve drive work.

---

## Vision + Pose Estimation

The robot knows where it is on the field by combining three data sources:

```
                  DATA SOURCE                         WHAT IT PROVIDES
               ┌──────────────┐
               │   Gyro       │──── Which direction the robot is facing
               │  (Pigeon2)   │     (yaw/heading)
               └──────┬───────┘
                      │
                      ▼
               ┌──────────────────────────┐
               │                          │
               │   SwerveDrivePose-       │     FUSED POSE
               │   Estimator              │──── "I am at (x, y) facing θ"
               │                          │     (best estimate, combining
               │   Kalman-filter style    │      all sources)
               │   fusion                 │
               │                          │
               └──────────────────────────┘
                      ▲              ▲
                      │              │
               ┌──────┴───────┐  ┌───┴───────────┐
               │  Odometry    │  │   Vision       │
               │ (Encoders)   │  │  (Cameras)     │
               │              │  │                │
               │ "How far     │  │ "I see AprilTag│
               │  each wheel  │  │  #5 at this    │
               │  has turned" │  │  angle/distance│
               │              │  │  so I must be  │
               └──────────────┘  │  HERE"         │
                                 └────────────────┘
```

### Vision Filtering (Vision.java)

Not all camera data is trustworthy. The Vision subsystem filters out bad readings:

```
Camera sees AprilTag(s)
  │
  ├── 0 tags seen? ──────────────────── REJECT (nothing to work with)
  │
  ├── 1 tag, ambiguity > 0.3? ──────── REJECT (pose is uncertain)
  │
  ├── Estimated Z height > 0.75m? ──── REJECT (robot isn't flying)
  │
  ├── Pose outside field boundaries?── REJECT (impossible position)
  │
  └── Passes all checks? ──────────── ACCEPT
        │
        ├── Calculate confidence (closer + more tags = higher confidence)
        └── Send to SwerveDrive.addVisionMeasurement()
```

---

## The Interface Pattern (Real vs. Simulation)

Every piece of hardware has an **interface** that defines *what* it does,
and then separate classes for *how* it does it on real hardware vs. in simulation.

```
                    ┌─────────────────────┐
                    │    Interface         │
                    │  (e.g. SwerveModule- │
                    │   IO.java)           │
                    │                      │
                    │  "Here's what a      │
                    │   module CAN DO"     │
                    └──────┬──────┬────────┘
                           │      │
              ┌────────────┘      └────────────┐
              ▼                                ▼
   ┌──────────────────────┐       ┌──────────────────────┐
   │  SwerveModuleIOReal  │       │  SwerveModuleIOSim   │
   │                      │       │                      │
   │  Talks to actual     │       │  Runs physics math   │
   │  SparkMax motors     │       │  to fake the motors  │
   │  and CANcoders       │       │                      │
   └──────────────────────┘       └──────────────────────┘
```

This pattern repeats for every hardware component:

| Interface | Real Implementation | Sim Implementation |
|-----------|--------------------|--------------------|
| `SwerveModuleIO` | `SwerveModuleIOReal` (SparkMax + CANcoder) | `SwerveModuleIOSim` |
| `GyroIO` | `GyroPigeon` (Pigeon2 IMU) | `GyroSim` |
| `OdometryIO` | `OdometryReal` | `OdometrySim` |
| `VisionIO` | `VisionIOPhotonVision` / `VisionIOLimelight` | `VisionIOPhotonVisionSim` |

**Why?** So the exact same robot logic runs on the real robot AND in the
desktop simulator. The code in `SwerveDrive.java` never knows or cares
which one it's talking to.

The selection happens in `RobotContainer` using `RobotBase.isSimulation()`.

---

## Subsystem & Command Relationship

WPILib's **Command-Based** framework is the backbone. Here's how it works:

```
┌─────────────────────────────────────────────────────────┐
│                    SUBSYSTEMS                            │
│          (things the robot HAS)                          │
│                                                          │
│   ┌──────────────┐          ┌──────────────┐            │
│   │  SwerveDrive │          │   Vision     │            │
│   │              │          │              │            │
│   │ • drive()    │◄─────────│ • Sends pose │            │
│   │ • getPose()  │  vision  │   corrections│            │
│   │ • periodic() │  data    │ • periodic() │            │
│   └──────────────┘          └──────────────┘            │
│          ▲                                               │
│          │ "I need SwerveDrive"                          │
│          │ (requirement)                                 │
│          │                                               │
│   ┌──────────────────────────────────────┐              │
│   │              COMMANDS                 │              │
│   │       (things the robot DOES)         │              │
│   │                                       │              │
│   │   ┌─────────────────────┐            │              │
│   │   │ TeleopDriveCommand  │ ← DEFAULT  │              │
│   │   │ (runs during teleop)│   COMMAND   │              │
│   │   └─────────────────────┘            │              │
│   │   ┌─────────────────────┐            │              │
│   │   │ TestBangBang        │            │              │
│   │   │ (test autonomous)   │            │              │
│   │   └─────────────────────┘            │              │
│   │   ┌─────────────────────┐            │              │
│   │   │ PathPlanner Auto    │            │              │
│   │   │ (generated paths)   │            │              │
│   │   └─────────────────────┘            │              │
│   └──────────────────────────────────────┘              │
└─────────────────────────────────────────────────────────┘
```

**Rule:** Only ONE command can use a subsystem at a time. If a new command
wants `SwerveDrive`, the old one gets canceled. The `TeleopDriveCommand`
is the *default* -- it runs whenever nothing else needs the drive.

---

## Hardware Wiring Reference

```
CAN Bus Layout
──────────────────────────────────────────────────────
 Device              │ CAN ID │ Type
─────────────────────┼────────┼──────────────────────
 Gyro (Pigeon2)      │   9    │ CTRE Pigeon2
─────────────────────┼────────┼──────────────────────
 FL Angle Motor      │  11    │ REV SparkMax
 FL Drive Motor      │  12    │ REV SparkMax
 FL Absolute Encoder │  13    │ CTRE CANcoder
─────────────────────┼────────┼──────────────────────
 FR Angle Motor      │  21    │ REV SparkMax
 FR Drive Motor      │  22    │ REV SparkMax
 FR Absolute Encoder │  23    │ CTRE CANcoder
─────────────────────┼────────┼──────────────────────
 BL Angle Motor      │  31    │ REV SparkMax
 BL Drive Motor      │  32    │ REV SparkMax
 BL Absolute Encoder │  33    │ CTRE CANcoder
─────────────────────┼────────┼──────────────────────
 BR Angle Motor      │  37    │ REV SparkMax
 BR Drive Motor      │  38    │ REV SparkMax
 BR Absolute Encoder │  39    │ CTRE CANcoder
──────────────────────────────────────────────────────

 Each swerve module = 1 drive motor + 1 steering motor + 1 absolute encoder
 Total: 8 motors + 4 encoders + 1 gyro = 13 CAN devices
```

---

## Key Numbers (from Constants.java)

| Parameter | Value | What it means |
|-----------|-------|---------------|
| Max speed | 3.75 m/s | ~8.4 mph top speed |
| Max rotation | 3.75 rad/s | ~215 deg/s turning |
| Track width | 0.576 m | Distance between left and right wheels |
| Wheel base | 0.576 m | Distance between front and back wheels |
| Wheel diameter | 0.1016 m | 4-inch wheels |
| Drive gear ratio | 6.75:1 | Motor spins 6.75x for 1 wheel rotation |
| Steer gear ratio | 21.43:1 | Motor spins 21.43x for 1 wheel rotation |
| Drive current limit | 40 A | Protects drive motors |
| Steer current limit | 20 A | Protects steering motors |

---

## "Where do I look to..."

| I want to... | Look at... |
|---|---|
| Change max drive speed | `Constants.java` → `MAX_VELOCITY` |
| Change joystick sensitivity | `TeleopDriveCommand.java` → the cubic curve (`Math.pow`) |
| Add a new motor/mechanism | Create a new Subsystem class, wire it in `RobotContainer` |
| Change swerve module encoder offsets | `Constants.java` → module constants (FL, FR, BL, BR) |
| Change PID tuning for drive motors | `Constants.java` → `SwerveMotorConfig` |
| Add a new autonomous routine | Create it in PathPlanner, it auto-loads via `autoChooser` |
| Change camera positions | `VisionConstants.java` → camera transforms |
| Change the custom field layout | `Betawolves2026LabField.json` (in deploy directory) |
| Understand how a joystick press becomes wheel motion | Read this doc's "Teleop Drive" section |
| Add a new button binding | `RobotContainer.java` → `configureButtonBindings()` |

---

## Glossary

| Term | Meaning |
|------|---------|
| **Swerve Drive** | A drivetrain where each wheel can independently steer AND drive. Allows the robot to move in any direction while facing any direction. |
| **Kinematics** | The math that converts "move forward at 2 m/s while rotating" into "wheel 1 at 30 deg going 1.8 m/s, wheel 2 at..." |
| **Odometry** | Tracking the robot's position by counting how far each wheel has turned. Drifts over time. |
| **Pose Estimator** | Combines odometry, gyro, and vision to get the best possible position estimate. |
| **AprilTag** | A printed barcode on the field. Cameras detect them to figure out where the robot is. |
| **Field-Relative** | Pushing the joystick "up" always drives toward the far end of the field, regardless of which way the robot is facing. |
| **Robot-Relative** | Pushing the joystick "up" drives whichever direction the robot's front is pointing. |
| **CAN Bus** | The wiring network that connects the RoboRIO to all motors, encoders, and the gyro. |
| **SparkMax** | REV motor controller that drives the NEO brushless motors. |
| **CANcoder** | CTRE absolute encoder that always knows the exact angle of the steering wheel, even after power cycling. |
| **Pigeon2** | CTRE IMU (gyroscope + accelerometer) that tracks the robot's heading. |
| **PathPlanner** | Software tool for drawing autonomous paths on a field map. Generates commands that the robot follows. |
| **AdvantageKit** | Logging framework that records all sensor data so you can replay and debug matches after the fact. |
| **IO Interface** | The abstraction layer that lets the same code run on real hardware or in simulation. |
| **Command** | An action the robot performs (e.g., "drive with joystick"). Only one command per subsystem at a time. |
| **Subsystem** | A capability the robot has (e.g., the drivetrain). Manages hardware and provides methods for commands to call. |
