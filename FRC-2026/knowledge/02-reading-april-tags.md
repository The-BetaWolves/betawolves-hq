# Reading AprilTags - FRC 2026 Vision Setup

This document covers our AprilTag vision implementation for the FRC 2026 season, including code setup, testing methodology, and visualization using AdvantageScope.

---

## Overview

We successfully configured our robot to read AprilTags and estimate field position using PhotonVision. The robot can detect AprilTags placed around the field and use them for localization during autonomous and teleop periods.

**Special thanks to Eric** for his help getting the AprilTag code working!

---

## Code Repository

### Source Repository

Our 2026 chassis bot code was created from our FRC 2025 repository:

- **Original 2025 Repo:** https://github.com/The-BetaWolves/FRC2025
- **2026 Chassis Bot Repo:** https://github.com/The-BetaWolves/2026chassisbot

### Working Commit

The AprilTag vision code is confirmed working as of this commit:

```
Commit: 46bf8b2abbeade7e5baf1d7a9b3c634889b7ccec
```

**Direct link:** https://github.com/The-BetaWolves/2026chassisbot/tree/46bf8b2abbeade7e5baf1d7a9b3c634889b7ccec

If you need to reference a known-working state of the AprilTag code, check out this commit.

---

## What We Changed for 2026

When updating from the 2025 codebase to 2026, we made the following adjustments:

1. **Updated WPILib to 2026** - Ensured all vendor dependencies are 2026-compatible
2. **Updated PhotonVision libraries** - Matched PhotonVision version to 2026 release
3. **Updated AprilTag field layout** - Configured for the REBUILT (2026) field AprilTag positions
4. **Adjusted camera mounting parameters** - Updated robot-to-camera transform for our chassis

---

## Testing with AdvantageScope

We used AdvantageScope's 3D Field visualization to verify that our AprilTag detection was working correctly and that pose estimation was accurate.

**AdvantageScope Documentation:** https://docs.advantagescope.org/whats-new/

### Why AdvantageScope?

AdvantageScope's 3D field tab is particularly useful for AprilTag localization testing because it:

- Displays robot pose in 3D space (supports `Pose2d`, `Pose3d`, `Translation2d`, `Translation3d`)
- Shows the robot's estimated position on the field in real-time
- Allows comparison between odometry-only pose and vision-corrected pose
- Supports multiple camera views (Orbit Field, Orbit Robot, Driver Station, Fixed Camera)

### Field Model Note

> **Important:** The 2026 FRC field model in AdvantageScope is consistent with the AprilTag layout for the **welded** field variant. There may be minor differences (~0.5 inch) from AndyMark field layouts. See our [Field Specs documentation](00-game-field-specs.md) for perimeter type by region.

### Testing Procedure

1. **Deploy code to robot** with AprilTag vision enabled
2. **Connect AdvantageScope** to the robot via NetworkTables
3. **Open the 3D Field tab** and drag the field to the 'Poses' section
4. **Add robot pose visualization** - drag the estimated pose data to the field
5. **Point camera at AprilTags** and verify:
   - AprilTag is detected (check PhotonVision dashboard)
   - Robot pose updates in AdvantageScope
   - Pose estimate is reasonable for the robot's actual position

### Logging Pose Data

To visualize pose in AdvantageScope, we publish pose data using WPILib's struct publishers:

```java
// Example: Publishing 3D pose for AdvantageScope visualization
StructPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Vision/EstimatedPose", Pose3d.struct).publish();

// In periodic function
posePublisher.set(estimatedPose);
```

For AdvantageKit users:
```java
Logger.recordOutput("Vision/EstimatedPose", estimatedPose);
```

---

## Results

After implementing and testing:

- ✅ Robot successfully detects AprilTags via PhotonVision
- ✅ Pose estimation updates correctly when AprilTags are visible
- ✅ AdvantageScope displays robot position matching physical location
- ✅ Position updates smoothly as robot moves and different tags come into view

---

## Related Documentation

- [Field Specifications](00-game-field-specs.md) - AprilTag locations and heights for 2026 REBUILT field
- [Firmware Updates](01-updating-firmware-to-latest-frc-versions.md) - PhotonVision co-processor setup (Step 5)
- [PhotonVision Docs](https://docs.photonvision.org/) - Official PhotonVision documentation
- [AdvantageScope Docs](https://docs.advantagescope.org/) - 3D field visualization and logging

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No AprilTags detected | Check PhotonVision dashboard, verify camera connected, check pipeline settings |
| Wrong pose estimate | Verify robot-to-camera transform, check AprilTag field layout matches 2026 |
| Pose jumps erratically | May indicate incorrect camera calibration or noisy detection - check lighting |
| AdvantageScope not showing pose | Verify data is being published, check NetworkTables connection |
| Tags detected but pose not updating | Check that pose estimator is consuming vision measurements |

---

## Change Log

| Date | Author | Changes |
|------|--------|---------|
| 2026-01-26 | Carney Robotics | Initial documentation - AprilTag vision setup and testing |
