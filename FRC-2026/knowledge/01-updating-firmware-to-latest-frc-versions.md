# Updating Robot System Software & Component Firmware to latest FRC 2026 Season Updates

This document covers the pre-season checklist for updating all firmware and software for our FRC robot. Complete these steps before the kickoff event to ensure everything is competition-ready.

---

## Step 1: VividHosting Radio Configuration

The VividHosting radio replaced the older OpenMesh radio starting in FRC 2025. This is the official FRC radio and must be properly configured for your team.

**Official Quick Start Guide:** https://frc-radio.vivid-hosting.net/overview/quick-start-guide

### Hardware Overview

The VividHosting radio has the following ports:
- **12V** - Power input (12V DC from robot power)
- **RIO** - Ethernet connection to the roboRIO
- **AUX2** - Auxiliary Ethernet port

![VividHosting Radio Ports](photos/vividhosting-radio-setup/IMG_0173.jpg)
*Close-up showing the 12V power input, RIO port with green status LED, and AUX2 port*

### Prerequisites

Before configuring the radio, ensure you have:
- [ ] Windows laptop with FRC Radio Configuration Tool installed
- [ ] Ethernet cable to connect laptop directly to the radio
- [ ] Robot powered on (or 12V power supply for bench configuration)
- [ ] Your team number ready
- [ ] Encryption key matches on both receiving radio and driver station computer

### Configuration Steps

#### 1. Connect the Radio for Configuration

Connect your laptop directly to the radio via Ethernet cable:

![Radio Configuration Setup](photos/vividhosting-radio-setup/IMG_0170.jpg)
*Radio connected via Ethernet for configuration - note the label showing important settings*

#### 2. Run the FRC Radio Configuration Tool

Launch the FRC Radio Configuration Tool on your laptop:

![FRC Radio Configuration Tool](photos/vividhosting-radio-setup/IMG_0171.jpg)
*Laptop running the FRC Radio Configuration Tool*

In the tool:
1. Select your team number
2. Choose the appropriate mode (typically "2.4GHz + 5GHz" for practice, competitions will use 5GHz only)
3. Click "Configure"
4. Wait for the configuration to complete

#### 3. Verify Firmware Version

After configuration, verify the radio is running the latest firmware. Our radio is currently running:

**Firmware Version: 1.2.6**

![Radio with Firmware Label](photos/vividhosting-radio-setup/IMG_0168.jpg)
*Radio mounted on robot showing firmware version label (1.2.6)*

> **Important Note:** The label shows "NOT 2.4GHz" as a reminder that competition fields use 5GHz only. The 2.4GHz band is disabled during official matches.

### Physical Installation

#### Mounting Location

Mount the radio in an accessible location on the robot, preferably:
- Away from motors and high-current wiring (to reduce interference)
- In a protected position to prevent damage during matches
- Where status LEDs are visible for troubleshooting

![Radio Mounting on Robot](photos/vividhosting-radio-setup/IMG_0172.jpg)
*Radio mounted on robot frame with clear labeling*

![Radio Position in Electronics Panel](photos/vividhosting-radio-setup/IMG_0174.jpg)
*Side view showing radio position relative to other electronics*

#### Wiring Connections

1. **Power**: Connect 12V DC power to the 12V port using the provided connector (red = positive, black = ground)
2. **RIO Connection**: Connect an Ethernet cable from the RIO port to the roboRIO's Ethernet port

![Full Electronics Panel](photos/vividhosting-radio-setup/IMG_0175.jpg)
*Complete electronics panel showing radio position (left), RSL light (right, orange), and overall wiring*

![Robot Overview](photos/vividhosting-radio-setup/IMG_0169.jpg)
*Full robot view showing electronics layout with radio visible on upper frame*

### Status LEDs

The VividHosting radio has several status LEDs:
- **Power LED** - Solid when powered on
- **RIO Port LED** - Green when roboRIO connection is active
- **Status LEDs** - Indicate Wi-Fi activity and connection status

A properly configured radio will show:
- Solid power LED
- Green LED on the RIO port when connected to roboRIO
- Blinking activity LEDs when communicating

### Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| No power LED | No 12V power | Check power connections and fuse |
| No RIO LED | Ethernet not connected | Check cable, verify roboRIO is on |
| Can't connect to Driver Station | Radio not configured | Re-run configuration tool |
| Intermittent connection | Interference or weak signal | Reposition radio, check for metal obstructions |

### Competition Day Checklist

- [ ] Radio is securely mounted
- [ ] Power and Ethernet cables are secure
- [ ] Radio firmware matches competition requirements
- [ ] Team number is correctly configured
- [ ] Encryption key verified
- [ ] Practice connection test completed

---

## Step 2: roboRIO v1 Firmware Update

*Documentation coming soon - we updated our roboRIO v1 to the latest firmware*

---

## Step 3: Motor Controller Firmware Updates

*Documentation coming soon*

---

## Step 4: FRC Game Tools Installation

*Documentation coming soon*

---

## Change Log

| Date | Author | Changes |
|------|--------|---------|
| 2026-01-26 | Carney Robotics | Initial documentation - VividHosting radio setup |
