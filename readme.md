<div align="center">

# DCServo AE

[![License: CERN-OHL](https://img.shields.io/badge/Hardware-CERN--OHL-yellow.svg)](https://ohwr.org/cernohl)
[![License: MIT](https://img.shields.io/badge/Software-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Project Status: WIP](https://img.shields.io/badge/Project%20Status-WIP-orange.svg)]()

**Transform any Brushed DC motor into a high-precision smart servo**

[Features](#features) • [Getting Started](#getting-started) • [Documentation](#documentation) • [Contributing](#contributing) • [Community](#community)

<img src="/api/placeholder/800/400" alt="DCServo AE Project Banner">

</div>

## 🚨 Project Status

> **This project is currently under active development by a single person.**
>
> **⚠️ IMPORTANT:** Manufacturing anything from this repository is not recommended at this time as designs are still being validated and tested.

### 🤝 Call for Contributors
We're actively seeking contributors with expertise in:
- CAD/Mechanical Design
- Specifically needed: Casing design for shaft-mounted versions
- Hardware validation and testing

## 🎯 Project Overview

DCServo AE is an open-source platform that transforms standard Brushed DC motors into continuous smart servos with high precision control. Our focus is on delivering:

- ⚡ **High Torque** output capabilities
- 🔄 **High RPM** performance
- 📏 **Precise Positioning** through absolute encoding
- 🔌 **Plug-and-Play** implementation

### Platform Variations

All versions utilize the **STM32G0** microcontroller series, chosen for its:
- 💰 Cost-effectiveness
- ⚡ Powerful timer peripherals
- 🎮 Advanced control capabilities

| Version | Description | Status |
|---------|-------------|---------|
| High Power | For industrial applications | 🟡 In Development |
| Low Power | For hobbyist projects | 🟡 In Development |
| High Voltage | 24V+ systems | 🟢 Planning |
| Low Voltage | Battery-powered devices | 🟢 Planning |

## ✨ Core Features

### Hardware Highlights
- 🔌 **Plug and Play Integration**
  - Compatible with extended/dual axis shaft motors
  - Requires magnet mounting for absolute encoding
  - Minimal design considerations needed

- 📏 **Absolute Position Encoding**
  - Magnetic absolute encoders for precise positioning
  - No homing or recalibration needed after power loss
  - Reliable position feedback

- 🎛️ **Modular Architecture**
  - Multiple configurations available
  - Scalable from hobby to industrial applications
  - Compact form factor (as small as 12mm diameter)

### Software Features
- 🔓 **Open Source**
  - Full hardware design files
  - Complete firmware source code
  - Comprehensive documentation (WIP)

- 🎮 **Control Interfaces** (Planned)
  - CAN bus
  - UART serial
  - PWM input

## 🛠️ Getting Started

### Current Manufacturing Notes
If you wish to help test the current designs:
```
📁 /Export           - Pre-exported Gerbers for each version
🏭 JLCPCB           - Files optimized for JLCPCB manufacturing
⚠️ Manual Assembly  - Currently requires hand soldering (SMT optimization coming soon)
```

### Build Instructions

#### Hardware
1. Access design files in the `/hardware` directory
   - KiCad 8.0 format PCB and schematics
   - Separate directories for each version
   - Complete BOMs included

#### Software
1. Find firmware in the `/software` directory
   - Written in C for STM32
   - Includes all necessary drivers
   - Configuration templates provided

## 📚 Documentation

> 🚧 Documentation is currently under development

Planned documentation will cover:
- [ ] Hardware assembly guides
- [ ] Software configuration
- [ ] Control interface specifications
- [ ] Performance tuning
- [ ] Application examples

## 👥 Contributing

We welcome contributions! Here's how you can help:

1. 🐛 **Report Issues**
   - Use GitHub Issues for bugs
   - Suggest improvements
   - Request features

2. 💻 **Submit Pull Requests**
   - Hardware improvements
   - Firmware enhancements
   - Documentation updates

3. 🤝 **Join Discussions**
   - Participate in GitHub Discussions
   - Join our Discord community
   - Share your implementations

## 🌟 Similar Projects

Projects that inspired DCServo AE:

- [Mechaduino](https://hackaday.io/project/11224-mechaduino)
  - Stepper motor focus
  - Open-source architecture

- [MotCTRL](https://github.com/osannolik/MotCtrl)
  - BLDC motor controller
  - Advanced control algorithms

## 📜 License

- Hardware: [CERN Open Hardware License](LICENSE-HARDWARE)
- Software: [MIT License](LICENSE-SOFTWARE)

## 🤝 Community

> 🚧 Community platforms are being established

Future platforms will include:
- Hackaday.io project page
- Discord community
- YouTube channel for tutorials and demos

---

<div align="center">

**[Back to Top](#dcservo-ae)**

</div>
