_This entire project is currently a WIP by a single person_
_Looking for contributors in the space of CAD / Mechanical. I need help making a casing for the shaft-mounted versions_

**DCServo AE**

  DCServo AE is an open-source platform designed to effortlessly transform any Brushed DC motor into a continous smart servo with **high precision** and control. Built to provide **high-torque**, **high-RPM**, and accurate positioning through absolute encoding, DCServo AE is a modular solution for hobbyists, roboticists, and makers. 
  The platform allows users to easily drop in the compact board onto a brushed DC motor, turning it into a powerful and precise smart servo, regardless of the motor's size or specifications.

**Project Overview**

  The platform aims to support several variations, including options for high-power, low-power, high(er)-voltage, and low-voltage setups. Each variation is designed to accommodate different performance requirements while maintaining the core features of precise motion control and absolute position feedback. All variations will be based around the **STM32G0** series of microcontrollers due to their **low cost** and **powerful timers and peripherals**. Additionally, all versions will likely use **DRV-series drivers** to ensure robust motor control.

**Core Features**
- Plug and Play: With minimal considerations for design, it is ready to be strapped on the back of any **extended or dual axis/shaft, with magnet** BRUSHED DC Motor and turn it into a Smart Servo Motor.
- Absolute Encoder: DCServo AE uses magnetic absolute encoders for precise positioning and reliable feedback, eliminating the need for homing or recalibration after power loss.
- Modular Design: Multiple configurations allow for flexibility across a wide range of applications, from lightweight robotics to heavy-duty industrial tasks.
- Open-Source: Both hardware and software are fully open-source, allowing anyone to contribute, modify, and improve the platform.
- Compact Size: The designs are optimized for compactness, with some controllers as small as 12 mm in diameter, making them suitable for space-constrained applications.

**Getting Started**

***_It would be a REALLY bad idea to try and manufacture anything present here at this time_***

If you do, however want to help me test it out:
- All Gerbers are pre-exported in the "Export" folder for each version
- They were made with JLCPCB in mind
- They are not JLC SMT friendly whatsoever, so expect the need to manually solder (sorry, fixing in the next verison 🤞)

**Build Your Own:**

- Design files (PCB, schematics) are provided in each version's hardware directory. Design files are in KiCad 8.0 format.
- Code (C, STM32) is provided in each version's software directory.
- GERBERS and BOMs are also provided ready to upload to JLCPCB.

**Documentation**

_WIP, planned control: CAN, UART, PWM_

**Contributing**

We welcome contributions to the PowerServo AE project! Whether you want to improve the hardware, add new features to the firmware, or suggest new variations, feel free to contribute through:

- Issues and Pull Requests on GitHub
- Community discussions in our Discord channel

**License**
- Hardware: Licensed under the CERN Open Hardware License.
- Software: Licensed under the MIT License.

**Join the Community**

_WIP_
Stay updated with the latest developments, tutorials, and community projects:

- Follow on Hackaday.io
- Join the discussion on Discord
- Watch video tutorials and project demos on YouTube

**Similar Projects (and source of inspiration)**

Mechaduino: Similar, but for Steppers specifically
https://hackaday.io/project/11224-mechaduino

MotCTRL: Similar, but for Brushless specifically
https://github.com/osannolik/MotCtrl
