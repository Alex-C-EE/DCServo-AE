# TODO List for PCB V0.4

## Schematic Updates
- [x] Move U1 and Part Number to top of microcontroller
- [x] Add 25 MHz frequency value next to Y1
- [x] Change IC# designators to U#
- [x] Rearrange symbols to remove text/line crossings
- [ ] Change microcontroller symbol and move SWD/NRST connections to the bottom left
- [ ] Flip CAN RX and TX
- [ ] Add Common Node Choke for CAN


## Component Changes
- [x] Upgrade ISENS resistor to 2512 package for 2W+ power dissipation at 100 mOhm
- [x] Change 22uF caps rating from 6V to 16V after the buck
- [x] Change 47uF capacitor specifications (C11 - 47uF 50V in 0805 doesn't exist)
- [x] Move RSENSE closer to driver
- [x] Add electrolytic capacitor near driver's VMOT where space permits
- [x] Move C11 further from IC1

## PCB Layout
- [x] Increase spacing between XT30 connectors to allow usage of heat shrink
- [x] Move connector description silkscreen to back side
- [ ] Add pin 1 triangular designators
- [x] Add shield pad vias on encoder connector
- [x] Space out U3 traces
- [x] Add more vias near data vias
- [x] Reduce unnecessary vias covering text
- [x] Move the matched CAN trace away from nearby pad
- [x] Increase ISENS trace width for 4.1A current
- [x] Fix copper hole/teardrops near C17 by using through-hole test point as via
- [x] Reduce excessive via grids (no denser than lambda/10 of highest frequency)
- [ ] *Add ground pin to CAN connectors?*
- [x] Route CAN H/L as differential pair
- [x] Move board version next to "DCServo" text in upper right corner

## Documentation
- [ ] Disable net names on all 2D PCB images (except board dimensions)
- [x] Fix SWD text to show "vdd-gnd-gnd"
- [x] Move board date to bottom side

## 3D Model
- [ ] Remove duplicate XT30 connectors floating above board

## Design Review
- [x] Rethink CAN pass-through and node management/stud length
- [x] Don't compromise manufacturability for perfect length matching (10cm tolerance ok)
- [x] Avoid placing traces between pads even if within minimum track width rules