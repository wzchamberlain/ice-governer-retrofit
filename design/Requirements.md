# Requirements Design Document
This document outlines the requirements of this project in order to complete the goal of providing a electronic throttle governance (by speed) retrofit for small engines.
At the writing of this section, previous considerations had been made and hardware selections made. As such this document regards **software requirements** under the constraints of the previously selected hardware rather than the *hardware requirements* that dictated the hardware selections.

## Functional Requirements
These are the functional objectives that the software must achieve.

1. Produce a PWM signal at 50 Hz (t=20 ms) with duty cycle of 1-2 ms (5-10 %) to control throttle positioning servo (sg90).
2. Time spark pulse intervals.
3. Mark spark pulse polarities to eliminate transience.
4. Analog read for speed set point. *Alteratively as Position Error constant.*
5. Interrupt read on speed set to enable sleep-wake behaviour.
6. Analog read for corrective response speed/aggression. *Alternatively as Integral Error constant.*
7. Analog read for corrective response dampening. *Alternatively as Derivative Error constant.*

## Assignments of functions on ATTiny85
The ATTiny85 provides all the needed functional peripherals to achieve these functions, but they must be configured to connect the bus functions and peripheral components as such.

Needed peripheral assignments:

| Function/Peripheral | Count | Reason/Assignment | Port Number(s) | Pin Number(s) |
| ------------------: | ----: | :---------------- | -------------: | ------------: |
| PWM (Timer with prescale and dedicated reset output) | 1x | PWM to pin for throttle servo control signal. Outputs directly to pin. | 1 | 6x |
| Timer (with prescale) | 1x | For measurement of timing of spark pulses (for engine speed calculation and Process). No physical IO, logically controlled by software interrupts. | N/A | N/A |
| PCINT (Process Interrupt) | 2x | For catchment of spark trigger events. Mapped to spark sense pins. | 0, 2 | 5, 7 |
| PCINT/INT | 1x (Assigned on sleep) | For wake from sleep state. Mapped to speed/proportion response pin. | 4 | 3 |
| ADC | 3x | ADC measurement of trim potentiometers for adjustment of Set Point and/or PID constants | 3, 4, 5 | 1, 2, 3 |
