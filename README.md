# ICE-Governor-Retrofit

## Rationale & Goal

The goal of this project is to provide a low cost electronic governor retrofit for small/carburetor engines to replace poor, worn, malfunctioning, or destroyed governors. Typical off-the-shelf variants cost in excess of $100 USD, surpassing the remaining value of the small engine they might be applied to. The target deployment engine this project is built for specifically is a 4000W single phase (true) generator, where the engine must be kept at a steady 3600 RPM or 3000 RPM for 60 Hz or 50 Hz respectively. Provisions should be made in the design to allow for dynamic target speed setting both for fine tuning or for a dynamic governor, such as in a small tractor or ride on lawnmower, perhaps even a gas powertool.

## Pin Count

### Pin Planning
Max IO: 6 pins. One of the potentiometers should be limited to range of 2.5V to HIGH so that I can be used on the RESET line of the ATTiny (GROUND/LOW Pull is reset active).
1. Spark Detect +
    - Bit 2 (Pin7) is the external interrupt pin and would be preferred, however there is only one external interrupt.
2. Spark Detect -
    - Bit 0 (Pin 5) for convince, has Pin Change Interrupt.
3. Servo PWM
    - Bit 4 or Bit 1 should be used for the non-inverted PWM output.
    - Pin 6 or Pin 3.
4. Speed (Position) Potentiometer
    - Potentiometer for speed
    - (Optional) Grounding/High switch to signal off.
        - Cannot be on reset (Bit 5, Pin 1)
5. Integral Potentiometer
6. Derivative Potentiometer

### Signaled Shutoff Plan
This variant of the schematic and pin planning uses the switch (SPDT) to signal the ATTiny to ramp down (and stall) the engine before entering a sleep state. This will result in a parasitic draw when in sleep/off to allow the microcontroller to wake up and for the LDO Voltage Regulator to down-regulate the battery. This is done as a trade off to make operation safer by ensuring that the throttle body is closed to prevent a runaway. The battery planned is a relatively larger 9V box battery.

#### By Pin Number
1. Second Order Reaction
2. First Order Reaction
3. Speed Set and Sleep Trigger
4. GND
5. Spark Trigger
6. Servo Position PWM
7. Spark Trigger
8. VCC

#### By Port Number (All Port B)
0. Spark Trigger
1. Servo Position PWM
2. Spark Trigger
3. First Order Reaction
4. Speed Set and Sleep Trigger
5. Second Order Reaction

#### Schematic Plan
![Schematic](./design/assets/ice-governor-retrofit-Signaled-Shutoff.drawio.svg)