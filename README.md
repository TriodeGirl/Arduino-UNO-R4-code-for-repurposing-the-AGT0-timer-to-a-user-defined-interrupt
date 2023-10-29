# Arduino-UNO-R4-code-for-repurposing-the-AGT0-timer-to-a-user-defined-interrupt
Arduino UNO R4 code for repurposing the AGT0 timer to a user defined interrupt, enable AGT0 output on D5, plus use fast ADC and DAC operation, fast-pins, and HOCO to XTAL clock change

Code can be used to test the ADC accuracy under different clock conditions, enable an external XTAL with the PLL if part fitted onto PCB.

Scope pic shows ATG0 output (4 - green) which is toggling at 1.000mS intervals, plus asociated interrupts timing shown using fast-pin writes.

![susan-arduino-R4-AGT0-repurpose-user-interrupt-1](https://github.com/TriodeGirl/Arduino-UNO-R4-code-for-repurposing-the-AGT0-timer-to-a-user-defined-interrupt/assets/139503623/54518166-0762-4fcb-abaa-b5a63569093f)
