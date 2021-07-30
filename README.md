# StepSequencer (Teensy)

Started as a small challenge, this project kept growing. 
Its a StepSequencer based on a powerful Teensy w. 16 steps, a OLED Display, Midi In/Out, etc.

In general its working pretty good. 
A main issue is the display: refreshing breaks the time loop.

I guess the problem is the missing hardware acceleration for SPI, because the teensy was too new.
Without refreshing the display, the time loop is pretty accurate.

The teensy is listening for incoming Midi Clock. Then it follows the Clocks tempo.

Maybe there will be hardware acceleration featured or the OLED display can be replaced with a simple LED counter for bpm.

Its fun! :-)
