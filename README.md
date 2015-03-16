# Alt_tricopter
I decided to write my own sketch for an Arduino altitude hold for my tri copter.

The throttle signal is fed into the Arduino and the channel 5 switch. 
Channel 5 is the mode select switch. Off = manual mode. On = Automatic (altitude hold)
In manual mode the throttle signal is passed through onto an output pin.
In automatic mode a pressure is set as a target and then a PID adjusts the throttle to maintain the altitude. 
This was programed on a Leonardo Arduino using aT5403 break out board.
The stock library for I2C causes hangs. I used https://github.com/rambo/I2C and modified the sparkfun T5403 libraries https://github.com/sparkfun/T5403_Barometric_Breakout to function.
