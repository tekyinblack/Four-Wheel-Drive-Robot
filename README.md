# Four-Wheel-Drive-Robot
This is the code associated with the robot platforms described on tekyinblack.com
These are basic robot cars using four wheel drive, to which a variety of features have been added. 
They are intended to provide an example route from being remote controlled to autonomous 
All are based around teh ESP32 D1 controller board, similar in layout to the Arduino Uno
# Robot with tank steering
This has a very basic move-stop-turn-continue operation, so has forward, backward, spin left or spin right actions
The robot itself has a single L298D motor driver module, which the motors on each side wired in parallel.
# Robot with differential steering
This has a more sophisticated proportional steering which facilitates steering while driving by adjusting the drive to the motors
# Robot with mecanum wheels
This uses two L298D driver modules to provide indepenednt proportional drive to each wheel. This allows the robot platform to accomplish 
forwards, backwards, spin left, spin right, sideways left, sideways right control
