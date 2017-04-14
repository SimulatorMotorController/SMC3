# SMC3

The SMC3 is a "Simulator Motor Controller for 3 Motors" written for the Arduino UNO R3. At the time of writing this it has not 
been tested on any other Arduino model. The SMC3 uses a PID motor control loop. The PID algorithm has been optimised for this
application and achieves 4096 PID updates per second for all three motors giving smooth precise motor control with well tuned
parameters. Some characteristics of the controller:

- Designed specifically for use with Simtools motion control software
- Controls the position of upto three motors using analogue feedback
- Hard (reverse) braking if motors move beyond defined limits
- 10bit accuracy (1024 steps) for both "target" position commands and position "feedback"
- Extensive command set to setup and monitor the parameters for each motor
- A second serial port for (optional) real time "live" monitoring/configuration
- 4096 PID calculation updates per second for each motor
- Companion windows software to configure and monitor the SMC3
- There are currently two MODES of operation that are configured by a single line of code at compile time before uploading to the Arduino.

## MODE1: Supports the more common H-Bridges used in the forums.
This mode has a PWM output pin plus two Motor Direction output pins.
Examples include the MonsterMoto shield.

## MODE2: Designed for H-Bridges that require direct drive of Highside and Lowside switch inputs.
In this mode one switch is driven as a direction pin and the other with the PWM output however
the PWM duty needs to be inverted whenever the motor changes direction with the direction pin.
(An alternate approach would be to switch the PWM between inputs as direction changes)
An example H-Bridge that uses this mode is the cheap 43A IBT-2 found on ebay.

## MODE3: Support for Adafruit Motor Controller Shield v1
This mode requires importing the Adafruit library, and uses the library exclusively for setting
speed and rotation of the motors.

## Software Setup

- As the SMC3 is an Arduino motor controller the first thing you need to do is get the software onto your "Arduino UNO R3".
- Download the SMC3 software attached to this post (SMC3.ino)
- Get the Arduino IDE tools installed on your computer if you don't already have them. Follow the Arduino Getting Started
  guide found here: http://arduino.cc/en/Guide/Windows (Note the step for installing the drivers!)
- Open the SMC3.ino with the Arduino IDE
- Edit the code at the top if the file to select the MODE you will be using
- Compile and Download the SMC3.ino program to your Arduino UNO R3

If you want, you could now test out the [Windows SMC3 Utilities](http://github.com/SimulatorMotorController/SMC3Utils) software and check communications before proceeding further.
