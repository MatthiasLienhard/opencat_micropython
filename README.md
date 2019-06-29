# opencat_micropython
micropython implementation for esp32 of the opencat

I used Ez 3D Printed OpenCat from thingiverse: https://www.thingiverse.com/thing:3384371

Here you see the first steps it made: https://www.youtube.com/watch?v=MNgKbt5TRXs


This project is in a very early state. I did not look at the original opencat firmware, so pinout is probably different.
Servo ids: FLS, FLK, BLS, BLK, FRS, FRK, BRS, BRK, T, HY, HP
- F: front leg
- B: back (hind) leg
- T: tail
- H: head
- L: left
- R: right
- S: shoulder
- K: knee
- Y: yaw (left-right, shake)
- P: pitch (up-down, nod)




## Features: 
- Two Link kinematics
  - used for the legs to control the servo angles
- linear motion plans for all legs, tail and head:
  - A motion plan is a function in time that returns the servo angles theta
  - f(time)=Theta1,Theta2
  - defined by a set of 2D points and the time the movement between the points should take
- two different gaits
- some simple gestures (sleep, eat, nod, shake haead, wag)


## My Plans:
### Software (ESP32 micropython)
- webserver for simple remote control
- walk backwards and turn
- reverse the prefered angle in the kinetics of the front legs, such that all 'knees' are pointing inwards when walking
- motion plan with circular trajectories ( better walking)
- integrate head and tail to stabilize waling 
- integrate sensors -> requires hardware
- autonomous mode (obstical avoidance) -> requires sensors
- more cute gestures
- sound (meow?...) with according head movement
- orientation aware motion plans
- object recognition (on camera module)
- deepsleep standby?

### Hardware
- power: 
  - on/of swicth
  - 2S 7.2V lipo
    - servos are powerd directly
  - 5V buck converter 
    - if sufficient: esp board (which has a 3.3V converter) 
    - audio amp
    - ultrasonic sensor
- MPU gyrosensor (i2c)
- ultrasonic senosor
- 5V audio amp (ESP DAC)
  - try old headphone speakers?
- Servo feedback? (currently analog feedback on one of the sensors prepared to monitor motion)
- camera module (e.g. JeVois)
