<snippet>
  <content>
# ESP8266_Balanging_ROBOT
!! In testing phase !!
2 wheel robot based B-Robot and Oleg Kravtchenko changes. The code is running 100% on ESP8266, this includes Motor Control, PID and OSC user interface on a mobile device.
## Installation
Compilation using Arduino 1.6.12 and ESP8266 plugin v2.3.0
Processor: ESP8266 Running at 3.3V !!!
Motor interface: DRV8825 (under test)
Motor 2x NMEA 17 (under test)
Gyro IMU 9150, using i2c interface(Drotek)
5V step-down module (Banggood)

## Usage
-You will need to install OSC-TOUCH on a PC and upload the user interface on your mobile phone or tablet.
-Configure the ESP code to get wifi access to your robot
-Connect with your mobile divice to the robot
Communication is possible over a local router (Station mode) or directly (Soft-AP mode).


## History
Code Origin:
https://github.com/jjrobots/B-ROBOT/
http://jjrobots.com/forum/thread-826.html?highlight=only+you


## Credits
B-Robot, jjrobots and Oleg Kravtchenko
## License
GPL v2
</content>
</snippet>
