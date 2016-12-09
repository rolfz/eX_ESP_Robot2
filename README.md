<snippet>
  <content><![CDATA[
# ${1:Project Name}
!! In testing phase !!
2 wheel robot based B-Robot and Oleg Kravtchenko changes. The code is running 100% on ESP8266, this includes Motor Control, PID and OSC user interface
## Installation
Compilation using Arduino 1.6.12 and ESP8266 plugin v2.3.0
Processor: ESP8266 Running at 3.3V !!!
Motor interface: DRV8825 (testing)
Motor 2x NMEA 17 (testing)
Gyro IMU 9150 (using i2c interface)

## Usage
-You will need to install OSC-TOUCH on a PC and upload the user interface on your mobile phone or tablet.
-Configure the ESP code to get wifi access to your robot
-Connect with your mobile divice to the robot
Communication is possible over a local router (Station mode) or directly (Soft-AP mode).


## History
Code Origin:
https://github.com/jjrobots/B-ROBOT/

TODO: Write history
## Credits
B-Robot and Oleg Kravtchenko
## License
GPL v2
]]></content>
  <tabTrigger>readme</tabTrigger>
</snippet>
