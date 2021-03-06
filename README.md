# ArPiBot
This project is based on the following hardware robot: [Elegoo robot car kit (2.0)](https://www.elegoo.com/product/elegoo-uno-project-upgraded-smart-robot-car-kit-v2-0/)[(review)](http://robotfanatics.com/elegoo-smart-car-review/)

### Video on YouTube (from [review](http://robotfanatics.com/elegoo-smart-car-review/))
This video shows the basic default behavior of this bot. It drives, looks around and avoids obstacles.
<a href="http://www.youtube.com/watch?feature=player_embedded&v=f8ZBd4TpMBg
" target="_blank"><img src="http://img.youtube.com/vi/f8ZBd4TpMBg/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>

### Standalone
As standard, the bot cannot look around and change behavoir at the same time due to synchronous processing. The ultrasonic sensor is not electronically connected to the interrupt pins on the arduino, making it impossible (without altering the bot) to implement async behaviour using these interrupts. Therefore, we will use a software version.

This software version and comparable simple behaviour is implemented in the "standalone" folder

Behaviour:
- Asynchronously readout ultrasonic sensor measurements
- Simply act on measurements and control motors
- Drive straight untill obstacle is detected. Simple "calculation" to find best way out. (Possibly back away and drive an available direction. Or simply adjust forward course)

### Arduino with Raspberry Pi
The bot's Arduino will perform rudimentary communication with the Raspberry Pi. It can be found in the "arduino_with_rpi" folder.
- Arduino USB connected to RPi USB. 
- Serial communication
- Sensor measurements from Arduino -> RPi
- Control commands from RPi -> Arduino (Aim sensor, retrieve sensor value, drive speed, drive direction)