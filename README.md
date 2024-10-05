
Remote Raspberry Pi Connector
Overview
Remote Raspberry Pi Connector for Visual Studio Code simplifies SSH management of multiple Raspberry Pi devices. Control GPIO pins, run commands, transfer files, and automate tasks—all within the IDE. Ideal for robotics, IoT, and medical engineering, it boosts productivity and streamlines remote development.

Features
-Multi-Device SSH Management: Connect and switch between multiple Raspberry Pi devices.

-Command Execution: Run shell commands remotely on Raspberry Pi devices.

-File Transfer: Upload and download files between your local system and Raspberry Pi.

-Integrated Terminal: Open a terminal for each connected Raspberry Pi within Visual Studio Code.

-Custom Command Storage: Save frequently used commands for quick execution.

Installation
Install Visual Studio Code.
Go to the Extensions view by clicking the Extensions icon in the Activity Bar on the side of the window or by pressing Ctrl+Shift+X.
Search for Remote Raspberry Pi Connector and click Install.
Usage
Once the extension is installed, you can utilize the following commands from the Visual Studio Code command palette (Ctrl+Shift+P):

1. Connect to Raspberry Pi: Establish a new SSH connection to a Raspberry Pi device.

2. Run Command on Raspberry Pi (Ctrl+Shift+r): Execute shell commands on the connected Raspberry Pi.

3. Upload File to Raspberry Pi: Transfer a file from your local system to the Raspberry Pi.

4. Download File from Raspberry Pi: Retrieve a file from the Raspberry Pi to your local machine.

5. Open Integrated Terminal: Launch a terminal session within Visual Studio Code for remote interactions.

6. Save Custom Command: Save custom commands for easy and quick access.

7. Add Device: Add a new Raspberry Pi device to your configuration list.

8. Switch Device: Switch between different connected devices as needed.

9. Disconnect from Raspberry Pi: Terminate the current SSH session.

Connecting to a Raspberry Pi
Open the Command Palette (Ctrl+Shift+P).
Type Raspberry Pi: Remote Pi: Connect to Raspberry Pi and select it.
Enter the SSH details of your Raspberry Pi.
Note: GPIO Pinout

   3V3  (1) (2)  5V    
 GPIO2  (3) (4)  5V    
 GPIO3  (5) (6)  GND   
 GPIO4  (7) (8)  GPIO14
   GND  (9) (10) GPIO15
GPIO17 (11) (12) GPIO18
GPIO27 (13) (14) GND   
GPIO22 (15) (16) GPIO23
   3V3 (17) (18) GPIO24
GPIO10 (19) (20) GND   
 GPIO9 (21) (22) GPIO25
GPIO11 (23) (24) GPIO8 
   GND (25) (26) GPIO7 
 GPIO0 (27) (28) GPIO1 
 GPIO5 (29) (30) GND   
 GPIO6 (31) (32) GPIO12
GPIO13 (33) (34) GND   
GPIO19 (35) (36) GPIO16
GPIO26 (37) (38) GPIO20
   GND (39) (40) GPIO21

Running Commands
Open the Command Palette (Ctrl+Shift+P).
Type Raspberry Pi: Remote Pi: Run Command on Raspberry Pi and select it.
Enter the command you want to run.
Some File Management Commands
Get detailed information about your Raspberry Pi's hardware:cat /proc/cpuinfo
Current Directory: Your current directory is /home/[yourname]
List files: ls
List files in the project directory: ls project/
Copy the file to the project directory: cp 01.py project/
Run the copied file: python project/01.py
Uploading and Downloading Files
Uploading Files

Right-click on the code file in VSCode, and select Raspberry Pi: Remote Pi: Upload File to Raspberry Pi.
After selecting your file for upload, type the target directory and file name, for example: project/003.jpg
Downloading Files

Right-click on the code file in VSCode, and select Raspberry Pi: Remote Pi: Download File from Raspberry Pi.
After running the download command, type the source directory and file name, for example: project/008.jpg
Then select the target location on your computer (e.g., type extension for it: downloaded-file.jpg).

Example Usage - Blink the LED:

Methode 1:

Create Your Code: For example, create a file named 01.py with your Python script.

Upload it to Raspberry Pi: Right-click on the code file in VSCode, and select Raspberry Pi: Remote Pi: Upload File to Raspberry Pi.

Run the Code: Right-click on the code file in VSCode, and select Raspberry Pi: Remote Pi: Run Command on Raspberry Pi. Type: python 01.py

Methode 2:

To set GPIO19 as output directly: gpio -g mode 19 out
Turn on the LED (set GPIO19 to high):gpio -g write 19 1
Turn off the LED (set GPIO19 to low): gpio -g write 19 0
Managing GPIO
Use the integrated terminal to manage GPIO pins using your preferred method (e.g., Python scripts).

Commands
The extension contributes the following commands:

Raspberry Pi: Remote Pi: Connect to Raspberry Pi
Raspberry Pi: Remote Pi: Run Command on Raspberry Pi (Ctrl+Shift+r)
Raspberry Pi: Remote Pi: Upload File to Raspberry Pi
Raspberry Pi: Remote Pi: Download File from Raspberry Pi
Raspberry Pi: Remote Pi: Disconnect from Raspberry Pi
Raspberry Pi: Remote Pi: Open Integrated Terminal Type: ssh yourPiUserName@ip (For example: ssh pi@192.168.19.15)
Raspberry Pi: Remote Pi: Save Custom Command
Raspberry Pi: Add Device
Raspberry Pi: Switch Device
Raspberry Pi Projects Guide
Raspberry Pi Common Commands
Prefix: rpi-commands
Description: Comprehensive commands for managing Raspberry Pi, robotics, sensors, and more.

GPIO Setup
Prefix: rpi-gpio-setup
Description: Basic GPIO pin setup and control using RPi.GPIO library.

Blinking LED
Prefix: rpi-blink-led
Description: Blinking LED example using RPi.GPIO.

Read Button Press
Prefix: rpi-read-button
Description: Read the state of a button and print a message when pressed.

Read DS18B20 Temperature
Prefix: rpi-read-ds18b20
Description: Read temperature from a DS18B20 sensor connected to the Raspberry Pi.

Take a Photo with Pi Camera
Prefix: rpi-photo
Description: Capture a photo using the Pi Camera module.

Robot Move Forward
Prefix: rpi-robot-forward
Description: Move the robot forward for 2 seconds.

Robot Move Backward
Prefix: rpi-robot-backward
Description: Move the robot backward for 2 seconds.

Robot Turn Left
Prefix: rpi-robot-left
Description: Turn the robot left for 1 second.

Robot Turn Right
Prefix: rpi-robot-right
Description: Turn the robot right for 1 second.

Robot Stop
Prefix: rpi-robot-stop
Description: Stop the robot immediately.

Line Follower Robot
Prefix: rpi-robot-linefollower
Description: Follow a black line using IR sensors.

Obstacle Avoider Robot
Prefix: rpi-robot-obstacleavoider
Description: Avoid obstacles using an ultrasonic sensor.

Ball Tracker Robot
Prefix: rpi-robot-balltracker
Description: Track and follow a yellow ball using the camera feed.

PID Motor Controller
Prefix: rpi-robot-pidcontroller
Description: Basic PID controller for motor speed using an encoder.

Maze Solver Robot
Prefix: rpi-robot-mazesolver
Description: Solve a simple maze using IR sensors.

Drone Takeoff and Land
Prefix: rpi-drone-takeoffland
Description: Basic drone takeoff and land using the dronekit library.

Bluetooth Controlled Robot
Prefix: rpi-robot-bluetoothcontrol
Description: Control a Raspberry Pi robot using Bluetooth.

Drone Path Planning
Prefix: rpi-drone-pathplanning
Description: Autonomous drone path planning using predefined GPS waypoints with dronekit.

Set GPIO Pin as Output
Prefix: rpi-gpio-set-output
Description: Sets a GPIO pin as an output using the gpio command.

Turn On LED
Prefix: rpi-gpio-turn-on-led
Description: Turns on an LED connected to a specified GPIO pin.

Turn Off LED
Prefix: rpi-gpio-turn-off-led
Description: Turns off an LED connected to a specified GPIO pin.

Read GPIO State
Prefix: rpi-gpio-read-pin
Description: Reads the state of a specified GPIO pin (high/low).

Set GPIO Pin as Input
Prefix: rpi-gpio-set-input
Description: Sets a GPIO pin as an input using the gpio command.

Blink LED
Prefix: rpi-gpio-blink-led
Description: Blinks an LED on GPIO using the watch command.

Control Multiple LEDs
Prefix: rpi-gpio-multiple-leds
Description: Turns on and off multiple LEDs connected to different GPIO pins.

Button-Controlled LED
Prefix: rpi-gpio-button-led
Description: Button controls the LED state based on its press/release state.

Ultrasonic Distance Measurement
Prefix: rpi-ultrasonic
Description: Measures distance using an ultrasonic sensor (HC-SR04).

Motion Detection with PIR
Prefix: rpi-motion-sensor
Description: Detects motion using a PIR sensor and controls an LED.

Temperature and Humidity Reading
Prefix: rpi-dht-sensor
Description: Reads temperature and humidity from a DHT11 or DHT22 sensor.

Light Sensor with LDR
Prefix: rpi-ldr-sensor
Description: Detects light levels using an LDR.

Continuous Ultrasonic Measurement
Prefix: rpi-continuous-ultrasonic
Description: Continuously measures distance using an ultrasonic sensor (HC-SR04).

Heart Rate Monitor
Prefix: rpi-heart-rate
Description: Monitors heart rate using a pulse sensor.

Temperature Measurement with Thermocouple
Prefix: rpi-thermocouple
Description: Measures temperature using a thermocouple (MAX31855).

Blood Pressure Monitoring
Prefix: rpi-blood-pressure
Description: Simulates blood pressure readings using a pressure sensor.

ECG Signal Monitoring
Prefix: rpi-ecg-signal
Description: Simulates ECG signal readings from an ECG sensor.

Glucose Level Monitoring
Prefix: rpi-glucose-level
Description: Simulates glucose level readings from a glucose sensor.

Send Temperature Data to ThingSpeak
Prefix: rpi-thingspeak
Description: Sends temperature and humidity data to ThingSpeak.

Control LED Remotely with Flask
Prefix: rpi-flask-led
Description: Controls an LED remotely using a Flask web server.

MQTT Temperature Monitoring
Prefix: rpi-mqtt
Description: Publishes temperature readings to an MQTT broker.

Data Collection with Adafruit IO
Prefix: rpi-adafruit-io
Description: Collects temperature and humidity data and sends it to Adafruit IO.

Smart Home Control with GPIO
Prefix: rpi-smart-home
Description: Controls smart home devices using a web interface.

IoT Weather Station with BME280
Prefix: rpi-weather-station
Description: Reads data from a BME280 sensor and sends it to a weather API.

Node-RED Dashboard Integration
Prefix: rpi-nodered
Description: Integrates Raspberry Pi with Node-RED for creating a visual dashboard.

Send Data to Blynk
Prefix: rpi-blynk
Description: Sends temperature and humidity data to Blynk for visualization.

Raspberry Pi Controls Arduino
Prefix: rpi-arduino-serial
Description: Sends commands from Raspberry Pi to Arduino to control an LED.

Raspberry Pi Speech Recognition
Prefix: rpi-speech-recognition
Description: Uses the SpeechRecognition library to recognize speech input from a microphone.

Raspberry Pi Camera Stream
Prefix: rpi-camera-stream
Description: Streams live camera feed using Flask web framework.

Raspberry Pi Text-to-Speech
Prefix: rpi-text-to-speech
Description: Uses the pyttsx3 library to convert text to speech.

Raspberry Pi File Transfer Tool
Prefix: rpi-file-transfer
Description: A GUI tool for transferring files to a Raspberry Pi using SCP.

Raspberry Pi Motion Detection Camera
Prefix: rpi-motion-detect-camera
Description: Captures images using the Raspberry Pi camera when motion is detected.

Raspberry Pi SSH Command
Prefix: rpi-sshCommand
Description: SSH command example.

Contributing
Contributions are welcome! Please visit the repository to submit issues or pull requests.

License
This project is licensed under the MIT License. See the LICENSE file for details.

Testing Information
This extension has been tested on the following environments:

Operating System: Windows 8.1, Windows 10, Windows 11, Ubuntu 22.04, Ubuntu 24.04
Raspberry Pi Model: Raspberry Pi 3 Model B, Raspberry Pi 4 Model B
Publisher: daryoushalipourtirotir
Version: 1.3.0
VS Code Engine: ^1.75.0
Categories: Programming Languages, Other
Keywords: Raspberry Pi, SSH, Remote, Development, IoT, Python, Robot, Robotics, Device Management

🎉 Thank You for Using Remote Raspberry Pi Connector!
We hope this extension enhances your workflow and makes your coding experience smoother. Your feedback is invaluable to us.

If you enjoyed using it, please consider ⭐️ starring this project and sharing it with your friends and colleagues.

Happy Coding! 🚀
