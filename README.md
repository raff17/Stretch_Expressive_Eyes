This is a custom LED eye module design for the Hello-Robo RE2. This Eye module displays different emotions (sad, angry, happy, confuse, and normal (staring)) and can gaze utilizing the tilt-pan camera located on the robot. 


![Eyes_expressions gaze1-3](https://github.com/user-attachments/assets/1a81e5c7-6dae-4575-aa92-3ec6ed713f14)


![Eyes_expressions-3](https://github.com/user-attachments/assets/40aa3db7-43b3-4b1e-9a08-403490bf8655)


**BOM** 

Arduino Mega or an ESP32 board

Ability to 3D print (stand)

BTF-Lighting (8cm*32cm) LED strip (link: https://a.co/d/6l0Y1tK)
**LED strip wire connections** 

Red = Vout (5V)

Black = Ground 

Green = signal (Pin4 in Arduino)

**How to install** 
Clone the repo into your Catkin folder.

**How to run**

1) run the Hello-Robot calibration code 
{Stretch_robot_home.py}

2) Run the launch file 
{roslaunch Expressive_Eyes expression.launch}

3) Run the serial communication (allows you to collect information from the USB port)
{rosrun rosserial_python serial_node.py /dev/ttyACM2}

*Disclaimer: The ttyACM2 corresponds to the USB port the Arduino was connected to. It might be different for you.*
