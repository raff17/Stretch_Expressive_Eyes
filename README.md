This is a custom LED eye module design for the Hello-Robo RE2. This Eye module displays different emotions (sad, angry, happy, confused, and normal (staring)) and can gaze utilizing the tilt-pan camera located on the robot. 


![Eyes_expressions gaze1-3](https://github.com/user-attachments/assets/1a81e5c7-6dae-4575-aa92-3ec6ed713f14)


![Eyes_expressions-3](https://github.com/user-attachments/assets/40aa3db7-43b3-4b1e-9a08-403490bf8655)


**BOM** 

Arduino Mega or an ESP32 board

Ability to 3D print (stand)

BTF-Lighting (8cm*32cm) LED strip (link: https://www.amazon.com/BTF-LIGHTING-Individual-Addressable-Flexible-Controllers/dp/B09XWR1Y5K/ref=sr_1_5?crid=8J6ZMHYBALX0&dib=eyJ2IjoiMSJ9.1xnIkPzQ5iqcq1PRC6M9nCeNOSXXcAX1VYdgFX2gk2B354Oh3ZdfH8_WAFrJGWeAbLsZ8hl9l4I6Jy6wzR8zNO7WOZ6AgkCJiwvE2okwLEhQrFjYJ2NS0kM8bHtoLf7N-a3Hyd9ARJ0_0_0pD7ku2__CK6vs_Pqmk0UqLADlt0hCMDOaBOwts965Hk8yue3lCp9Y8Z6R5uaTqJG4Ihi3JpqAZstZVO1N6UHNOClApq3-rqVv68uI8gsgfgvxDLtPWkTsHSCB_t560MH2AZLffnDM2I6cOfN2dhpZEd_WXCY.B0ewe3eqZNB2wKoaLC0sSWDF7U3jQh2lvY8rNELxum8&dib_tag=se&keywords=BTF-Lighting+%288cm*32cm%29+LED+strip&qid=1726448037&sprefix=btf-lighting+8cm+32cm+led+strip%2Caps%2C202&sr=8-5)

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
