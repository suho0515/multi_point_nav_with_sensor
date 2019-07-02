# multi_point_nav_with_sensor
multi point navigation source using weight sensor

primatily it's for RB1_Base Mobile Robot of Robotnik.
but can be used for any other Robots.

# rb1_base package
you better use packages in your rb1_base PC
you just need rb1_description for rviz

# Set up for this file.
The hardware system is "Sensor" - "Arduino" - "Master PC"
So if you wanna use it with sensor. you should change the topic name of the sensor. original topic name is "/weight"
And you should change the location written in this file.

# Interface
rb1_base does navigation
remote pc visualize rviz only

# How to use it
1. Start a Navigation which should have a map
2. Run the file. "python multi_point_nav_with_sensor.py"
3. Run the "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600" for Arduino
4. Turn on the Rviz
5. Input the position using 2D Pose Estimation
