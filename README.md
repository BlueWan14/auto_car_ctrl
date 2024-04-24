# Abstract
This repository present you a package running with ROS noetic. It is used for [Autonomous car competition](https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/) organized by Paris Saclay.  
This is an academic project for the authors[^1] during their 2<sup>nd</sup> year of [mecatronic in ISTY](https://www.isty.uvsq.fr/cycle-ingenieur-mecatronique)[^2].

[^1]: Pardie MOSKOFIAN, Mateo GELIN and Erwan MAWART
[^2]: *Institue des Sciences et Techniques des Yvelines*


# Installation
## Hardware
### Requierments
- Raspberry PI 4[^3]
- Arduino Mega 2560[^3]
- Lidar X4 from ydlidar
- TAMAYA car

[^3]: equivalent or better configuration

### Connections
Please look at the [electrical schematic](electrical_schematic.png).


## Sofware
### Requierments
- [ROS noetic](https://wiki.ros.org/ROS)
- [ydlidar ros driver package](https://www.ydlidar.com/dowfile.html?cid=5&type=2)
- [serial ros driver package](https://github.com/ros-drivers/rosserial)
- [Arduino IDE](https://www.arduino.cc/en/software)

### Package installation
After installing requierments listed above, install this package with those commands :
```
cd ~/catkin_ws/src/
```
```
git clone https://github.com/BlueWan14/auto_car_ctrl.git
```
```
cd .. && catkin_make
```

Then use this command to compile ROS libraries for Arduino :
```
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries/
```
And upload [car_motors.ino](./car_motors/car_motors.ino) in the Arduino card with the Arduino IDE.


# Using the package
## Full program
```
roslaunch auto_car_ctrl cmd_car.launch
```
This command will launch the entire program. That include lidar and arduino processes.
> [!CAUTION]  
> Make sure that Arduino card and lidar are plug before execute.

#### Possible arguments :  
- `dist_follow_wall:=[decimal number]` => Distance minimal between the lidar and the wall in meter that the car needs to keep. *By default set to `0.15`.*  
> [!IMPORTANT]  
> Can't be set under `0.1` because of the car size.
- `speed_max:=[decimal number]` => Top speed used by the car. It's a pourcentage of the motor maximum speed. *By default set to `50.0`.*
- `rviz:=[boolean]` => If true, starts rviz application with correct configuration. *By default set to `false`.*
- `arduino_port:=[string]` => Selected port where the arduino is plug.Useally get the form `ttyACM#` in dev/ repository. *By default set to `ttyACM0`.*

## Raspberry's nodes only
```
roslaunch auto_car_ctrl auto_car_ctrl_node.launch
```
This command will launch the three nodes inside this package only. It is usefull for testing the program with a rosbag. You can find some records on the official track [here](./rosbags/).  
 
#### Possible arguments :  
- `dist_follow_wall:=[decimal number]` => Distance minimal between the lidar and the wall in meter that the car needs to keep. *By default set to `0.15`.*  
> [!IMPORTANT]  
> Can't be set under `0.1` because of the car size.
- `speed_max:=[decimal number]` => Top speed used by the car. It's a pourcentage of the motor maximum speed. *By default set to `50.0`.*
- `rviz:=[boolean]` => If true, starts rviz application with correct configuration. *By default set to `false`.*
- `lidar:=[boolean]` => If true, includes lidar process in the launch file. *By default set to `false`.*  
> [!CAUTION]  
> Make sure that lidar is plug before execute.

Next, if you use a rosbag, you'll need to use this command in a new terminal while the previous roslaunch command is running.  
```
rosbag play -l ~/catkin_ws/src/auto_car_ctrl/rosbags/[NameOfTheRosbag].bag --topics /scan
```

> [!TIP]  
> To observ outputs you can use ```rostopic echo [topic]```

## Lidar processing only
```
roslaunch auto_car_ctrl lidar_process.launch
```
This command will launch the three nodes inside this package only. It is usefull for testing the program with a rosbag. You can find some records on the official track [here](./rosbags/).  
 
#### Possible arguments :  
- `dist_follow_wall:=[decimal number]` => Distance minimal between the lidar and the wall in meter that the car needs to keep. *By default set to `0.15`.*  
> [!IMPORTANT]  
> Can't be set under `0.1` because of the car size.
- `rviz:=[boolean]` => If true, starts rviz application with correct configuration. *By default set to `true`.*
- `lidar:=[boolean]` => If true, includes lidar process in the launch file. *By default set to `false`.*  
> [!CAUTION]  
> Make sure that lidar is plug before execute.

Next, if you use a rosbag, you'll need to use this command in a new terminal while the previous roslaunch command is running.  
```
rosbag play -l ~/catkin_ws/src/auto_car_ctrl/rosbags/[NameOfTheRosbag].bag --topics /scan
```


# What could be improved
In the [issue tab](https://github.com/BlueWan14/auto_car_ctrl/issues) you can find every enhancement that we[^1] think interesting to work on.  
Feel free to continue this project.
