# WLD
Water Leak Detection system using RTOS and ARM7

##Create a ROS Workspace

```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/
catkin_make
source devel/setup.bash
```
##Create a ROS Package for the ARM7 Simulation Node
```
cd ~/ros_ws/src
catkin_create_pkg your_package_name rospy std_msgs
```

## Write the ARM7 Simulation Node

```
cd ~/ros_ws/src/your_package_name/src
nano arm7_simulation_node.py
chmod +x arm7_simulation_node.py
```

## Create a Launch File for ARM7 Simulation (Optional)

```
cd ~/ros_ws/src/your_package_name
mkdir launch
cd ~/ros_ws/src/your_package_name/launch
nano arm7_simulation.launch   #create file and save code
```

##Create a ROS Package for the Sensor Data Simulation Node

```
cd ~/ros_ws/src
catkin_create_pkg sensor_pkg rospy std_msgs
```

##Write the Sensor Data Simulation Node

```
cd ~/ros_ws/src/sensor_pkg/src
nano sensor_data_simulation_node.py
chmod +x sensor_data_simulation_node.py
```

##Create a Launch File for Sensor Data Simulation (Optional)

```
cd ~/ros_ws/src/sensor_pkg
mkdir launch
cd ~/ros_ws/src/sensor_pkg/launch
nano sensor_data_simulation.launch   #create file and save code
```

##Run the Simulation
Terminal 1
```
roslaunch your_package_name arm7_simulation.launch
```

terminal 2
```
roslaunch sensor_pkg sensor_data_simulation.launch
```

###see the output data
Terminal 3
```
rostopic echo /sensor_data
```
Terminal 4
```
ostopic echo /control_commands
```
