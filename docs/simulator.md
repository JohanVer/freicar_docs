# Simulator
Our simulator is capable of simulating all sensors of the physical FreiCar. The simulator comes with the docker image and thus does need to be installed manually.
 
## Starting the Simulator
The simulator together with some necessary ROS-nodes can be started by running:

```roslaunch freicar_launch local_comp_launch.launch```

The simulator needs always to run if you want to start your own programs. Your ROS-nodes must be started in a separate launch file.
In this launch file also the map is specified. See down below how to change it.

## Updating the Simulator
We might update the simulator from time to time. When a new simulator version comes out you can upgrade the simulator by running ```update_simulator.bash``` from the docker directory.

## Simulated Sensors/Actuators
### Throttle, Steering, Brake
All cars can be controlled over throttle, steering and brake. The corresponding ROS-message can be looked up [here](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/raiscar_msgs/msg/ControlCommand.msg) and can be sent to `/AnyCarName/control`.

### Localization
If the competition mode is turned off (see explanation below), a ground-truth localization will be provided as tf-transform from `/map -> /AnyCarName/handle` (center of the car). You can use a [ROS tf-listener](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29) to query this transform in your code.  

### Odometry
The odometry of the car is provided on the topic `/AnyCarName/odometry` using the  [odometry message](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html) message type.

### RGB Camera
The RGB image of the front facing camera is published as [ROS image message](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) on the topic: `/AnyCarName/sim/camera/rgb/front/image`.

![rgb_sensor](https://github.com/JohanVer/freicar_docs/raw/master/images/rgb_sensor.png "") 

### Depth Camera
We also provide a depth image (also as [ROS image message](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)) ,which is aligning with the RGB camera, on the topic : `/AnyCarName/sim/camera/depth/front/image_float`. The depth image is encoded as single channel floating point image and the unit is meters.

![depth_sensor](https://github.com/JohanVer/freicar_docs/raw/master/images/depth_sensor.png "")

### Lidar
We simulate the Sick lidar on the topic `/AnyCarName/sim/lidar` as  [PointCloud2 message](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html). After receiving the message in one of your nodes we recommend to [convert the PointCloud2 message to a pcl](https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/) format for convenience.

## Maps
We have prepared various urban and race maps that can be loaded into the simulator. 

In order to change the map open the file ```local_comp_launch.launch``` from the package ```freicar_launch``` and change the ```map_name``` to the corresponding map-name in the table below.

You can also provide the map_name as command-line argument like: ``` roslaunch freicar_launch local_comp_launch.launch map_name:=freicar_race_1.aismap ```

Preview | ROS Map Name (local_comp_launch.launch)
--- | ---
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar1.png "") | freicar_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar2.png "") | freicar_2.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_1.png "") | freicar_race_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_2.png "") | freicar_race_2.aismap

## Competition Mode
By default the simulator provides ground-truth localization (a tf-transform from /map to /car_name) and ground-truth odometry. However in the final competition this information won't be available. 
In the ```local_comp_launch.launch``` a argument named ```comp_mode``` is defined (default: false). If ```comp_mode``` is set to true the ground-truth localization is not published anymore and the odometry will be noisy.

Like the map-name you can set the comp_mode as a command line argument like: ``` roslaunch freicar_launch local_comp_launch.launch map_name:=freicar_1.aismap comp_mode:=true```

In the final competition ```comp_mode``` is set to true.  


## Spawn Poses
There are two ways to define the initial spawn pose of the car:

1. The freicar_carla_proxy node provides the parameters ```spawn/x```, ```spawn/y```, ```spawn/z``` and ```spawn/heading``` that can be used to setup the initial pose.
 Note that if you use the ```sim_base.launch``` these parameters are getting overwritten by the values defined in this launch file.
 
2. Additionally freicar_carla_proxy provides the parameter ```use_yaml_spawn```. Note again that if you use ```sim_base.launch``` the value gets overwritten by the file.
 If ```use_yaml_spawn``` is set to true the spawn pose will be read from the files located in ``` freicar_base/freicar_launch/spawn_positions ``` depending on the map that is defined by the global parameter ```map_path``` (set in ```sim_base.launch```).
 If the spawn position is used from the yaml file also the ros-parameter ``` spawn/x ``` etc will be set accordingly.

The heading angle is in degree and defined as follows:

![angle_definition](https://github.com/JohanVer/freicar_docs/raw/master/images/angle_def.png "")  
   

## Reset/Set Car Positions Manually

After spawning a car two ROS services are created (by freicar_carla_proxy).
 - ```/FREICAR_NAME/set_position``` (e.g ```/freicar_1/set_position```)
 - ```/FREICAR_NAME/reset_position``` (e.g ```/freicar_1/reset_position```)

These services can be used to either reset the cars position to the spawn pose or to set a custom pose. In both cases the car will be beamed to the changed pose.

### Set a new pose
Example to set a new pose (in case the car is called "freicar_1"):

``` rosservice call /freicar_1/set_position "{x: -0.3, y: -0.3, z: 0.0, heading: -45.0}" ```

### Reset to spawn pose
Example how to reset the cars pose to the spawn pose (in case the car is called "freicar_1"):

``` rosservice call /freicar_1/reset_position true ```

## Spawning Dynamic Cars
In order to test your system with other dynamic traffic participants you can spawn multiple other scripted cars that obey the traffic rules.

To do so, run: ```rosrun freicar_executables freicar_carla_agent_node 5```. This will spawn 5 other dynamic cars. Note that the simulation has to run before.

## Using a Gamepad

If you like you can use a gamepad to control your car. Just type: ```roslaunch freicar_launch start_joystick.launch name:=freicar_anyname``` where "freicar_anyname" is your car-name.
We support currently xbox controllers but you could change the "FreiCar Joy" node to your needs (see [Freicar Overview](https://freicar-docs.readthedocs.io/nodes/freicar_overview/))

![angle_definition](https://github.com/JohanVer/freicar_docs/raw/master/images/xbox_joy_buttons.png "")  
