# Simulator
Our simulator is capable of simulating all sensor of the physical FreiCar. The simulator comes with the docker image and thus does need to be installed manually.
 
## Starting the Simulator
The simulator together with some necessary ROS-nodes can be started by running:

```roslaunch freicar_launch local_comp_launch.launch```

The simulator needs always to run if you want to start your own programs. Your ROS-nodes must be started in a separate launch file.
In this launch file also the map is specified. See down below how to change it.

## Updating the Simulator
We might update the simulator from time to time. When a new simulator version comes out you can upgrade the simulator by running ```update_simulator.bash``` from the docker directory.

## Maps
We have prepared various urban and race maps that can be loaded into the simulator. 

In order to change the map open the file ```local_comp_launch.launch``` from the package ```freicar_launch``` and change the ```map_path``` to the corresponding map-name in the table below.   

Preview | Simulator Map Name (freicar_settings) | ROS Map Name (local_comp_launch.launch)
--- | --- | ---
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar1.png "") | freicar1 | freicar_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar2.png "") | freicar2 | freicar_2.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_1.png "") | freicar_race_1 | freicar_race_1.aismap
![alt text](https://github.com/JohanVer/freicar_docs/raw/master/images/maps/freicar_race_2.png "") | freicar_race_2 | freicar_race_2.aismap

## Spawn Poses
There are two ways to define the initial spawn pose of the car:
1. The freicar_carla_proxy node provides the parameters ```spawn/x```, ```spawn/y```, ```spawn/z``` and ```spawn/heading``` that can be used to setup the initial pose.
 Note that if you use the ```sim_base.launch``` these parameters are getting overwritten by the values defined in this launch file.
 
2. Additionally freicar_carla_proxy provides the parameter ```use_yaml_spawn```. Note again that if you use ```sim_base.launch``` the value gets overwritten by the file.
 If ```use_yaml_spawn``` is set to true the spawn pose will be read from the files located in ``` freicar_base/freicar_launch/spawn_positions ``` depending on the map that is defined by the global parameter ```map_path``` (set in ```sim_base.launch```).

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