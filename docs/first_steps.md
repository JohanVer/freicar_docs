# First Steps

This section explains how to start with developing in the FreiCar framework.

1. As the first step you should start the simulator by starting ```local_comp_launch.launch```. For this please read the [Simulator section](https://freicar-docs.readthedocs.io/simulator/) section.
2. Now the simulator should be started and you are ready to spawn your car.
 Run ``` roslaunch freicar_agent spawn_car.launch name:=freicar_anyname tf_name:=freicar_anyname spawn/x:=0 spawn/y:=0 spawn/z:=0 spawn/heading:=20 use_yaml_spawn:=true sync_topic:=! ```.
  Now your car "freicar_anyname" is spawned in the world and all sensors of the respective car are running. The car-name should be changeable throughout the course, so do not hardcode the name in your future own programs but use always ros-parameters.
3. Now start the [Rviz](http://wiki.ros.org/rviz#Overview) tool to visualize the world and the cars sensors. You should explore what data is available with ```rostopic list```

# Start Programming your own Code

For the FreiCar Course you can freely program any ROS node you want to have. However one node has to send the "Track Request".

We prepared a template node [Freicar Agent](https://freicar-docs.readthedocs.io/nodes/freicar_agent/) for you that sends the "Track Request" and shows how to initialize a HD map, get the sensor data or requests a localization-pose from the tf-system. This node is a good starting point if you want to program in C++.

Overall, either you base on the template Freicar Agent node that has this functionality already, or you simply run the Freicar Agent node as it is in order to send the track request.

Running the Freicar Agent node can be done with (NOTE: This also spawns the car so use this command instead of spawn_car.launch): ```roslaunch freicar_agent sim_agent.launch name:=freicar_anyname tf_name:=freicar_anyname spawn/x:=0 spawn/y:=0 spawn/z:=0 spawn/heading:=20 use_yaml_spawn:=true sync_topic:=!```

You are allowed to use any code from the freicar_base submodule. Read the sections [Freicar Overview](https://freicar-docs.readthedocs.io/nodes/freicar_overview/) and [Freicar Map](https://freicar-docs.readthedocs.io/nodes/freicar_map/) to read which nodes are available.

# Code Style

Always make your own **private** repository for your software. No changes to the submodules ``` freicar_base```, ```freicar_exercises``` or ```freicar_executables``` are allowed.
 
 If you want to use code from these submodules you are allowed to copy these nodes, rename them and push them in your repository. 
