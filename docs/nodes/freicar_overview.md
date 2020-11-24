# Introduction to the FreiCar Nodes

In the following, we'll give you a short overview of the ROS nodes we've developed over the last year for the FreiCar project. We'll introduce them in more detail as the course continues.

*Note* it is not possible to exhaustively test all these nodes in an academical environment. If you run into any bugs, send a message to one of your supervisors along with the the steps required to reproduce the it.

___
# 1. FreiCar Agent
This is the base node for all agents. Currently it:

- Sends `Track` requests at start and shutdown
- Publishes `FreiCarStatus` messages
- Publishes `FreiCarLocalization` messages
- Publishes a 3D model for visualization on RVIZ
___
# 2. FreiCar Carla Proxy
This node handles all the necessary tasks that are related to the simulator:

- Spawning the agent in CARLA
- Setting up the sensors and publishing their information
- Publishing the recevied pose on tf (simulated agents only)
- Removing the agent from simulation at shutdown

The sensor definition for each agent is a yaml file located at [`freicar_carla_proxy/param/sensors.yaml`](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_carla_proxy/param/sensors.yaml). You can read more about carla sensors [here](https://carla.readthedocs.io/en/latest/ref_sensors/).
___
# 3. FreiCar Carla Agent
This node spawns an arbitrary number of scripted agents in the environment. These agents generate random plans and follow them while obeying right of way rules and avoiding collisions. They perform these tasks by using the `FreiCarAgentLocalization` messages from other agents. 

## Use
After compiling the node, you can start it with:
```bash
rosrun freicar_carla_agent freicar_carla_agent_node <number-of-agents> <seed>
```
The random seed is an optional argument. Providing it will change the behavior of the agents, i.e. where they spawn and how they plan.

___
# 4. FreiCar Chaperone
The chaperone node is responsible for making sure the agents do not collide with eachother. It also makes sure the agents remain inside a specified boundary polygon.
The chaperone node also serves `Track` requests. These help the chaperone node keep track of all the running agents and uses their tf information to predict where they might end up in the next few seconds.

___
# 5. FreiCar Common

This node serves as a reference for all the messages, services and shared headers in our stack. If you're familiar with ROS, you'd know that sharing any type of code between two packages is a tedious task. For large projects, it is much easier to create a single package that is `find_package()`ed in every CMakeLists.txt.

## 5.1. Messages
The following messages are used in the FreiCar project. You can click on each one to see the message file.

### 5.1.1. [FreiCarAgentLocalization](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/msg/FreiCarAgentLocalization.msg)
Each agent is required to publish this message at ~10Hz+ on the `car_localization` topic. This information is currently only used by the [scripted agents](#3-freicar-carla-agent). Each message contains the name of the agent, the uuid of the current lane it's on, its offset along that lane, its velocity and 3D pose. With the simulated cars, the 3D pose is recieved from unreal and published as a [ros::tf](http://wiki.ros.org/tf) message.

**Note** At the beginning, you will receive the ground truth pose from the tf. As the course goes on, you will replace it with the output of your localization node. This message however still has to be filled with ground truth data. Currently this task is done by the [freicar_agent](#1-freicar-agent) node. 

### 5.1.2. [FreiCarControl](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/msg/FreiCarControl.msg)
This is a reserved message, used by the administrator to suspend or release agents.

### 5.1.3. [FreiCarGoTo](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/msg/FreiCarGoTo.msg)
These messages are sent over the `freicar_goto`. They specify a certain [lane, lane offset] or [x, y, heading] as a goal. The agent is then supposed to plan to the requested position and drive there.

### 5.1.4. [FreiCarHalt](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/msg/FreiCarHalt.msg)
All agents must subscribe to 'halt'. These messages usually come from the [chaperone node](#4-freicar-chaperone) to prevent a collision with other agents. When this message is received, the agent is supposed to stop as soon as possible. The message contains the name of the agents involved as well as a basic reason why the agent was suspended.

### 5.1.4. [Resume](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)
The chaperone node publishes the names of the agents that are allowed to continue after being suspended. The message is just a string, published on `resume`. As with the FreiCarHalt message, all agents must subscribe to this topic.

### 5.1.5. [FreiCarStatus](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/msg/FreiCarStatus.msg)
Each agent must publish a message of this type on  `freicar_status` at least once every second. The admin GUI uses these messages to provide an overview of all running agents. Currently they are published by [freicar_agent](#1-freicar-agent), but this could change as the we progress.

## 5.2. Services
You can think of services as bidirectional ROS message. We currently have two services.

### 5.2.1. [Track Request](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/srv/Track.srv)
Track requests are served by the [chaperone node](#4-freicar-chaperone). Each agent must send a track request at initialization, stating its name, the name of its tf frame and whether it's a scripted agent. The latter is only true for the scripted agents in the [freicar_carla_agent](#3-freicar-carla-agent) node. As the agent is shutting down, it must send another request with `bool track` set to false. Currently [freicar_agent](#1-freicar-agent) handles sending these track requests.

### 5.2.2. [WayPoint Request](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_common/srv/WayPoint.srv)
This is a deprecated service in `freicar_map`. It has the same functionality as `freicar::planning::lane_follower::GetPlan(...)`. It'll most likely be removed in a future commit.

## 5.3. Shared Headers
These headers contain enums that are used by other nodes.

### 5.3.1. halt_type.h
```cpp
enum HaltType : unsigned char {
	CORRIDOR_INTERSECT,
	COLLISION,
	DISABLED,
	OUT_OF_BOUNDS,
	ADMIN_ORDER,
	NONE
};
```
An instance of this enum is sent with each FreiCarHalt message to indicate the reason behind suspension.

### 5.3.2. planner_cmd.h
```cpp
enum PlannerCommand : unsigned char {
	LEFT = 1,		// go   left   if possible, fail at the junction otherwise
	RIGHT = 2,		// go   right  if possible, fail at the junction otherwise
	STRAIGHT = 3,	// go straight if possible, fail at the junction otherwise
	RANDOM = 4,		// choose a random direction
	POINT = 5,		// lane-star planner to a specific point
	DIRECT = 6,		// direct path strategy
	EMPTY = 7		// empty plan
};
```
This enum is used by planners to specify the command.
___

# 6. FreiCar Joy
This node is used in tandem with others to enable joystick controls for a simulated agent. An example can be seen in [`freicar_agent/launch/sim_agent.launch`](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_agent/launch/sim_agent.launch).

## Supported Joysticks
The `controller_type` parameter controls which joystick template the node uses. We've only tested the node with xbox360 and ps4 joysticks but feel free to add your own definition.

# 7. FreiCar Map
This node contains the core map structure and additional services that are built on top of it. You can find a more detailed explanation [here](https://freicar-docs.readthedocs.io/nodes/freicar_map).

# 8. FreiCar Setting
This node is responsible for applying the simulation settings and setting a couple of global ROS parameters.

## 8.1. YAML File
The CARLA API exposes 3 settings for the simulation. They are read from [freicar_setting/param/carla_settings.yaml](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_setting/param/carla_settings.yaml).

### no rendering
If `no-render-mode` is set to true, the simulator will stop rendering completely. This obviously uses less resources but also means the camera images will not be available.
**This is not the same as headless rendering**.

### simulated steps per second
`sim-steps-per-second` determines the desired number of simulated steps per second. To achieve acceptable physics simulation, the number should be at least 10. If set to 0, the simulation will run at full speed. This will naturally lead to variable delta time between the steps.

**Note:** This does not determine the simulator's FPS, although it can affect it. It also doesn't determine the amount of data you get from the sensors. Those are set in the sensor description file.

### synchronous mode
if `synchronous` is set to true, the simulation server will not proceed until a `Tick()` is received. Technically all CARLA clients can send it but this node is currently the only one responsible for the sake of consistency. Setting this to true will start a thread ([currently deactivated](https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_base/-/blob/master/freicar_setting/src/main.cpp#L86)) that tries to tick the server according to `sim-steps-per-second`.

You can read more about [world settings](https://carla.readthedocs.io/en/latest/python_api/#carla.WorldSettings) and [client-server synchrony](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/).

## 8.2. Parameters
The aforementioned settings are also set as global ROS parameters.

### sim_sps
This defines the simulated steps per second. Other nodes (e.g. freicar_agent) use this as their thread's sleep value. If the simulator is trying to simulate the environment `x` times per seconds, it makes sense for us to update our agents with the same frequency.

### sim_sync_mode
Indicates whether the synchronous mode has been activated. So far, no node uses this information.

### sim_norender
Indicates whether the no-render mode has been activated. So far, no node uses this information.

# 9. Raiscar Messages
This package contains legacy messages from the audi cup that are not yet integrated into the FreiCar project.