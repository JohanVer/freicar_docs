# FreiCAR Competition Rules

## Competition Tasks:
Overall you have to solve the following tasks:

### 1. Follow a circular race track (10 P)
This is the first task you have to solve. A map is given that consists of a single circular route with a 1/1 road scheme (1 ego-lane, 1 opposite-lane).
Your task is to follow this track at least one round as fast as possible. No high level commands will be sent for this task. There are no other cars and no traffic signs have to be considered. You can try the task 5 times. After the last attempt we continue with the next task.

### 2. Follow a complex road topology (10 P)
This task involves taking turns at junctions and considering stop signs and right-of-way signs. Your car is controlled by the supervision team with high-level commands (see below) and is supposed to take at least 5 successfull turns in a row. If no high-level command is sent before a junction, an arbitrary direction can be selected.

For stop-signs stop at the stop-line at least 1 sec, for right-of-way signs continue driving without interruption.

For this task there is no other traffic on the map.
A turn is successful if the vehicle does not enter a forbidden lane completely (e.g opposite) while turning and the correct junction arm (as sent in the high-level command) is fully entered. 
You can try the task 5 times (The attempt counter is incremented as soon as an unsuccessful turn is executed). After the last attempt we continue with the next task.

### 3. Consider other vehicles (10 P)
Similar to task 2, your car will be controlled by the supervision team using high-level commands. However, this time there will be other cars driving on the map.
You are asked to visualize the detections of other traffic participants in rviz. Furthermore, your car needs to drive for at least 2 minutes without crashing into any other object (e.g. vehicle or sign) including considering stop signs and right-of-way signs. Incorrect behavior of the agent (i.e. not yielding at a right-of-way when a traffic participants rightly enters a junction), causing an other vehicle having to emergency-brake also counts as a failed attempt.
You can try the task 5 times. After the last attempt we continue with the next task.

### 4. Overtaking (10 P)

This task involves overtaking a standing vehicle in a safe and timely manner. The standing vehicle can be located anywhere in the map, but outside junction areas. The standing vehicle has to be detected, and the agent has to find a safe trajectory around the vehicle by entering the opposing lane and not leaving the road. The opposing lane must be left immediately after passing the standing vehicle. No other traffic participants will be on the road for this task. The overtaking maneuver has to be executed successfully two times in total, for varying standing vehicle positions.
You can try the task 5 times (you have to get 2/5 overtaking maneuvers correct).


### Agent behavior assessment (10 P)

In addition to the task rules as stated above, the team of supervisors will judge the overall vehicle behavior in all tasks according to the following rules:

- Driving style: The vehicle should follow a smooth driving trajectory and should drive with reasonable speed (faster than a snail) when it is safe.
- Behavior w.r.t other vehicles: A safe distance should be kept to all traffic participants at all times. The vehicle should yield at a right-of-way sign if another vehicle is approaching. Crashing into other vehicles is not allowed.
- Leaving the road or fully entering the opposing lane counts as a failed attempt

### Code Quality

We also judge the quality of the submitted competition code. Good code could be judged by the following criteria:
- Clear naming convention of variables and overall readability and formatting
- Clear structure of codebase into disjoint submodules that explicitly solve a single sub-task


### General notes:


- The vehicle will be re-spawned to an initial pose and initial state after each task.
- No code change is allowed between the attempts. This includes the prohibition to use any task-specific agent profiles (the vehicle cannot know which task it is solving at a given time) changing the foundational logic of your overall approach (i.e. switching a component on or off). But changes to single parameters of a component are fine. Code recompilation between tasks or runs is also not allowed.
- The final competition map will not be available before the competition. 

Full marks (10 P) for a task will be given if the required number of successful attempts is reached. Partial marks (< 10 P) will be awarded if the required number of successful attempts is not reached. The exact number of points awarded is decided by the supevisor team according to the severeness of the occured failure and the overall situational circumstances.

#### Control System
The supervision team will control your cars while evaluation of all tasks using high-level commands. 
This high-level command can be:
 
 - `left`
 - `right`
 - `straight`
 - `stop`
 - `start`
 
If i.e. a `right` command is send to your car your car is supposed to take a right turn when entering the immediately upcoming junction. After the junction is exited, this command can be considered stale and does no longer have to be considered. The same goes for `left` and `straight`. The commands `stop` and `start` should be followed at all times and should make the vehicle stop and start in a safe manner.

The high-level command is represented as `ros-msg` (`freicar_common/FreiCarControl`) with the types:

- `name: string`
- `command: string`

You can publish all commands using the scripts in: ```freicar_ws/src/freicar_base/freicar_launch/bash/commands``` .


#### System Setup and Startup

In your repository root directory, we expect a `setup_env.sh` bash script that downloads/compiles any needed libraries, sets system and environment variables, and compiles your ROS nodes. Next, we expect a  `start_system.sh` bash script in the root of your submitted repository which calls your roslaunch file starting all your system components. No further interaction should be needed from our side. This includes starting additional roslaunch files, changes in directory or setting of environment variables, etc.
Your submitted repository will be placed in `/home/freicar/freicar_ws/src` with all other submodules remaining, so make sure that there aren't any conflicts.

Any deviation from this code convention will reflect in the final grade. 

