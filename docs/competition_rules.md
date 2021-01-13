#FreiCAR Competition Rules

UNDER CONSTRUCTION

###Tasks to complete:
Overall you have to solve various tasks:
####1. Follow a circular race track
This is the first task you have to solve. A map is given that consists of a single circular route with a 1/1 road scheme (1 ego-lane, 1 opposite-lane).
Your task is to follow this track at least one round as fast as possible. There are no other cars and no traffic signs have to be considered. You can try the task 5 times. After the last try we continue with the next task.

####2. Follow a complex road topology
This task involves taking turns at junctions and considering stop signs and right-of-way signs. Your car is controlled by the supervision team with high-level commands and is supposed to take at least 5 successfull turns in a row.

For stop-signs stop at the stop-line at least 1 sec, for right-of-way signs continue driving without interruption.

For this task there is no other traffic on the map.
A turn is successful if it does not enter a un-allowed lane completely (e.g opposite) , cutting is allowed.
You can try the task 5 times. After the last try we continue with the next task.

####3. Consider other vehicles
Similar to task 2 your car will be controlled by the supervision team using high-level commands. However, this time there will be other cars driving on the map.
You are asked to visualize the detections of other traffic participants in rviz. Further your car need to drive at least 2 minutes without crashing into any other object.
You can try the task 5 times. After the last try we continue with the next task.

####4. Overtaking
TBD
  
####Notes:
No code change is allowed between the tries.

The final competition map will not be available before the competition. 

###Control System
The supervision team will control your cars while evaluation of all tasks using high-level commands. 
This high-level command can be :
 
 - 'left'
 - 'right'
 - 'straight'
 - 'stop'
 - 'start'
 
If a 'right' command is send to your car your car is supposed to take a right turn when entering a junction.

The high-level command is represented as ros-msg (freicar_common/FreiCarControl) with the types:

- name: string
- command: string

You can publish all commands using the scripts in:```freicar_ws/src/freicar_base/freicar_launch/bash/commands``` .