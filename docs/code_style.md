# FreiCar Code Guidelines

Always make your own **private** repository for your software. No changes to the submodules ``` freicar_base```, ```freicar_exercises``` or ```freicar_executables``` are allowed.
 
If you want to use code from these submodules you are allowed to copy these nodes, improve them, rename them and push them in your repository. 

Note: Do not forget to rename the nodes in case you copied something over to your private repository. Otherwise there will be **name conflicts** since there would be multiple nodes with the same name.

# Code Submission 
For the final submission (or qualification) you must provide a link to your private git repository of your code.
For the repository the following requirements **need** to be taken into account (otherwise you can't participate in the competitions):

1. In the root of the cloned repository there has to be a `setup_env.sh` script that does all setting-up (like downloading/compiling any needed libraries or setting up system and environment variables) without the need to additionally manually execute any commands. In the case that modifications to the docker image need to be integrated (i.e. building system libraries not present in the image), 
you can include the specific commands (installing, building, ...) in the `setup_env.sh` script. Note that **no human-interaction** should be used, so while installing you can not type "yes" for some installation routines. The folder freicar_deps will be not available in the competition.
2. In the root of the cloned repository there has to be a `start.sh` bash script that starts your nodes/launch-file of your autonomous system. After starting this script your car should be in stop-mode. You should start exactly one launch-file in the script, however you can include additional launch-files in this launch file. 

Further, this bash script **must** take at least 5 arguments that define the name of your car (we might give your car a different name than the name you are using), use_yaml_spawn (bool), the spawn x-coordinate, the spawn y-coordinate and the spawn-heading. Additional parameters can be passed to your start.sh script after the last mandatory parameter (spawn_heading) in the syntax of `param_name:=param_value param_name2:=param_value2` ... . These additional parameters are received in the start.sh as `add_commands` which can be passed directly to parametrize a launch-file (see example to learn how). 
You should use the start.sh in the freicar_demo_submission repository (link below) as a starting point to create your script.
3. The simulator will be started by us in a separate terminal (before starting your `start.sh` script). So the ```local_comp_launch.launch``` file must **NOT** be started by your launch files.
4. All code should be in a directory called `ros_code`. All content in this folder will be copied over automatically in the workspace folder `freicar_ws/src`.
5. Provide a file called `agent_description.json` that defines the name of your teamname, git-url, and each ros-parameter you may want to change during the competition. These modifiable parameters are passed as additional parameters to your `start.sh` file (see point 2).
An example `agent_description.json` file is available in the freicar_demo_submission repository. It is important to exactly follow the scheme of the template.

We provide an example repository structure with `start.sh` script in the freicar_demo_submission repository: https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_demo_submission.git. Carefully study this structure and make sure your repository matches.


# Testing your Submission-Code
We provide multiple scripts that can be used to test if your repository matches the requirements.
## Building your code (as on our competition server):
In order to build your code automatically go to `freicar_docker/deploy_scripts` and type:
 
`./build_repo.sh deploy_teamname https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_demo_submission.git`
  
(Replace deploy_teamname with your team-name and the repository url with yours. However you can also execute this command directly if you want to build the demo_submission code).

This process will create a separate docker image where:

1. all commands in `setup_env.sh` are executed and committed
2. your `ros_code` will be copied into the `freicar_ws` of this new docker image
3. all ros-code is compiled

## Running your code (as on our competition server):
In order to run your code automatically do the following:

1. create a separate terminal, run fcc and then start the simulator with ```roslaunch freicar_launch local_comp_launch.launch```
2. In a separate terminal (when the simulator is fully started) go to `freicar_docker/deploy_scripts` and run ```./run_repo.sh deploy_teamname anycarname true 0 0 0 spawn_sensors:=true```. (Replace the deploy_teamname with your teamname (same as in the build process), set any car name that you like. Also set use_yaml_spawn, x-spawn, y-spawn and heading-spawn to some values you like. Note that "spawn_sensors:=true" refers to an additional parameter that is passed to the start.sh file).

This will automatically start up your built docker and internally starts `start.sh`. Your car should be ready to drive now. You can fct in the main docker and send your high-level commands to your car.

### Hints
Thorough testing will save valuable time during the competition. 


<font size="20">**If your submission does not meet the written requirements you can not participate in the competitions. Thus this will reflect in your final marks.**</font>

