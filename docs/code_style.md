# FreiCar Code Guidelines

Always make your own **private** repository for your software. No changes to the submodules ``` freicar_base```, ```freicar_exercises``` or ```freicar_executables``` are allowed.
 
If you want to use code from these submodules you are allowed to copy these nodes, improve them, rename them and push them in your repository. 

Note: Do not forget to rename the nodes in case you copied something over to your private repository. Otherwise there will be **name conflicts** since there would be multiple nodes with the same name.

# Code Submission 
For the final submission (or qualification) you should provide a link to your private git repository of your code.
For the repository the following requirements need to be taken into account:

1. In the root of the cloned repository there has to be a `setup_env.sh` script that does all setting-up (like downloading/compiling any needed libraries or setting up system and environment variables) without the need to additionally manually execute any commands. In the case that modifications to the docker image need to be integrated (i.e. building system libraries not present in the image), 
you can include the specific commands (installing, building, ...) in the `setup_env.sh` script.
2. In the root of the cloned repository there has to be a `start.sh` bash script that starts your nodes/launch-files of your autonomous system. After starting this script your car should be in stop-mode. 
Further, this bash script **must** take exactly one argument that defines the name of your car (we might give your car a different name than the name you are using).
3. The simulator will be started by us (before starting your `start.sh` script). So the ```local_comp_launch.launch``` file must not be started by any of your scripts.

Overall on our competition server the following commands are executed:

1. Start clean docker with no committed changes. 
2. `roslaunch freicar_launch local_comp_launch.launch` (with comp_mode=true, separate terminal)
3. `roscd && cd ../src`
4. `git clone YOUR_REPO_ADDRESS submission_repo`
5. `cd submission_repo`
6. `./setup_env.sh`
7. `source ~/.bashrc`
8. `roscd && cd .. && catkin build`
9. `./start.sh`

During the complete process no further interaction will be possible.

### Hints

Please test the complete setup process on a blank testing machine (no traces of your system present on that machine) in order to simulate what the setup will be like
on our competition machine. Thorough testing will save valuable time during the competition. 

If the setup process requires a lot of intervention during the competition, this will reflect on your final marks.

