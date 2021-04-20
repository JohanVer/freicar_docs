 # FreiCAR Code Testing Guidelines

*UNDER CONSTRUCTION*

### General:

You should provide one of the two options (first one preferred)

#### Option 1 

Instructions on how to clone your codebase from within the 20.04 docker image provided with the course (in a `README.md` file). 
All setting-up should be done in a `setup_env.sh` without the need to additionally manually execute any commands. 
In the case that modifications to the docker image need to be integrated (i.e. building system libraries not present in the image), 
you can include the specific commands (installing, building, ...) in the `setup_env.sh` script. Note that this setup-process must not exceed a few minutes due to temporal
limitations during the competition. If your setup process needs more time than that, see Option 2.


#### Option 2 
Your custom built docker image. If your system depends on many libraries that are not present in the docker image provided by us, you can send us the complete build docker image
However, please note that the image including the simulator and all other libraries can easily weigh dozends of GB and may take hours to upload.


Regardless of the option, the README file should also explain how to start your system  (ideally, you could put all necessary startup stuff in a `start_system.sh` script).


### Hints

Please test the complete setup process on a blank testing machine (no traces of your system present on that machine) in order to simulate what the setup will be like
on our competition machine. Thorough testing will save valuable time during the competition. 

If the setup process requires a lot of intervention during the competition, this can reflect on your final marks.

