# FreiCAR Environment

Complex robots always involve large software stacks and setup procedures. 

To ease the process as much as possible we provide a complete docker container that includes everything you need, such as the simulator, libraries and IDEs.

# Installation
To setup everything on your pc you will need:

- A recent pc with a Nvidia GPU (better than GTX 1050 ti)
- Ubuntu >=18.04

In the following we describe the individual steps to make your pc ready!

- Install the latest nvidia driver on your pc. We recommend to use at least version 450 (the newer the better).
    - `sudo apt-get purge nvidia*` get rid of your old driver
    - `sudo apt-get install nvidia-driver-460`
    - restart your pc
- Clone the following repo to your pc (the hdd should have 40 gb free)
    - `git clone --recurse-submodules https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_docker.git`
- enter the cloned repo : `cd freicar_docker`
- install docker-ce and nvidia-docker with our script: `./install_docker.bash`
- build the entire environment and download the simulator (this will take a while, >1h): `./setup.bash` (You will be asked for your sudo password at some point in order to install the docker image to your pc)
- Restart your computer (otherwise the docker deamon might not be started)
- You are all set! :) 

## Structure of ROS Workspace
Within the FreiCar container a ROS Workspace `freicar_ws`  will already be setup. In the workspace three git submodules are placed that contain several ROS nodes `freicar_base`, `freicar_executables` and `freicar_exercises`.

The `freicar_base` submodule contains many ROS modules that you need during the course. All modules in `freicar_executables` are closed-source. The module `freicar_exercises` will contain all template code that is necessary for the exercises. 

NOTE: If there are updates to any submodules in the `freicar_docker` repository (e.g if there is new exercise code available) you need to pull the `freicar_docker` repository using : `git submodule update --init --recursive` . 

## Usage
Our scripts adds a few aliases to your bashrc. So after building everything close every open terminal so that on the next start everything gets sourced correctly.

Our docker container will provide most of the tools you need in a single container.
Using the container feels like a virtual machine that does not have the performance limitations of traditional VMs.

* Everytime when you want to use the environment you need start our container:
Therefore type `fcc` (freicar container)in a new terminal. It will ask you for the password of the user freicar. Guess what, it is freicar. This container must run only in one terminal at a time!

* Everytime you want to open additional terminals that interact with the containered environment just open a new terminal and type `fct` (freicar terminal). You will notice that the user changes to freicar.

The directory freicar_deps and freicar_ws are shared with your host. So every data that you create inside the docker within this directory can be read by the host, or vice versa. Additionally only the data in these folders are persistent if you restart the docker environment. 


## Persistent Changes to Docker Container
Sometimes it becomes necessary to install additional libraries or packages if your program needs them. While you can install them in the docker container with `sudo apt-get install`, the changes would vanish as soon you stop the docker.

However you can make these changes persistent by executing the script `commit_changes.bash` before closing the docker container. This will keep all changes you made and the next time you start up the container (with `fcc`) you will have changed environment.

NOTE: Always keep track what you changed in the container as you will need a bash script for the final competition that install all required additional libraries on a fresh docker. (See [Code Style](https://freicar-docs.readthedocs.io/code_style/) section )

## IDEs
In the container there are two IDEs pre-installed: pycharm and clion. The first is for developing in python and the latter in C++.
Inside the docker you can simply type `pycharm` or `clion` to open the respective IDE. 

Note in case of clion that you should register for a educational licence [here](https://www.jetbrains.com/community/education/#students) in order to be able to use it.