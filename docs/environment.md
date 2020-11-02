# FreiCAR Environment

Complex robots always involve large software stacks and setup procedures. 

To ease the process as much as possible we provide a complete docker container that includes everything you need, such as the simulator, libraries and IDEs.

###Setup
To setup everything on your pc you will need:
1. A recent pc with a nvidia GPU
2. Ubuntu 18.04

In the following we describe the individual steps to make your pc ready!
* Install the latest nvidia driver on your pc. We recommend to use at least version 450.
  * `sudo apt-get purge nvidia*` get rid of your old driver
  * `sudo apt-get install nvidia-driver-455`
  * restart your pc
* Clone the following repo to your pc (the hdd should have 30 gb free)
  * `git clone https://aisgit.informatik.uni-freiburg.de/vertensj/freicar_docker.git`
  * enter the cloned repo : `cd freicar_docker`
  * install docker-ce and nvidia-docker with our script: `./install_docker.bash`
  * build the entire environment and download the simulator (this will take a while): `./setup.bash`
* Restart your computer (otherwise the docker deamon might not be started)
* You are all set! :) 

###Usage
Our scripts add a few aliases to your bashrc. So after building everything close every open terminal so that on the next start everything gets sourced correctly.

Our docker container will provide most of the tools you need in a single container.
Using the container feels like a virtual machine that does not have the performance limitations of traditional VMs.

* Everytime when you want to use the environment you need start our container:
Therefore type `fcc` in a new terminal. It will ask you for the password of the user freicar. Guess what, it is freicar. This container must run only in one terminal at a time!

* Everytime you want to open additional terminals that interact with the containered environment just open a new terminal and type `fct`. You will notice that the user changes to freicar.

The directory freicar_deps and freicar_ws are shared with your host. So every data that you create inside the docker within this directory can be read by the host, or vice versa. Additionally only the data in these folders are persistent if you restart the docker environment. 



