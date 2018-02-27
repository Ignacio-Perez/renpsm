# RENPSM

A software simulator for Random Enzymatic Numerical P Systems with Shared Memory (RENPSM) 
 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. The recommended OS is Ubuntu 16.04.3 Desktop 64 bits.

### Prerequisites

* [ROS Kinetic Framework](http://wiki.ros.org/kinetic) 
* [MobileSim Software](http://robots.mobilerobots.com/wiki/MobileSim)
* [ROSARIA node](http://wiki.ros.org/ROSARIA)
* [AMR ROS](https://github.com/MobileRobots/amr-ros-config) 


### Installing

* Install the ROS kinetic framework following the next [instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu)

* Create a Catkin Workspace following the next [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

* Download and install MobileSim following the next [instructions](http://robots.mobilerobots.com/wiki/MobileSim)

* Install ROSARIA node following the next [tutorial](http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA)

* Clone the [AMR ROS](https://github.com/MobileRobots/amr-ros-config) repository in your *$HOME/catkin_ws/src* directory

* Clone the [RENPSM](https://github.com/Ignacio-Perez/renpsm) repository in your your *$HOME/catkin_ws/src* directory

* Run [catkin make](http://wiki.ros.org/catkin/commands/catkin_make)


## Running the tests

### Bidirectional RRT algorithm in the ARM Office

#### Run MobileSim
* Select the robot model *p3dx-sh-lms1xx*
* Select the map *AMROffice.map*

#### Launch renpsm_office.launch
* *roslaunch renpsm renpsm_office.launch*

#### In the Rviz GUI, select a navigation goal to see the visualization

### Bidirectional RRT algorithm in a virtual environment

#### Run MobileSim
* Select the robot model p3dx-sh-lms1xx
* Select the map map1.map

#### Launch renpsm_rrt.launch
* roslaunch renpsm renpsm_rrt.launch

#### In the Rviz GUI, select a navigation goal to see the visualization


### Benchmark in a virtual environment

#### Run MobileSim
* Select the robot model p3dx-sh-lms1xx
* Select the map map1.map

#### Launch renpsm_rrt1.launch
* roslaunch renpsm renpsm_rrt1.launch

#### The benchmarking results will be shown in the standard output.


## Authors

* **Ignacio Perez-Hurtado** - *Initial development* - [Ignacio-Perez](https://github.com/Ignacio-Perez)


## License

This project is licensed under the GPL v3.0 License - see the [LICENSE.md](LICENSE.md) file for details



