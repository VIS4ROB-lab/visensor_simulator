VI-Sensor Simulator
========================
The simulation of the VI-Sensor. No Guarantees.

This work is described in the letter "Aerial Single-View Depth Completion with Image-Guided Uncertainty Estimation", by Lucas Teixeira, Martin R.
Oswald, Marc Pollefeys, Margarita Chli, published in the IEEE
Robotics and Automation Letters (RA-L) [IEEE link](https://doi.org/10.1109/LRA.2020.2967296).

#### Video:
<a href="https://www.youtube.com/embed/IzfFNlYCFHM" target="_blank"><img src="http://img.youtube.com/vi/IzfFNlYCFHM/0.jpg" 
alt="Mesh" width="240" height="180" border="10" /></a>

#### Citations:
If you use this code for research, please cite the following publication:

```
@article{Teixeira:etal:RAL2020,
    title   = {{Aerial Single-View Depth Completion with Image-Guided Uncertainty Estimation}},
    author  = {Lucas Teixeira and Martin R. Oswald and Marc Pollefeys and Margarita Chli},
    journal = {{IEEE} Robotics and Automation Letters ({RA-L})},
    doi     = {10.1109/LRA.2020.2967296},
    year    = {2020}
}
```


License
------
there is copyright code from other libraries. Mostly commented in the source code.

Installation
------
* Requirements:
  * ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
  * Blender 2.80 or newer- (https://askubuntu.com/questions/110821/how-to-install-blender-from-the-official-website)

* Initialize catkin workspace (using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)):
```sh
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws  
  $ source /opt/ros/melodic/setup.bash
  $ catkin init  # initialize your catkin workspace
  $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin config --merge-devel
```
* Get the simulator and dependencies
```sh
  $ cd ~/catkin_ws/src
  $ sudo apt-get install liblapacke-dev python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev libopenexr-dev libopenblas-dev
  $ sudo apt-get install ros-melodic-joy ros-melodic-octomap-ros
  $ git clone git@github.com:catkin/catkin_simple
  $ git clone git@github.com:ethz-asl/rotors_simulator
  $ git clone git@github.com:ethz-asl/mav_comm
  $ git clone git@github.com:ethz-asl/eigen_catkin
  $ git clone git@github.com:ethz-asl/glog_catkin
  $ git clone git@github.com:ethz-asl/mav_control_rw
  $ pip install OpenEXR
  
  $ git clone git@github.com:VIS4ROB-lab/visensor_simulator.git -b blender_2.8  


```
* Build the workspace  
```sh
  $ catkin build
```

* Open your Blender, navigate to Edit > Preferences > Add-ons, and import the script "visensor_simulator/scripts/blender/visensor_sim_blender_addon.py" as an add-on by clicking on "Install..." (https://docs.blender.org/manual/en/latest/editors/preferences/addons.html)


Step-by-step
------
1. Create a project - this is a folder with any name. Configure the cameras and the waypoints In the folder resources there is a example "project_test.tar"

2. starts gazebo 
```sh
  $ roslaunch visensor_simulator uav_vi_blender.launch
```
3. run the back_end: 
```sh
  $ roslaunch visensor_simulator ros_backend.launch project_folder:="/home/lucas/data/test/project_testA"
```
4. open your scene on blender, select the camera, file->import->VISensor Simulator Project(*json), choose the file visim_project.json on your project.

5. Render. Quick render is faster, but it is less realistic.

6. run the bagcreator(namespace is optional):  
```sh 
  $ rosrun visensor_simulator visensor_sim_bagcreator.py --output_bag your_output.bag --project_folder "/home/lucas/data/test/project_testA" --namespace "firefly"
```

Roadmap
------
* write a camera path exporter compatible with our waypoint planner, better if we introduce this on our dataset format
* change from firefly to neo11
* develop a software to build a simplified version of the world to allow collision on the simulation. BVH and Octomap are options
* better error msg on the bagcreator when it is not possible to create the bagfile
* expose the simple_planner waypoint tolerance as ros parameters
* write my owm spawn with noise and vi_sensor pose as parameters
* add imu name on the json file
* autoselect a camera from the json
* add option to disable the simple planner
* detect incomplete render sequence and jump to the latest one (support in case of shutdown)
* make everything relative to the project file instead of the project folder
* add test for topic names on the bagcreator



