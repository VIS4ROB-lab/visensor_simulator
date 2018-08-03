VI-Sensor Simulator
========================
The simulation of the VI-Sensor.   This is a alpha version. No Guarantees.
**Please do not share without the autorization of the author**

License
------
???-- there is copyright code from other libraries.

Installation
------
* Requirements:
  * ROS Indigo or Kinect - (http://wiki.ros.org/indigo/Installation/Ubuntu)
  * Blender 2.79 or newer- (https://askubuntu.com/questions/110821/how-to-install-blender-from-the-official-website)

* Initialize catkin workspace:
```sh
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws
  $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin init  # initialize your catkin workspace
```
* Get the simulator and dependencies
```sh
  $ cd ~/catkin_ws/src
  $ sudo apt-get install liblapacke-dev python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev libopenexr-dev
  $ sudo apt-get install ros-kinetic-joy ros-kinetic-octomap-ros (or ros-indigo-joy ros-indigo-octomap-ros)
  $ git clone git@github.com:catkin/catkin_simple
  $ git clone git@github.com:ethz-asl/rotors_simulator
  $ git clone git@github.com:ethz-asl/mav_comm
  $ git clone git@github.com:ethz-asl/eigen_catkin
  $ git clone git@github.com:ethz-asl/glog_catkin
  $ git clone git@github.com:ethz-asl/mav_control_rw
  $ pip install OpenEXR
  
  $ git clone git@github.com:VIS4ROB-lab/visensor_simulator.git
  $ git checkout devel


```
* Build the workspace  
```sh
  $ catkin build visensor_simulator
```

* Import the script "visensor_simulator/scripts/blender/visensor_sim_blender_addon.py" as an add-on in your blender (https://blender.stackexchange.com/questions/1688/installing-an-addon)


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



