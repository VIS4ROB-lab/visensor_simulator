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
  $ git checkout lite_v1


```
* Build the workspace  
```sh
  $ catkin build
```

* Import the script "blender_visensor_sim_tool_lite.py" as an add-on in your blender (https://blender.stackexchange.com/questions/1688/installing-an-addon)

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
4. open blender, select the camera, file->import->VISim Project(*json), choose the file visim_project.json on your project
5. setup your scene objects and lights
6. render. OpenGL render is faster, use the Material Shader as Display method.

7. run the bagcreator(namespace is optional):  
```sh 
  $ python (path to visensor-simulator package)/scripts/bagcreator_lite.py --output your_output.bag --project_folder "/home/lucas/data/test/project_testA" --namespace "firefly"
```

Roadmap
------
* write a camera path exporter compatible with our waypoint planner, better if we introduce this on our dataset format
* change from firefly to neo11
* develop a software to build a simplified version of the world to allow collision on the simulation. BVH and Octomap are options


