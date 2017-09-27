VI-Sensor Simulator
========================
The simulation of the VI-Sensor.   This is a very alpha version. No Guarantees.
**Please do not share without the autorization of the author**

License
------
???-- there is copyright code from other libraries. I still have to check or replace them.

Installation
------

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
  $ sudo apt-get install liblapacke-dev python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-indigo-joy ros-indigo-octomap-ros
  $ git clone git@github.com:catkin/catkin_simple
  $ git clone git@github.com:ethz-asl/rotors_simulator
  $ git clone git@github.com:ethz-asl/mav_comm
  $ git clone git@github.com:ethz-asl/eigen_catkin
  $ git clone git@github.com:ethz-asl/glog_catkin
  $ git clone git@github.com:ethz-asl/mav_control_rw
  
  $ git clone git@github.com:VIS4ROB-lab/visensor_simulator.git

```
* Build the workspace  
```sh
  $ catkin build
```

* Import the script "blender_camera_file_import.py" as an add-on in your blender (https://blender.stackexchange.com/questions/1688/installing-an-addon)

Step-by-step
------
1. starts gazebo 
```sh
  $ roslaunch visensor_simulator uav_vi_blender.launch
```
2. start to record ( at least /firefly/vi_sensor/ground_truth/pose and /firefly/vi_sensor/imu): 
```sh
  $ rosbag record -o your_bag.bag /firefly/vi_sensor/ground_truth/pose  /firefly/vi_sensor/imu
```
3. run our planner (or any other planner). The default waypoint file is in the resources folder, but your can give as argument for the launch file (waypoint_file:="/home/you/waypoints.txt"): 
```sh
  $ roslaunch visensor_simulator waypoint_planner.launch
```
4. publish a msg to send the uav to the initial position of the trajectory (you can use the rqt topic publisher)
```sh
  $ rostopic pub --once /firefly/command/pose/ geometry_msgs/PoseStamped \
  '{header: {stamp: now, frame_id: "world"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}'
```
5. stop the recording (Sanity checks: 1. use "rosbag info" to check if the number of msg from the two topic are the same. 2. use rqt_bag to see if the two topics are aligned in time)
6. extract the visensor imu poses: 
```sh
  $ rostopic echo -b your_bag.bag -p /firefly/vi_sensor/ground_truth/pose > vi_imu_poses.csv
```
7. extract the visensor imu measurements:
```sh
  $ rostopic echo -b your_bag.bag -p /firefly/vi_sensor/imu > vi_imu.csv
```
8. open blender, select the camera, file->import->Ros poses dump file(*csv), choose the file vi_imu_poses.csv
9. setup your scene objects and lights
10. set the Active viewport to the camera view(Numpad 0)
11. set the begining and end of the render in the timeline
12. render using OpenGL render of the active viewport, use the Material Shader as Display method.
13. copy the file vi_imu.csv to the folder blender_result_####date####
14. run the renamer: 
```sh
  $ python rename_blender_frames.py --folder /home/lucas/data/blender_test/blender_result_####date####
```
15. run the bagcreator:  
```sh 
  $ python kalibr_bagcreator.py --output-bag your_output.bag --folder /home/lucas/data/blender_test/blender_result_####date####/
```

Roadmap
------
* read camera comfiguration from another file, including a transformation between camera imu.
* suport multiple cameras
* write a camera path exporter compatible with our waypoint planner, better if we introduce this on our dataset format
* change from firefly to neo11
* develop a software to build a simplified version of the world to allow collision on the simulation. BVH and Octomap are options
* better simple planner.
* replace copyright code.

Camera Parameters
------
* T_SC is the transformation from Camera frame to IMU frame.
* The file vi_imu_poses.csv contains the ground truth pose in imu frame at each timestamp.
```
cameras:
    - {T_SC:
        [ 0, 0, 1, 0,
          -1, 0, 0, 0,
          0,-1, 0, 0,
          0, 0, 0, 1],
        image_dimension: [752, 480],
        distortion_coefficients: [0.0, 0.0, -0.0, 0.0],
        distortion_type: radialtangential,
        focal_length: [455, 455],
        principal_point: [376.5, 240.5]}

camera_rate: 20
imu_rate: 200
```
