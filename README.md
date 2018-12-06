# success_ros_aruco
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu  

A simple ROS wrapper for the [ArUco](https://www.uco.es/investiga/grupos/ava/node/26) Library.
It takes the rostopics `/image` and `/camera info` and output any detected markers in `/markers`. Please refer to the launch file for example.


## Interface
We also provide an Python API interface
```
module = ArucoTagModule()

list_of_ids = module.get_seen_ids()

pose = module.get_pose_for_id(10, duration=None, frame_id='base')
```