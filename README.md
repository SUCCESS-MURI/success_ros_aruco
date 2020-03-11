# success_ros_aruco
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu  

A simple ROS wrapper for the [ArUco](https://www.uco.es/investiga/grupos/ava/node/26) Library.
It subscribes to the rostopics `/image` and `/camera info` and output any detected markers in `/markers` and `/marker_poses`. For most usage, you will need to remap the topics to correct ones. Please refer to the launch files for example.

## Interface
We also provide an Python API interface
```
module = ArucoTagModule()

list_of_ids = module.get_seen_ids()

pose = module.get_pose_for_id(10, duration=None, frame_id='base')
```


## Changelog
* 03/11/2020
    * Updated launch files to rely on each other and decreased repeats.
    * Changed the error message for when the image is not up-to-date to be more descriptive.
    * Change the node to be lazy subscribe to the camera topics. It will only connect if there are nodes subscribed to it.
    * Added a python script for testing