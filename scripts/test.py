#!/usr/bin/env python

import rospy
from success_ros_aruco import ArucoTagModule

if __name__ == "__main__":
    rospy.init_node("aruco_test")

    module = ArucoTagModule()

    list_of_ids = module.get_seen_ids()
    print("Found: {}".format(list_of_ids))
    module.wait_for_id(17)
    print(module.get_pose_for_id(17, duration=None, frame_id='head_camera'))
