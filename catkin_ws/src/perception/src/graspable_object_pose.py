#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pub_object_pose_ecst import SurfaceObjectFilter

# Here we are performing object detection for an object placed on top of a cafe table mesuring 78 cm
if __name__ == "__main__":
    rospy.init_node("object_detection_node", log_level=rospy.INFO)

    try:
        SurfaceObjectFilter(table_height_init=0.78, error_height=0.10).run()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
