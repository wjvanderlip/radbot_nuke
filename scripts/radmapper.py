#!/usr/bin/env python


import logging

import rospy
import rospkg
import tf

def __init__(self):
    self.my_uid              = self.set_uid()
    self.tf1_listener = tf.TransformListener()  ## do this
    self.tf1_listener.waitForTransform(self.global_frame, self.baselink_frame, rospy.Time(0), rospy.Duration(35.0))  ## do this

    rospy.Subscriber("/detector_data", detector_data, self.intensity_loc)


def intensity_loc(self, data):
    for detection in data.detections:
        for result in detection.results:
            if result.id in self.target_list:
                obj_pose_stamped = PoseStamped()
                obj_pose_stamped.header = detection.header
                obj_pose_stamped.pose = result.pose.pose
                obj_pose_utm = self.tf1_listener.transformPose("utm", obj_pose_stamped)  ## do this !!!!
                ## from sensor to map frame
                ## returns X Y Z and orientation

                (obj_latitude,obj_longitude) = UTMtoLL(23, obj_pose_utm.pose.position.y, obj_pose_utm.pose.position.x, self.zone) # 23 is WGS-84.
                self.takserver.send(mkcot.mkcot(cot_identity="neutral",
                    cot_stale = 1,
                    cot_type="a-f-G-M-F-Q",
                    cot_how="m-g",
                    cot_callsign=result.id,
                    cot_id="object",
                    team_name="detector",
                    team_role="obj detector",
                    cot_lat=obj_latitude,
                    cot_lon=obj_longitude ))
