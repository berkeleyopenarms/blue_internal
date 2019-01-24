#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from blue_msgs.msg import GravityVectorArray
import numpy as np
import tf.transformations as transformations

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def get_accel(msg):
    global publisher

    markerArray = MarkerArray()
    # rospy.logerr(msg.frame_ids)
    for frame, vec in zip(msg.frame_ids, msg.vectors):
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points.append(Point(0,0,0))
        marker.points.append(Point(vec.x,vec.y,vec.z))

        markerArray.markers.append(marker)

    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    # rospy.logerr(markerArray)
    publisher.publish(markerArray)


#################################################################################################

def main():
    global publisher

    rospy.init_node('acc_recorder', anonymous=True)

    topic = 'visualization_marker_array'
    publisher = rospy.Publisher(topic, MarkerArray)

    rospy.Subscriber("/left_arm/blue_hardware/gravity_vectors", GravityVectorArray, get_accel, queue_size=1)

    while not rospy.is_shutdown():
        rospy.sleep(0.01)


if __name__ == '__main__':
    main()
