#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_marker():
    rospy.init_node('marker_publisher')
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "people"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0  # Thay đổi giá trị x, y, z để cập nhật vị trí
        marker.pose.position.y = 1.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Kích thước của marker
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Độ trong suốt
        marker.color.r = 0.0
        marker.color.g = 1.0  # Màu xanh lá cây
        marker.color.b = 0.0

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        create_marker()
    except rospy.ROSInterruptException:
        pass
