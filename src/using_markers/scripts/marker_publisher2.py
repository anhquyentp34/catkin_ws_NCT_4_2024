#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher:
    def __init__(self):
        rospy.init_node('marker_publisher')
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('/people ', Point, self.position_callback)
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.ns = "people"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2  # Kích thước của marker
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.a = 1.0  # Độ trong suốt
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0  # Màu xanh lá cây
        self.marker.color.b = 0.0
        self.rate = rospy.Rate(10)  # 10 Hz
        self.run()

    def position_callback(self, data):
        self.marker.pose.position = data

    def run(self):
        while not rospy.is_shutdown():
            self.marker.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.marker)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MarkerPublisher()
    except rospy.ROSInterruptException:
        pass
