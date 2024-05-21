#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from people_msgs.msg import People

class MarkerPublisher:
    def __init__(self):
        rospy.init_node('marker_publisher')
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('/people', People, self.people_callback)
        self.markers = []
        self.rate = rospy.Rate(10)  # 10 Hz

    def people_callback(self, data):
        self.markers = []
        for i, person in enumerate(data.people):
            marker = Marker()
            marker.header.frame_id = data.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "people"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = person.position.x
            marker.pose.position.y = person.position.y
            marker.pose.position.z = person.position.z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.markers.append(marker)

    def run(self):
        while not rospy.is_shutdown():
            for marker in self.markers:
                marker.header.stamp = rospy.Time.now()
                self.marker_pub.publish(marker)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        MarkerPublisher().run()
    except rospy.ROSInterruptException:
        pass
