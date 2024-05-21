#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from people_msgs.msg import People
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import math

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
            # Marker hình cầu biểu diễn vị trí
            sphere_marker = Marker()
            sphere_marker.header.frame_id = data.header.frame_id
            sphere_marker.header.stamp = rospy.Time.now()
            sphere_marker.ns = "people"
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = person.position.x
            sphere_marker.pose.position.y = person.position.y
            sphere_marker.pose.position.z = person.position.z
            sphere_marker.pose.orientation.x = 0.0
            sphere_marker.pose.orientation.y = 0.0
            sphere_marker.pose.orientation.z = 0.0
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.5
            sphere_marker.scale.y = 0.5
            sphere_marker.scale.z = 0.5
            sphere_marker.color.a = 1.0
            sphere_marker.color.r = 0.0
            sphere_marker.color.g = 1.0
            sphere_marker.color.b = 0.0
            self.markers.append(sphere_marker)

            # Marker mũi tên biểu diễn hướng di chuyển
            arrow_marker = Marker()
            arrow_marker.header.frame_id = data.header.frame_id
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "people_arrow"
            arrow_marker.id = i + len(data.people)  # Đảm bảo ID là duy nhất
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = person.position.x
            arrow_marker.pose.position.y = person.position.y
            arrow_marker.pose.position.z = person.position.z
            # Tính toán hướng của mũi tên dựa trên vận tốc
            arrow_marker.pose.orientation = self.get_orientation_from_velocity(person.velocity)
            arrow_marker.scale.x = 1.0  # Độ dài của mũi tên
            arrow_marker.scale.y = 0.1  # Độ dày của mũi tên
            arrow_marker.scale.z = 0.1  # Độ dày của mũi tên
            arrow_marker.color.a = 1.0
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 1.0
            arrow_marker.color.b = 0.0
            self.markers.append(arrow_marker)

    def get_orientation_from_velocity(self, velocity):
        # Tính toán góc yaw từ vận tốc
        yaw = 0-math.atan2(velocity.x, velocity.y)
        quat = quaternion_from_euler(0, 0, yaw)
        return Quaternion(*quat)

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
