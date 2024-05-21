#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class StartGoalPublisher:
    def __init__(self):
        rospy.init_node('start_goal_publisher')
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.publish_markers()

    def create_marker(self, x, y, z, ns, marker_id, marker_type, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        return marker

    def create_text_marker(self, x, y, z, text, ns, marker_id, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x 
        marker.pose.position.y = y+ 0.5 
        marker.pose.position.z = z  # Position above the marker
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.4  # Text height
        marker.color.r = 0.0  # Black color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = a
        marker.text = text
        return marker

    def publish_markers(self):
        while not rospy.is_shutdown():
            # Robot start point
            robot_start = self.create_marker(-8.0, 0.0, 0.0, "robot_start", 0, Marker.SPHERE, 0.0, 1.0, 0.0, 1.0)
            self.marker_pub.publish(robot_start)
            robot_start_text = self.create_text_marker(-8.0, 0.0, 0.0, "Robot Start", "robot_start_text", 10, 0.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(robot_start_text)

            # Robot goal point
            robot_goal = self.create_marker(5.0, 0.0, 0.0, "robot_goal", 1, Marker.SPHERE, 1.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(robot_goal)
            robot_goal_text = self.create_text_marker(5.0, 0.0, 0.0, "Robot Goal", "robot_goal_text", 11, 0.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(robot_goal_text)

            # Person start point
            person_start = self.create_marker(-1.0, 4.0, 0.0, "person_start", 2, Marker.SPHERE, 0.0, 0.0, 1.0, 1.0)
            self.marker_pub.publish(person_start)
            person_start_text = self.create_text_marker(-1.0, 4.0, 0.0, "Person Start", "person_start_text", 12, 0.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(person_start_text)

            # Person goal point
            person_goal = self.create_marker(-1.0, -4.0, 0.0, "person_goal", 3, Marker.SPHERE, 1.0, 1.0, 0.0, 1.0)
            self.marker_pub.publish(person_goal)
            person_goal_text = self.create_text_marker(-1.0, -4.0, 0.0, "Person Goal", "person_goal_text", 13, 0.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(person_goal_text)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        StartGoalPublisher()
    except rospy.ROSInterruptException:
        pass
