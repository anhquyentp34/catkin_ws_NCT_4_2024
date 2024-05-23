#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import cos, sin, radians

class StartGoalPublisher:
    def __init__(self):
        rospy.init_node('start_goal_publisher')
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.publish_markers()

    def create_star_marker(self, x, y, z, ns, marker_id, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Define the points of the star
        points = []
        for angle in range(0, 360, 72):
            outer_angle = radians(angle)
            inner_angle = radians(angle + 36)
            points.append(Point(x + 0.5 * cos(outer_angle), y + 0.5 * sin(outer_angle), z))
            points.append(Point(x + 0.2 * cos(inner_angle), y + 0.2 * sin(inner_angle), z))
        points.append(points[0])  # Close the star shape

        marker.points = points
        marker.scale.x = 0.1  # Line width
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
        marker.pose.position.y = y + 0.5
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
            robot_start = self.create_star_marker(-1.0, -5.0, 0.0, "robot_start", 0, 0.0, 1.0, 0.0, 1.0)
            self.marker_pub.publish(robot_start)
            robot_start_text = self.create_text_marker(-1.0, -5.0, 0.0, "Robot Start", "robot_start_text", 10, 0.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(robot_start_text)

            # Robot goal point
            robot_goal = self.create_star_marker(-1.0, 5.0, 0.0, "robot_goal", 1, 1.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(robot_goal)
            robot_goal_text = self.create_text_marker(-1.0, 5.0, 0.0, "Robot Goal", "robot_goal_text", 11, 0.0, 0.0, 0.0, 1.0)
            self.marker_pub.publish(robot_goal_text)

            # Person start point
            # person_start = self.create_star_marker(-1.0, 4.0, 0.0, "person_start", 2, 0.0, 0.0, 1.0, 1.0)
            # self.marker_pub.publish(person_start)
            # person_start_text = self.create_text_marker(-1.0, 4.0, 0.0, "Person Start", "person_start_text", 12, 0.0, 0.0, 0.0, 1.0)
            # self.marker_pub.publish(person_start_text)

            # Person goal point
            # person_goal = self.create_star_marker(-1.0, -4.0, 0.0, "person_goal", 3, 1.0, 1.0, 0.0, 1.0)
            # self.marker_pub.publish(person_goal)
            # person_goal_text = self.create_text_marker(-1.0, -4.0, 0.0, "Person Goal", "person_goal_text", 13, 0.0, 0.0, 0.0, 1.0)
            # self.marker_pub.publish(person_goal_text)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        StartGoalPublisher()
    except rospy.ROSInterruptException:
        pass
