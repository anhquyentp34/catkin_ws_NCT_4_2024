#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def fake_position_publisher():
    rospy.init_node('fake_position_publisher')
    position_pub = rospy.Publisher('/person_position', Point, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    position = Point()
    position.x = 0.0
    position.y = 0.0
    position.z = 1.0

    while not rospy.is_shutdown():
        # Cập nhật vị trí giả lập
        position.x += 0.01
        position.y += 0.01
        position_pub.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_position_publisher()
    except rospy.ROSInterruptException:
        pass
