import rospy
from geometry_msgs.msg import Twist

rospy.init_node('turn_left')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)
twist = Twist()
twist.linear.x = 0.1
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0
for i in range(10):
    pub.publish(twist)
    rate.sleep()

# task finished
