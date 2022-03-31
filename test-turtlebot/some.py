# https://answers.ros.org/question/361940/ros-robot-moves-to-certain-coordinate-point-but-how-to-stop-and-set-multiple-goals/?answer=361984#post-id-361984

from nav_msgs.msg import Odometry 
from euler_from import euler_from_quaternion 
from geometry_msgs.msg import Point, Twist
from math import atan2
import rclpy
# https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
import threading

x = 0.0 
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation 
    (roll, pitch, theta) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

    
rclpy.init()
node = rclpy.create_node('speed_controller')
# https://docs.ros2.org/foxy/api/rclpy/api/qos.html
sub = node.create_subscription(Odometry, "/odometry/filtered", newOdom, 0)
pub = node.create_publisher(Twist, "/cmd_vel", 0)

speed = Twist()

rate = node.create_rate(4)

goal = Point ()
goal.x = 5.0 # x coordinate for goal
goal.y = 5.0 # y coordinate for goal

try:
    while rclpy.ok():
        inc_x = goal.x - x
        inc_y = goal.y - y 

        angle_to_goal = atan2 (inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3   
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        pub.publish(speed)
        rate.sleep()
except KeyboardInterrupt:
    pass

rclpy.shutdown()
# thread.join()
