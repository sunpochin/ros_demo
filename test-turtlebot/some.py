from nav_msgs.msg import Odometry 
from euler_from import euler_from_quaternion 
from geometry_msgs.msg import Point, Twist
from math import atan2
import rclpy


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

sub = node.create_subscription(Odometry, "/odometry/filtered", newOdom)
pub = node.create_publisher(Twist, "/cmd_vel", queue_size=1)

speed = Twist()

r = speed.Rate(4)

goal = Point ()
goal.x = 5 # x coordinate for goal
goal.y = 5 # y coordinate for goal

while not sub.is_shutdown():
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

    r.sleep()