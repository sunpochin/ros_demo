# https://answers.ros.org/question/361940/ros-robot-moves-to-certain-coordinate-point-but-how-to-stop-and-set-multiple-goals/?answer=361984#post-id-361984

from nav_msgs.msg import Odometry 
from euler_from import euler_from_quaternion 
from geometry_msgs.msg import Point, Twist
from math import atan2
import rclpy
# https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
import threading
import numpy as np


x = 0.0 
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # print('x: ', x, ', y:', y, ' msg post:', msg.pose.pose.position)

    rot_q = msg.pose.pose.orientation 
    (roll, pitch, theta) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

    
rclpy.init()
node = rclpy.create_node('speed_controller')
# https://docs.ros2.org/foxy/api/rclpy/api/qos.html
sub = node.create_subscription(Odometry, "/demo/odom_demo", newOdom, 0)
pub = node.create_publisher(Twist, "/demo/cmd_demo", 0)

# sub = node.create_subscription(Odometry, "/odometry/filtered", newOdom, 0)
# pub = node.create_publisher(Twist, "/cmd_vel", 0)

speed = Twist()

# Spin in a separate thread
thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
thread.start()

rate = node.create_rate(4)


path_list = [(3, 3)]
point_index = 0  # instead of deleting stuff from a list (which is anyway bug prone) we'll just iterate through it using index variable.
goal = Point()

while rclpy.ok():
    if point_index < len(path_list): # so we won't get an error of trying to reach non-existant index of a list
        goal.x = float(path_list[point_index][0]) # x coordinate for goal
        goal.y = float(path_list[point_index][1]) # y coordinate for goal
    else:
        speed.linear.x = 0.0
        speed.linear.y = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        print("break")
        break # I guess we're done?

    inc_x = goal.x - x
    inc_y = goal.y - y 

    angle_to_goal = atan2(inc_y, inc_x)

    distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)
    print("distance_to_goal: ", distance_to_goal)
    if (distance_to_goal >= 0.6):
        goal_theta = angle_to_goal - theta
        # print("angle_to_goal: ", angle_to_goal, ", theta:", theta, ", goal_theta: ", goal_theta)
        if abs(goal_theta) > 0.2:
            speed.linear.x = 0.0
            speed.angular.z = 0.1
        else:
            speed.linear.x = 0.3
            speed.angular.z = 0.0
        pub.publish(speed)
    else:
        point_index += 1
        print("point_index: ", point_index)
    rate.sleep()
rclpy.shutdown()
print("rclpy.shutdown() ")
thread.join()
