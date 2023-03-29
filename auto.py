import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Twist
import math
from RobotClass import Robot
import time
from tf2_msgs.msg import TFMessage

robot = Robot()

prev_linear = 0
prev_angular = 0
distance = 0
prev_calc = time.time()

def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z)

# print("Waiting for odom message...")
# odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
# euler_x, euler_y, euler_z = euler_from_quaternion(odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z, odo_msg.pose.pose.orientation.w)
# print(f"Current angle: {euler_z}, current x: {odo_msg.pose.pose.position.x}, current y: {odo_msg.pose.pose.position.y}")

# print("Waiting for point message...")
# point_msg = rospy.wait_for_message('/clicked_point', PointStamped)
# goal_x, goal_y = point_msg.point.x, point_msg.point.y
# print(f"Goal x: {goal_x}, Goal y: {goal_y}")

# change_in_x = goal_x - odo_msg.pose.pose.position.x
# change_in_y = goal_y - odo_msg.pose.pose.position.y
# print(f"Change in x: {change_in_x}, Change in y: {change_in_y}")

# goal_angle = math.degrees(math.atan2(change_in_y, change_in_x))
# print(f"Goal angle: {goal_angle}, changed needed: {goal_angle - euler_z}")

# while round(goal_angle - euler_z, 0) != 0:
#     odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
#     euler_x, euler_y, euler_z = euler_from_quaternion(odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z, odo_msg.pose.pose.orientation.w)
#     print(f"Current angle: {euler_z}, goal angle: {goal_angle}, delta: {goal_angle - euler_z}")
#     if goal_angle - euler_z < 0:
#         # print(f"Right : {goal_angle - euler_z}")
#         robot.right(0.33)
#     else:
#         # print(f"Left : {goal_angle - euler_z}")
#         robot.left(0.33)
#     time.sleep(0.1)
#     robot.stop()
# print(f"done, final deviation: {goal_angle - euler_z}")







def pointcb(point_msg):
    print("Received goal point")

    def turn_to_goal(goal_x, goal_y):
        print("Turning to goal point")
        odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
        euler_x, euler_y, euler_z = euler_from_quaternion(odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z, odo_msg.pose.pose.orientation.w)
        print(f"Current angle: {euler_z}, current x: {odo_msg.pose.pose.position.x}, current y: {odo_msg.pose.pose.position.y}")

        
        print(f"Goal x: {goal_x}, Goal y: {goal_y}")

        change_in_x = goal_x - odo_msg.pose.pose.position.x
        change_in_y = goal_y - odo_msg.pose.pose.position.y
        print(f"Change in x: {change_in_x}, Change in y: {change_in_y}")

        goal_angle = math.degrees(math.atan2(change_in_y, change_in_x))
        print(f"Goal angle: {goal_angle}, changed needed: {goal_angle - euler_z}")

        while round(goal_angle - euler_z, 0) != 0:
            odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
            euler_x, euler_y, euler_z = euler_from_quaternion(odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z, odo_msg.pose.pose.orientation.w)
            print(f"Current angle: {euler_z}, goal angle: {goal_angle}, delta: {goal_angle - euler_z}")
            if goal_angle - euler_z < 0:
                # print(f"Right : {goal_angle - euler_z}")
                robot.right(0.35)
            else:
                # print(f"Left : {goal_angle - euler_z}")
                robot.left(0.35)
            time.sleep(0.1)
            robot.stop()
        print(f"done, final deviation: {goal_angle - euler_z}")


    def move_to_goal(goal_x, goal_y):
        print("Moving to goal point")
        odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
        change_in_x = goal_x - odo_msg.pose.pose.position.x
        change_in_y = goal_y - odo_msg.pose.pose.position.y
        print(f"Change in x: {change_in_x}, Change in y: {change_in_y}")

        prev_distance = 1000
        distance = math.sqrt(change_in_x**2 + change_in_y**2)
        while distance > 0.1:
            odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
            change_in_x = goal_x - odo_msg.pose.pose.position.x
            change_in_y = goal_y - odo_msg.pose.pose.position.y
            distance = math.sqrt(change_in_x**2 + change_in_y**2)
            deviation = distance - prev_distance
            if deviation > 0.01:
                print(f"Overshoot by distance: {distance - prev_distance}")
                return False
            prev_distance = distance
            print(f"Current distance: {distance}")
            robot.forward(0.3)
        print(f"done, final deviation: {distance}")
        return True

    goal_x, goal_y = point_msg.point.x, point_msg.point.y
    turn_to_goal(point_msg.point.x, point_msg.point.y)
    while not move_to_goal(goal_x, goal_y):
        turn_to_goal(point_msg.point.x, point_msg.point.y)
    robot.stop()


def loco_cb(data):
    print(f"Linear: {data.linear.x}, Angular: {data.angular.z}")
    global prev_linear, prev_angular
    if data.linear.x != prev_linear:
        prev_linear = data.linear.x
        robot.forward(data.linear.x)
    if data.angular.z != prev_angular:
        prev_angular = data.angular.z
        robot.left(data.angular.z)


def listener():
    rospy.init_node('listener', anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Listener node initialized, subscribing to loco...")
    rospy.Subscriber("loco", Twist, loco_cb)
    rospy.loginfo("Subscribed, subscribing to goto...")
    rospy.Subscriber('goto', PointStamped, pointcb)
    rospy.loginfo("Subscribed, spinning...")
    rospy.spin()
    rospy.loginfo("Spunned")

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        robot.stop()
        rospy.signal_shutdown("KeyboardInterrupt")