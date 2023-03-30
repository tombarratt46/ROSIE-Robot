import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String, Empty
import math
from RobotClass import Robot
import time
from tf2_msgs.msg import TFMessage
from typing import Tuple

robot = Robot()
emergency_stop = False
collision_stop = False
is_idle = True

prev_linear = 0
prev_angular = 0
distance = 0
prev_calc = time.time()


rospy.init_node('listener', anonymous=True, log_level=rospy.INFO)

status = "Ready"

def euler_from_quaternion(x : float, y : float, z : float, w : float) -> Tuple[float, float, float]:
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


def turn_to_goal(goal_x : float, goal_y : float) -> None:
    global emergency_stop, status, is_idle
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
    status = f"Rotation Remaining : {goal_angle - euler_z} degrees"
    while (abs(goal_angle - euler_z) >= 1) and (not emergency_stop):
        is_idle = False
        odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
        euler_x, euler_y, euler_z = euler_from_quaternion(odo_msg.pose.pose.orientation.x, odo_msg.pose.pose.orientation.y, odo_msg.pose.pose.orientation.z, odo_msg.pose.pose.orientation.w)
        print(f"Current angle: {euler_z}, goal angle: {goal_angle}, delta: {goal_angle - euler_z}")
        status = f"Rotation Remaining : {goal_angle - euler_z} degrees"
        if goal_angle - euler_z < 0:
            # print(f"Right : {goal_angle - euler_z}")
            robot.right(0.35)
        else:
            # print(f"Left : {goal_angle - euler_z}")
            robot.left(0.35)

        amount = abs(goal_angle - euler_z) / 20
        rospy.sleep(min(amount, 5))
        robot.stop()
        print(emergency_stop)
    if emergency_stop:
        print("Stopping due to emergency")
    else:
        print(f"done, final deviation: {goal_angle - euler_z}")

def move_to_goal(goal_x : float, goal_y : float) -> bool:
    global emergency_stop, status, is_idle
    print("Moving to goal point")
    odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
    change_in_x = goal_x - odo_msg.pose.pose.position.x
    change_in_y = goal_y - odo_msg.pose.pose.position.y
    print(f"Change in x: {change_in_x}, Change in y: {change_in_y}")

    prev_distance = 1000
    distance = math.sqrt(change_in_x**2 + change_in_y**2)
    while (distance > 0.1) and (not emergency_stop):
        is_idle = False
        status = f"Distance Remaining : {distance} meters"
        odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
        change_in_x = goal_x - odo_msg.pose.pose.position.x
        change_in_y = goal_y - odo_msg.pose.pose.position.y
        distance = math.sqrt(change_in_x**2 + change_in_y**2)
        deviation = distance - prev_distance
        if deviation > 0.10:
            print(f"Overshoot by distance: {distance - prev_distance}")
            robot.stop()
            return False
        prev_distance = distance
        print(f"Current distance: {distance}")
        robot.forward(0.25)

        amount = distance * 3
        rospy.sleep(min(amount, 5))
        robot.stop()
    if emergency_stop:
        print("Stopping due to emergency")
    else:
        print(f"done, final deviation: {distance}")
    return True

def pointcb(point_msg : PointStamped) -> None:
    global emergency_stop, status, is_idle
    is_idle = False
    print("Received goal point")

    goal_x, goal_y = point_msg.point.x, point_msg.point.y
    turn_to_goal(point_msg.point.x, point_msg.point.y)
    while (not move_to_goal(goal_x, goal_y)) and (not emergency_stop):
        turn_to_goal(point_msg.point.x, point_msg.point.y)
        status = "Turning Complete, Moving to Goal Point"
        rospy.sleep(0.1)
    status = "Moving Complete"
    robot.stop()
    emergency_stop = False
    is_idle = True


def loco_cb(data : Twist) -> None:
    global emergency_stop, collision_stop, status, is_idle
    is_idle = False
    print(f"Linear: {data.linear.x}, Angular: {data.angular.z}, Emergency_stop: {emergency_stop}")
    global prev_linear, prev_angular
    if emergency_stop:
        print("Emergency stop active, not moving")
        prev_linear = 0
        prev_angular = 0
        robot.stop()
        return
    if data.linear.x != prev_linear:
        if data.linear.x > 0:
            status = f"Moving forward at speed {data.linear.x}"
        elif data.linear.x < 0:
            status = f"Moving backward at speed {-data.linear.x}"
            
        prev_linear = data.linear.x
        robot.forward(data.linear.x)
    if data.angular.z != prev_angular:
        if data.angular.z < 0:
            status = f"Turning right at speed {-data.angular.z}"
        elif data.angular.z > 0:
            status = f"Turning left at speed {data.angular.z}"
        prev_angular = data.angular.z
        robot.left(data.angular.z)

def emergency_cb(data : Empty) -> None:
    global emergency_stop, status, is_idle

    status = f"E-Stop Received"
    if not is_idle:
        print("Emergency stop received, stopping robot")
        emergency_stop = True
    #robot.stop()

def collision_cb(data : Empty) -> None:
    global collision_stop, emergency_stop, status
    status = f"Possible Collision Detected"
    print("Collision detected, stopping robot")
    collision_stop = True
    emergency_stop = True
    #robot.stop()

def scan_cb(data : Empty) -> None:
    global emergency_stop, status, is_idle
    is_idle = False
    rotation_count = 100
    print("Scanning {rotation_count} times...")

    for i in range(rotation_count):
        status = f"Scanning {i+1}/{rotation_count}"
        print(status)
        robot.left(0.4)
        rospy.sleep(0.2)
        if emergency_stop:
            break
        robot.stop()
        rospy.sleep(0.4)
        if emergency_stop:
            break
    print("Scan complete")
    is_idle = True


def listener() -> None:
    global emergency_stop, collision_stop, status, is_idle
    rospy.Subscriber("emergency", Empty, emergency_cb)
    rospy.Subscriber("collision_stop", Empty, collision_cb)
    rospy.Subscriber("loco", Twist, loco_cb)
    rospy.Subscriber('goto', PointStamped, pointcb)
    rospy.Subscriber('scan', Empty, scan_cb)
    print("Subscribed to topics")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if emergency_stop:
            if is_idle:
                rospy.loginfo("Emergency stop, robot already idle")
                emergency_stop = False
            else:
                rospy.loginfo(f"Emergency stop, stopping robot")
                robot.stop()
                rospy.sleep(1)
                is_idle = True

        if collision_stop:
            rospy.loginfo("Collision stop, reversing robot")
            robot.backward(0.3)
            time.sleep(0.25)
            robot.stop()
            collision_stop = False
        
        status_pub = rospy.Publisher('/robot/status', String, queue_size=50)
        status_pub.publish(status)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        robot.stop()
        rospy.signal_shutdown("KeyboardInterrupt")