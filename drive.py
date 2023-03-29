import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from RobotClass import Robot
import time
import logging
# import serial

# serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
# serial.write(b'0')
robot = Robot()

prev_linear = 0
prev_angular = 0
distance = 0
prev_calc = time.time()

def loco_cb(data):
    print(f"Linear: {data.linear.x}, Angular: {data.angular.z}")
    global prev_linear, prev_angular
    if data.linear.x != prev_linear:
        prev_linear = data.linear.x
        robot.forward(data.linear.x)
    if data.angular.z != prev_angular:
        prev_angular = data.angular.z
        robot.left(data.angular.z)
        
# def cmd_cb(data):
#     print(data.data)
#     if data.data == "spin":
#         serial.write(b'0')
#         time.sleep(1)
#         for i in range(5):
#             print(i)
#             robot.left(0.4)
#             time.sleep(0.2)
#             robot.stop()
#             time.sleep(0.4)
#         #time.sleep(1)
#         serial.write(b'90')
#         time.sleep(4)
#         for i in range(5):
#             print(i)
#             robot.left(0.4)
#             time.sleep(0.2)
#             robot.stop()
#             time.sleep(0.4)
#         serial.write(b'0')



def listener():
    rospy.init_node('listener', anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("Node initialized, subscribing...")
    rospy.Subscriber("loco", Twist, loco_cb)
    # rospy.Subscriber("cmd", String, cmd_cb)
    rospy.loginfo("Subscribed, spinning...")
    #rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_cb)
    rospy.spin()
    rospy.loginfo("Spunned")

if __name__ == '__main__':
    listener()

#initialize the Robot class
# robot = Robot()

# #send a command to move forward the robot
# #robot.forward(-0.175)

# for i in range(50):
#     print(i)
#     robot.left(0.4)
#     time.sleep(0.2)
#     robot.stop()
#     time.sleep(0.4)

# #robot.left(0.35)
# #time.sleep(10)


# #stop the robot
# robot.stop()
