import rospy
from sensor_msgs.msg import Image
from RobotClass import Robot
import time
import numpy as np

robot = Robot()

def callback(data):
    np_arr = np.fromstring(data.data, np.uint16)
    np_arr = np_arr.reshape(data.height, data.width)
    np_arr[] = 
    print(np.average(np_arr[200:280],400:448]) )
    print(np_arr[240][424])
    print("\n\n ----------------------- \n\n")


def listener():
    print("initing node")
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("camera/depth/image_rect_raw", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()