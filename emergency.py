import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
import numpy as np

def depth_cb(data):
    global distance, prev_calc

    np_arr = np.fromstring(data.data, np.uint16)
    np_arr = np_arr.reshape(data.height, data.width)
    np_arr[:190,:] = 0
    np_arr[280:,:] = 0
    np_arr[:,:400] = 0
    np_arr[:,448:] = 0
    np_arr[np_arr > 1000] = 0
    np_arr[np_arr < 100] = 0
    np_arr[0,0] = 2000
    distance = np_arr[np_arr!=0].min()

    if distance < 350:
        print("Emergency stop - distance: ", distance)
        pub = rospy.Publisher('/collision_stop', Empty, queue_size=1)
        pub.publish(Empty())
    else:
        print("Distance: ", distance)
        
rospy.init_node('collision_checker', anonymous=True)
rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_cb)
rospy.spin()
