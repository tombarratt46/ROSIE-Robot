import rospy

def depth_cb(data):
    global distance, prev_calc
    # # if time.time() - prev_calc < 0.5:
    # #     return
    # prev_calc = time.time()
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