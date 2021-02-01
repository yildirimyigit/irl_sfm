import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

pub = rospy.Publisher("/state_sub", Float32MultiArray)
rospy.init_node("fake_stater")

# fake
# x = np.load("data/moving/failed_x_-2.4_y_8.5.npy")
# x = np.load("data/moving/failed_x_-2.9_y_6.4.npy")
# real
x = np.load("data/input.npy")[np.random.randint(0, 7068)]

rate = rospy.Rate(30)
for x_i in x:
    msg = Float32MultiArray()
    msg.data = x_i.tolist()
    pub.publish(msg)
    rate.sleep()
