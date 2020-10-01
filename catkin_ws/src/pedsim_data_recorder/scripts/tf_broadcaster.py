#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped


EAST = (0.5, 2.5, 0.0)
WEST = (28.5, 2.5, 0.0)

if __name__ == '__main__':
    rospy.init_node('pedsim_tf_broadcaster', anonymous=True)

    pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    t = TransformStamped()
    t.header.frame_id = "odom"
    t.transform.translation.y = EAST[1]  # = WEST[1]
    t.transform.translation.z = EAST[2]

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        t.child_frame_id = "east"
        t.transform.translation.x = EAST[0]
        t.header.stamp = rospy.Time.now()
        tfm = tf2_msgs.msg.TFMessage([t])
        pub_tf.publish(tfm)

        t.child_frame_id = "west"
        t.transform.translation.x = WEST[0]
        t.header.stamp = rospy.Time.now()
        tfm = tf2_msgs.msg.TFMessage([t])
        pub_tf.publish(tfm)

        rate.sleep()

    rospy.spin()
