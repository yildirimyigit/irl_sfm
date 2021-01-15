#!/usr/bin/env python

import rospy

try:
    import vrep
except:
    print('BOOM')


vel = -0.35  # m/s
x_pose = 1.0
y_pose_start, y_pose_limit = 9.5, 2.5

vrep.simxFinish(-1)
client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5) 
 
if client_id != -1: # if we connected successfully
    print ('Connected to remote API server')
print('starting')

rospy.init_node('obstacle_mover', anonymous=True)
res, obstacle_handle = vrep.simxGetObjectHandle(client_id,'obstacle',vrep.simx_opmode_oneshot_wait)
y_pose = y_pose_start

hz = 20
rate = rospy.Rate(hz)
while not rospy.is_shutdown():
    returnCode = vrep.simxSetObjectPosition(client_id, obstacle_handle, -1, (x_pose, y_pose, 2), vrep.simx_opmode_oneshot)
    
    y_pose += vel/hz
    if abs(y_pose) < abs(y_pose_limit):
        y_pose = y_pose_start

    rate.sleep()

vrep.simxFinish(-1)
print('finito')
    







