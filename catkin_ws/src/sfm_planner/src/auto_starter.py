#!/usr/bin/env python3

import os
import subprocess
import time
import fileinput

import rospy
import roslaunch


num_instances = 10
sleep_duration = 10
default_port_id = 19997  # starting from this number
pids = []

COPPELIA_ROOT_DIR = os.getenv('COPPELIA_ROOT_DIR')
WORLDS_DIR = os.getenv('COPPELIA_WORLDS_DIR')
SFM_LAUNCHES_DIR = os.getenv('SFM_LAUNCHES_DIR')
remote_connection_file = f'{COPPELIA_ROOT_DIR}/remoteApiConnections.txt'

port_id_prefix = 'portIndex1_port'

for inst_num in range(num_instances):
    node_name = "sim_" + str(inst_num)
    port_id = default_port_id + inst_num

    # + Changing the port number
    f1 = open(remote_connection_file, 'r')
    content = ""
    for line in f1:
        line = line.strip()
        if line.startswith(port_id_prefix):
            new_line = line.replace(line, f'{port_id_prefix} = {str(port_id)}')
        else:
            new_line = line
        content += new_line + '\n'
    f1.close()

    f2 = open(remote_connection_file, 'w')
    f2.write(content)
    f2.close()
    # - Changing the port number

    ########################################

    # + Running the simulator
    with open(os.devnull, 'w') as fp:  # discarding simulator's initialization messages
        pid = subprocess.Popen(["xvfb-run", "--auto-servernum", COPPELIA_ROOT_DIR+"/coppeliaSim.sh", "-h", WORLDS_DIR+"/small_corridor_obstacle_pose_publish.ttt", "-GROSInterface.nodeName="+str(node_name)], stdout=fp)
#    pid = subprocess.Popen(["xvfb-run", "--auto-servernum", COPPELIA_ROOT_DIR+"/coppeliaSim.sh", "-h", WORLDS_DIR+"/small_corridor_obstacle_pose_publish.ttt", "-GROSInterface.nodeName="+str(node_name)])

    pids.append(pid)
    time.sleep(sleep_duration)
    # - Running the simulator

    ########################################

    # + Running test conductor
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args = [f'{SFM_LAUNCHES_DIR}/ahtapot_test_conductor.launch', f'node_name:={node_name}', f'port:={str(port_id)}']
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    parent.start()
parent.spin()

    # - Running test conductor



