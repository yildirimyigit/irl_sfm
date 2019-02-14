import rosbag
import sys

# This program gets .bag file name and topic name as argument:
# USAGE: python get_data.py <bag_file> <topic_name>
bag = rosbag.Bag(sys.argv[1])
messages = []
topics = []
times  = []
tracks = []
for topic, msg, t in bag.read_messages():
	messages.append(msg)
	topics.append(topic)
	times.append(t)
	tracks.append(msg.tracks)

# These arrays blow hold the data as 2D arrays. 1st dimension represents which message
# the data comes from
# 2nd dimension represents which person is this
# Ex: pose_positions_x[i][j] means x position of jth person at ith message (iteration)

pose_positions_x = []
pose_positions_y = []
pose_orientations_z = []
pose_orientations_w = []

twist_linears_x = []
twist_linears_y = []
twist_angulars_z = []

for i in range(len(tracks)):
	pose_positions_x.append([])
	pose_positions_y.append([])
	pose_orientations_w.append([])
	pose_orientations_z.append([])

	twist_linears_x.append([])
	twist_linears_y.append([])
	twist_angulars_z.append([])

	for j in range(len(tracks[i])):
		pose_positions_x[i].append(tracks[i][j].pose.pose.position.x)
		pose_positions_y[i].append(tracks[i][j].pose.pose.position.x)
		pose_orientations_z[i].append(tracks[i][j].pose.pose.orientation.z)
		pose_orientations_w[i].append(tracks[i][j].pose.pose.orientation.w)

		twist_linears_x[i].append(tracks[i][j].twist.twist.linear.x)
		twist_linears_y[i].append(tracks[i][j].twist.twist.linear.y)
		twist_angulars_z[i].append(tracks[i][j].twist.twist.angular.z)
	

print '############ POSE_POSITION_X ############\n', pose_positions_x
print '############ POSE_POSITION_Y ############\n', pose_positions_y
print '############ POSE_ORIENTATION_W ############\n', pose_orientations_w
print '############ POSE_ORIENTATION_Z ############\n', pose_orientations_z

print '############ TWIST_LINEARS_X ############\n', twist_linears_x
print '############ TWIST_LINEARS_Y ############\n', twist_linears_y
#print '############ TWIST_ANGULARS_Z ############\n', twist_angulars_z