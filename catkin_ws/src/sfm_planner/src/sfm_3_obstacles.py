#!/usr/bin/env python

import rospy
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import Twist, Pose, Vector3, PoseStamped, Point, Quaternion, TransformStamped, Vector3Stamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Bool

import numpy as np


MULT = 10


class SFMController:
  def __init__(self):
    self.x_thr, self.y_thr = 0.3, 0.3
    ###################
    self.relaxation_time = 1.5
    self.last_cmd_vel = Twist()
    self.desired_speed = 0.5
    self.linear_max_speed = 0.9
    
    ###
    
    self.force_strength = 100
    self.force_range = 0.25
    self.sum_radii = 0.43
    
    self.starting_pose = PoseStamped(Header(0, 0, 'odom'), Pose(Point(rospy.get_param('/start/position/x', 0.0), rospy.get_param('/start/position/y', -14.0), 0), Quaternion(0, 0, rospy.get_param('/start/orientation/z', 0.706),rospy.get_param('/start/orientation/w', 0.707))))
    self.goal_pose = PoseStamped(Header(0, 0, 'odom'), Pose(Point(rospy.get_param('/goal/position/x', 3.0), rospy.get_param('/goal/position/y', -6.0), 0), Quaternion(0, 0, rospy.get_param('/goal/orientation/z', 0.706),rospy.get_param('/goal/orientation/w', 0.707))))
    self.last_repulsive_force, self.last_attractive_force = Vector3(), Vector3()
    
    self.distance = self.calculate_distance(self.starting_pose, self.goal_pose)
    self.distance_to_closest_obstacle = Vector3()
    self.distance_magnitude_to_closest_obstacle = 0
    self.goal_reached = False
    
    self.laser_subs = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
    self.pose_subs = rospy.Subscriber("/robotPose", PoseStamped, self.pose_callback)
    self.obs_pose_subs = rospy.Subscriber("/obstacle_pose", PoseStamped, self.obs_pose_callback)
    self.obs_0_pose_subs = rospy.Subscriber("/obstacle_0_pose", PoseStamped, self.obs_0_pose_callback)
    self.obs_1_pose_subs = rospy.Subscriber("/obstacle_1_pose", PoseStamped, self.obs_1_pose_callback)
    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.goal_reached_pub = rospy.Publisher("/sfm/goal_reached", Bool, queue_size=1)
    self.tf_listener = tf.TransformListener()
    
    ###
    self.last_pose = PoseStamped()
    self.obstacle_pose, self.obstacle0_pose, self.obstacle1_pose = [PoseStamped()]*3
    # self.obstacle_pose.pose.position.x = 1
    # self.obstacle_pose.pose.position.y = -10
    
    self.local_minima_angle_threshold = 0.001
    self.local_minima_magnitude_threshold = 4
    
    self.obs_read, self.obs0_read, self.obs1_read = [False]*3
    
  def laser_callback(self, msg):
    self.last_repulsive_force = self.calculate_repulsive_force(msg)

  def pose_callback(self, msg):
    self.last_pose = msg
    self.distance = self.calculate_distance(msg, self.goal_pose)
    self.last_attractive_force = self.calculate_attractive_force(self.distance)
    
  def obs_pose_callback(self, msg):
    self.obstacle_pose = msg
    self.obs_read = True
    
  def obs_0_pose_callback(self, msg):
    self.obstacle0_pose = msg
    self.obs0_read = True
    
  def obs_1_pose_callback(self, msg):
    self.obstacle1_pose = msg
    self.obs1_read = True
  
  def calculate_repulsive_force(self, scan):
    # TODO: use scan (now using predefined obstacle pose)
    
    self.distance_to_closest_obstacle = self.calculate_distance(self.obstacle_pose, self.last_pose)
    self.distance_magnitude_to_closest_obstacle = self.calculate_magnitude(self.distance_to_closest_obstacle)
    direction = self.normalize(self.distance_to_closest_obstacle)
    #rospy.loginfo(direction)
    
    force = Vector3()
    force.x = self.force_strength * np.exp((self.sum_radii - self.distance_magnitude_to_closest_obstacle)/self.force_range) * direction.x
    force.y = self.force_strength * np.exp((self.sum_radii - self.distance_magnitude_to_closest_obstacle)/self.force_range) * direction.y
    
    dist0_vec = self.calculate_distance(self.obstacle0_pose, self.last_pose)
    dist0 = self.calculate_magnitude(dist0_vec)
    direction0 = self.normalize(dist0_vec)
    
    force0 = Vector3()
    force0.x = self.force_strength * np.exp((self.sum_radii - dist0)/self.force_range) * direction0.x
    force0.y = self.force_strength * np.exp((self.sum_radii - dist0)/self.force_range) * direction0.y
    
    dist1_vec = self.calculate_distance(self.obstacle1_pose, self.last_pose)
    dist1 = self.calculate_magnitude(dist1_vec)
    direction1 = self.normalize(dist1_vec)
    
    force1 = Vector3()
    force1.x = self.force_strength * np.exp((self.sum_radii - dist1)/self.force_range) * direction1.x
    force1.y = self.force_strength * np.exp((self.sum_radii - dist1)/self.force_range) * direction1.y
    
    force_tot = Vector3()
    force_tot.x = force.x + force0.x + force1.x
    force_tot.y = force.y + force0.y + force1.y
    
    return force_tot
  
  def calculate_attractive_force(self, distance):
    distance_unit = self.normalize(distance)
    attr_force = Vector3()
    # Below: F_attr = k(v_desired - v_last)
    attr_force.x = (1.0/self.relaxation_time) * self.desired_speed * distance_unit.x - self.last_cmd_vel.linear.x
    attr_force.y = (1.0/self.relaxation_time) * self.desired_speed * distance_unit.y - self.last_cmd_vel.linear.y
    return attr_force
  
  def calculate_distance(self, p0, p1):
    distance = Vector3()
    distance.x = p1.pose.position.x - p0.pose.position.x
    distance.y = p1.pose.position.y - p0.pose.position.y
    return distance
  
  def execute(self):
    while not rospy.is_shutdown():
      tf_ready = False
      vel = Twist()
      try:
        (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        tf_ready = True
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
     
      if self.distance.x < self.x_thr and self.distance.y < self.y_thr:  # goal reached
        if not self.goal_reached:  # not reached before
          rospy.loginfo('Goal Reached!')
          self.goal_reached = True
      else:
        total_force = Vector3()
        
        # Checking for the local minima:
        attractive_unit = self.normalize(self.last_attractive_force)
        repulsive_unit = self.normalize(self.last_repulsive_force)
        total_unit_x = attractive_unit.x + repulsive_unit.x
        if abs(total_unit_x) < self.local_minima_angle_threshold and self.distance_magnitude_to_closest_obstacle < self.local_minima_magnitude_threshold:
            total_force.x = self.last_attractive_force.x + self.last_repulsive_force.x + np.random.normal()  # normal noise to ditch the local minima
            total_force.y = self.last_attractive_force.y + self.last_repulsive_force.y
            rospy.loginfo(total_unit_x)
        else:
            total_force.x = self.last_attractive_force.x + self.last_repulsive_force.x
            total_force.y = self.last_attractive_force.y + self.last_repulsive_force.y
        self.last_cmd_vel = self.force_to_vel(total_force)
        
        # Calculated cmd_vel is in global frame. Transforming to base_link
        if tf_ready:
          v = Vector3()
          v.x = self.last_cmd_vel.linear.x * MULT
          v.y = self.last_cmd_vel.linear.y * MULT
          
          # based on https://answers.ros.org/question/196149/how-to-rotate-vector-by-quaternion-in-python/?answer=196155#post-id-196155
          # rotate v by rot
          unit_v = tf.transformations.unit_vector(np.array([self.last_cmd_vel.linear.x, self.last_cmd_vel.linear.y, 0]))
          q = list(unit_v)
          q.append(0.0)
          rot_v = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(rot, q), tf.transformations.quaternion_conjugate(rot))[:3]
          magnitude_v = self.calculate_magnitude(v)

          vel = Twist()
          vel.linear.x = rot_v[0] * magnitude_v
          vel.linear.y = rot_v[1] * magnitude_v

      self.vel_pub.publish(vel)
      self.goal_reached_pub.publish(self.goal_reached)
      
      if self.goal_reached:
        rospy.signal_shutdown('Goal Reached!')

  def normalize(self, vec3):
    vec_len = self.calculate_magnitude(vec3)
    if vec_len == 0:
        return Vector3()
    return Vector3(vec3.x/vec_len, vec3.y/vec_len, vec3.z/vec_len)
    
  def calculate_magnitude(self, vec3):
    return (vec3.x**2 + vec3.y**2 + vec3.z**2)**0.5

  def pose_tf_to_base(self, p):
    tfed_p = PoseStamped()
    tfed_p.pose.position.x = p.pose.position.y
    tfed_p.pose.position.y = p.pose.position.x
    return tfed_p
    
  def force_to_vel(self, force):
    vel = Twist()
    vel.linear.x = force.x * self.linear_max_speed;
    vel.linear.y = force.y * self.linear_max_speed;
    # rospy.loginfo(vel)
    return vel


if __name__ == '__main__':
  rospy.init_node('sfm_control')
  controller = SFMController()
  controller.execute()

  # after rospy.shotdown()
  # controller.vel_pub.publish(Twist())
