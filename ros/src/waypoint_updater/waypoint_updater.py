#!/usr/bin/env python

import rospy
import math
import yaml
import time
import tf
from   cPickle           import loads, dumps
from   geometry_msgs.msg import PoseStamped, TwistStamped
from   styx_msgs.msg     import Lane, Waypoint, TrafficLightArray
from   std_msgs.msg      import Int32, Float32


RATE          = 10
LOOKAHEAD_WPS = 500


class WaypointUpdater(object):

    def __init__(self):

        rospy.init_node('waypoint_updater')

        # Subscriptions
        rospy.Subscriber('/base_waypoints',    Lane,         self.base_waypoints_cb)
        rospy.Subscriber('/current_pose',      PoseStamped,  self.current_pose_cb)
        rospy.Subscriber('/current_velocity',  TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32,        self.obstacle_waypoint_cb)
        rospy.Subscriber('/set_speed',         Float32,      self.set_speed_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32,        self.traffic_waypoint_cb)

        # Publication
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # Initialize constants
        self.traffic_waypoint = -1 
        self.current_velocity =  0.0
        self.braking          =  False

        # Load traffic light stopping positions
        config_string   = rospy.get_param("/traffic_light_config")
        config          = yaml.load(config_string)
        light_positions = config['stop_line_positions']
 
        # Wait for the subscriptions needed later
        while hasattr(self, 'base_waypoints') == False or hasattr(self, 'current_pose') == False:
            time.sleep(0.05)

        # Compute list of waypoint indices for each known traffic light position
        wpts = self.base_waypoints.waypoints
        self.light_indices = []
        for light_pos in light_positions:
            closest_distance = float('inf')
            closest_index    = None
            for i in range(len(wpts)):
                p = light_pos
                q = wpts[i].pose.pose.position
                d = math.sqrt((p[0]-q.x)**2 + (p[1]-q.y)**2)
                if d < closest_distance:
                    closest_distance = d
                    closest_index    = i
            self.light_indices.append(closest_index)

        # Set rate and loop
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


    def loop(self):

        # Create lane
        lane                 = Lane()
        lane.header.stamp    = rospy.Time().now()
        lane.header.frame_id = '/world'
        lane.waypoints       = []

        # Define pose and waypoints
        wpts = self.base_waypoints.waypoints
        p    = self.current_pose.pose.position

        # Compute nearest waypoint
        nearest_index    = None
        nearest_distance = float('inf')
        for i in range(len(wpts)):
            q = wpts[i].pose.pose.position
            d = math.sqrt((p.x-q.x)**2 + (p.y-q.y)**2 + (p.z-q.z)**2)
            if d < nearest_distance:
                nearest_index    = i
                nearest_distance = d

        # Ensure heading
        delta_py = wpts[nearest_index].pose.pose.position.y - p.y
        delta_px = wpts[nearest_index].pose.pose.position.y - p.x
        heading  = math.atan2(delta_py, delta_px)
        x = self.current_pose.pose.orientation.x
        y = self.current_pose.pose.orientation.y
        z = self.current_pose.pose.orientation.z
        w = self.current_pose.pose.orientation.w
        euler_angles_xyz = tf.transformations.euler_from_quaternion([x, y, z, w])
        theta = euler_angles_xyz[-1]
        angle = math.fabs(theta-heading)
        if angle > math.pi / 4.0:
            nearest_index += 1

        # Determine nearest traffic light
        closest_dist  = float('inf')
        closest_index = None
        for light_index in self.light_indices:
            dist = self.path_distance(wpts, nearest_index, light_index)
            if dist < closest_dist and light_index > nearest_index:
                closest_index = light_index
                closest_dist  = dist

        # Default speed handling and threshold
        ss = wpts[nearest_index].twist.twist.linear.x
        if hasattr(self, 'set_speed'):
            ss = self.set_speed
        threshold = self.compute_threshold(ss)

        # Handy green variable
        green = True
        if self.traffic_waypoint == -1:
            green = False
            
        # Logic to control actions
        sp = 0.0
        if closest_dist > threshold: 
            rospy.logwarn('Cruising')
            self.braking = False
            sp = ss
        elif closest_dist < threshold and green == False:
            rospy.logwarn('Braking')
            self.braking = True
            sp = 0.0 
        elif closest_dist < threshold and green == True:
            rospy.logwarn('Accelerating')
            self.braking = False
            sp = ss 
        else:
            rospy.logwarn('Default')
            self.braking = False
            sp = ss

        # Create forward list of waypoints 
        for i in range(nearest_index, nearest_index + LOOKAHEAD_WPS):
            idx = i % len(wpts)
            wpt = loads(dumps(wpts[idx], -1))
            lane.waypoints.append(wpt)

        # Set the speed
        for i in range(len(lane.waypoints)):
            lane.waypoints[i].twist.twist.linear.x = sp

        self.final_waypoints_pub.publish(lane)


    def compute_threshold(self, speed):
        if speed > 10:
            return 25
        if speed > 7:
            return 20
        if speed > 4: 
            return 14
        if speed > 2:
            return 9
        return 4
      

    def current_pose_cb(self, msg):
        self.current_pose = msg


    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


    def base_waypoints_cb(self, msg):
        self.base_waypoints = msg


    def traffic_waypoint_cb(self, msg):
        self.traffic_waypoint = msg.data


    def obstacle_waypoint_cb(self, msg):
        self.obstacle_waypoint = msg.data


    def set_speed_cb(self, msg):
        self.set_speed = msg.data


    def path_distance(self, waypoints, wp1, wp2):
        dist = 0.0
        dl   = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

