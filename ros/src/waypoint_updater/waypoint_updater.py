#!/usr/bin/env python

import rospy
import math
import yaml
import time
import tf
from   geometry_msgs.msg import PoseStamped, TwistStamped
from   styx_msgs.msg     import Lane, Waypoint, TrafficLightArray
from   std_msgs.msg      import Int32, Float32


RATE            = 10
DEFAULT_MPH     = 10.0
LOOKAHEAD_WPS   = 500
MIN_ACCEL_COEF  = 0.0
MAX_ACCEL_COEF  = 0.5
MIN_BRAKE_COEF  = 1.6 
MAX_BRAKE_COEF  = 2.2 


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
                d = math.sqrt((p[0] - q.x)**2 + (p[1] - q.y)**2)
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

        # Default speed handling
        ss = DEFAULT_MPH * 0.44704
        if hasattr(self, 'set_speed'):
            ss = self.set_speed

        # Min and max detection distances
        min_accel = self.current_velocity * MIN_ACCEL_COEF
        max_accel = self.current_velocity * MAX_ACCEL_COEF
        min_brake = self.current_velocity * MIN_BRAKE_COEF
        max_brake = self.current_velocity * MAX_BRAKE_COEF

        # Helpful variables for control
        sp = None
        tw = self.traffic_waypoint
        br = self.braking

        green = True
        if tw == -1:
            green = False
            
        # Logic to control actions
        if min_brake < closest_dist < max_brake and br == False and green == False:
            rospy.logwarn('1. Initiating braking       ({:07.2f} / {})'.format(closest_dist, tw))
            sp = [0.0 for i in range(LOOKAHEAD_WPS)]
            self.braking = True
        elif min_brake < closest_dist < max_brake and br == False and green == True:
            rospy.logwarn('2. Cruising through green   ({:07.2f} / {})'.format(closest_dist, tw))
            sp = [ss for i in range(LOOKAHEAD_WPS)]
            self.braking = False
        elif min_brake < closest_dist < max_brake and br == True and green == True:
            rospy.logwarn('3. Cruising through green   ({:07.2f} / {})'.format(closest_dist, tw))
            sp = [ss for i in range(LOOKAHEAD_WPS)]
            self.braking = False
        elif min_brake < closest_dist < max_brake and br == True and green == False:
            rospy.logwarn('4. Continuing braking       ({:07.2f} / {})'.format(closest_dist, tw))
            sp = [0.0 for i in range(LOOKAHEAD_WPS)]
            self.braking = True
        elif min_accel < closest_dist < max_accel and br == False:
            rospy.logwarn('5. Cruising through green   ({:07.2f} / {})'.format(closest_dist, tw))
            sp = [ss for i in range(LOOKAHEAD_WPS)]
            self.braking = False
        elif min_accel < closest_dist < max_accel and br == True:
            rospy.logwarn('6. Accelerating from stop   ({:07.2f} / {})'.format(closest_dist, tw))
            sp = [ss for i in range(LOOKAHEAD_WPS)]
            self.braking = False
        else:
            rospy.logwarn('7. Maintaining acceleration ({:07.2f} / {})'.format(closest_dist, tw))
            sp = [ss for i in range(LOOKAHEAD_WPS)]
            self.braking = False

        # Create forward list of waypoints 
        for i in range(nearest_index, nearest_index + LOOKAHEAD_WPS):
            index = i % len(wpts)
            lane.waypoints.append(wpts[index])

        # Set the speed
        for i in range(len(lane.waypoints)):
            lane.waypoints[i].twist.twist.linear.x = sp[i]

        self.final_waypoints_pub.publish(lane)


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

