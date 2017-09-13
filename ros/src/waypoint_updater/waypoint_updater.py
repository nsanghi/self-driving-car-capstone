#!/usr/bin/env python

import rospy
import math
import tf
from   geometry_msgs.msg import PoseStamped, TwistStamped
from   styx_msgs.msg     import Lane, Waypoint
from   std_msgs.msg      import Int32, Float32


LOOKAHEAD_WPS  = 800
MIN_BRAKE_COEF = 1.0
MAX_BRAKE_COEF = 7.0


class WaypointUpdater(object):

    def __init__(self):

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose',      PoseStamped,  self.current_pose_cb)
        rospy.Subscriber('/base_waypoints',    Lane,         self.base_waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',  Int32,        self.traffic_waypoint_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32,        self.obstacle_waypoint_cb)
        rospy.Subscriber('/current_velocity',  TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/set_speed',         Float32,      self.set_speed_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        self.traffic_waypoint = -1 
        self.current_velocity =  0.0
        self.max_brake_wps    =  0.0
        self.slowing          =  False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


    def loop(self):

        if hasattr(self, 'base_waypoints') and hasattr(self, 'current_pose'):

            lane                 = Lane()
            lane.header.stamp    = rospy.Time().now()
            lane.header.frame_id = '/world'

            wpts = self.base_waypoints.waypoints
            p    = self.current_pose.pose.position

            nearest_index    = None
            nearest_distance = float('inf')

            # Compute nearest waypoint
            for i in range(len(wpts)):
                q = wpts[i].pose.pose.position
                d = math.sqrt((p.x-q.x)**2 + (p.y-q.y)**2 + (p.z-q.z)**2)
                if d < nearest_distance:
                    nearest_index    = i
                    nearest_distance = d

            # Handling of traffic light information
            ss = 0.0
            if hasattr(self, 'set_speed'):
                ss = self.set_speed

            sp = None
            tw = self.traffic_waypoint
            ni = nearest_index

            min_brake_wps = int(MIN_BRAKE_COEF * self.current_velocity)

            if self.slowing == False:
                self.max_brake_wps = int(MAX_BRAKE_COEF * self.current_velocity)

            # If we are too close to bother slowing
            if tw != -1 and tw - ni < min_brake_wps:
                self.slowing = False
                sp = [ss for i in range(LOOKAHEAD_WPS)]

            # If we are in range to come to a halt (horizon of range is how many points ahead we look at)
            elif tw != -1 and tw - ni < self.max_brake_wps:
                self.slowing = True
                sp = [0.0 for i in range(LOOKAHEAD_WPS)]

            # Otherwise we don't have indication of a red light
            else:
                self.slowing = False
                sp = [ss for i in range(LOOKAHEAD_WPS)]

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

