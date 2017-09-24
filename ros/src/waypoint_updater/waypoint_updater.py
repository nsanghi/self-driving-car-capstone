#!/usr/bin/env python

import rospy
import math
import tf
from   geometry_msgs.msg import PoseStamped, TwistStamped
from   styx_msgs.msg     import Lane, Waypoint
from   std_msgs.msg      import Int32, Float32


DEFAULT_SPEED_MPH = 10.0
LOOKAHEAD_WPS     = 500
MIN_BRAKING_COEF  = 1.7 
MAX_BRAKING_COEF  = 2.4 


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
        self.braking          =  False

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
            if angle > math.pi/4:
                nearest_index += 1

            # Handling of traffic light information
            ss = DEFAULT_SPEED_MPH * 0.44704
            if hasattr(self, 'set_speed'):
                ss = self.set_speed

            sp = None
            tw = self.traffic_waypoint
            ni = nearest_index

            # Compute min and max braking distances
            min_braking_distance = self.current_velocity * MIN_BRAKING_COEF
            max_braking_distance = self.current_velocity * MAX_BRAKING_COEF

            # Check if we know the next light waypoint
            if tw != -1 and self.braking == False:
                dist = self.distance(wpts, ni, tw)
                if min_braking_distance < dist < max_braking_distance:
                    rospy.logwarn('Initiating braking')
                    self.braking = True
                    sp = [0.0 for i in range(LOOKAHEAD_WPS)]
                else:
                    rospy.logwarn('Red light in sight but not braking (' + str(dist) + 'm)')
                    sp = [ss for i in range(LOOKAHEAD_WPS)]
            elif tw != -1 and self.braking == True:
                rospy.logwarn('Continuing to brake')
                sp = [0.0 for i in range(LOOKAHEAD_WPS)]
            else:
                rospy.logwarn('No red light in sight (accelerating/cruising)')
                self.braking = False
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


    def distance(self, waypoints, wp1, wp2):
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

