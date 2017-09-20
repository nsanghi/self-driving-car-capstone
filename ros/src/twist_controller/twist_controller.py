import rospy
import math
from   yaw_controller import YawController
from   pid            import PID
from   lowpass        import LowPassFilter
from   std_msgs.msg   import Float32


class Controller(object):

    def __init__(self):

        # Constants we need for control
        self.brake_deadband  = rospy.get_param('~brake_deadband')
        self.decel_limit     = rospy.get_param('~decel_limit')
        self.max_lat_accel   = rospy.get_param('~max_lat_accel')
        self.max_steer_angle = rospy.get_param('~max_steer_angle') 
        self.steer_ratio     = rospy.get_param('~steer_ratio')
        self.vehicle_mass    = rospy.get_param('~vehicle_mass')
        self.wheel_base      = rospy.get_param('~wheel_base')
        self.wheel_radius    = rospy.get_param('~wheel_radius')

        # Max braking torque
        self.max_torque = self.vehicle_mass * math.fabs(self.decel_limit) * self.wheel_radius

        # Timer initializer 
        self.last_time = None

        # PID controllers
        self.pid_control  = PID(0.4, 0.2, 0.0)
        self.pid_steering = PID(0.4, 0.3, 0.05)

        # Steering LPFs
        self.lpf_pre  = LowPassFilter(0.05, 0.02)
        self.lpf_post = LowPassFilter(0.45, 0.02)

        # Yaw controller
        self.yaw_control = YawController(wheel_base      = self.wheel_base, 
                                         steer_ratio     = self.steer_ratio,
                                         min_speed       = 0.0, 
                                         max_lat_accel   = self.max_lat_accel,
                                         max_steer_angle = self.max_steer_angle)    


    def control(self, **kwargs):

        # Unpack and rename the control variables           
        dbw_enabled = kwargs['dbw_enabled']

        tc_l = kwargs['twist_cmd'].twist.linear
        tc_a = kwargs['twist_cmd'].twist.angular

        cv_l = kwargs['current_velocity'].twist.linear
        cv_a = kwargs['current_velocity'].twist.angular

        desired_linear_velocity  = tc_l.x
        desired_angular_velocity = tc_a.z

        current_linear_velocity  = cv_l.x
        current_angular_velocity = cv_a.z

        # Clear PID integral accumulator if speed is slow or drive by wire is not enabled
        if dbw_enabled is False:
            self.pid_control.reset()
            self.pid_steering.reset() 

        if current_linear_velocity < 1.0:
            self.pid_control.reset()
            self.pid_steering.reset() 

        # If we have been running and can compute a time difference
        if self.last_time is not None:
 
            # Update times
            time           = rospy.get_time()
            delta_t        = time - self.last_time
            self.last_time = time

            # Throttle and brake PID
            velocity_error = desired_linear_velocity - current_linear_velocity
            control        = self.pid_control.update(velocity_error, delta_t)

            # Default throttle and brake to zero
            throttle = 0.0
            brake    = 0.0

            # If PID implies acceleration
            if control > 0:
            	throttle = max(0.0, control)
                throttle = self.soft_scale(throttle, 0.5, 1.0)
                rospy.logwarn('Accelerating')
                rospy.logwarn('')

            # If PID implies deceleration
            else:
                #self.pid_control.reset()
            	brake = max(0.0, -control) 
                brake = self.soft_scale(brake, 0.5, self.max_torque) + self.brake_deadband
                rospy.logwarn('Braking')
                rospy.logwarn('')

            # Steering desired and current yaw estimates
            desired_steering = self.yaw_control.get_steering(desired_linear_velocity, 
                                                             desired_angular_velocity, 
                                                             desired_linear_velocity)

            current_steering = self.yaw_control.get_steering(current_linear_velocity,
                                                             current_angular_velocity,
                                                             current_linear_velocity)

            # Steering error, smoothing filters, PID and bounding
            steering = desired_steering - current_steering
            steering = self.lpf_pre.filter(steering)
            steering = self.pid_steering.update(steering, delta_t)
            steering = self.lpf_post.filter(steering)
            steering = self.bound(steering, self.max_steer_angle)
  
            #rospy.logwarn('desired:  ' + str(desired_linear_velocity))
            #rospy.logwarn('current:  ' + str(current_linear_velocity)) 
            #rospy.logwarn('error:    ' + str(velocity_error))
            #rospy.logwarn('throttle: ' + str(throttle))
            #rospy.logwarn('brake:    ' + str(brake))
            #rospy.logwarn('steering: ' + str(steering))
            #rospy.logwarn('')
 
            return throttle, brake, steering

        # If this is the first run and we need a baseline time
        else:
            self.last_time = rospy.get_time()
            return 0.0, 0.0, 0.0


    # Symmetric bounding constraint applied to a value
    def bound(self, value, limit):
        if value < -limit:
            return -limit
        if value > limit:
            return limit
        return value


    # Smooth curve through zero with asymptotic positive behavior
    def soft_scale(self, value, squish, stretch):
        if value < 0.0:
            return 0.0
        else:
            return stretch * math.tanh(value / squish)

