from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = 0.4
        self.pid_controller = PID(kp, ki, kd, mn, mx)
        
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.last_time = rospy.get_time()

    def control(self, linear_vel, angular_vel, curr_vel, dbw_enabled):
        if not dbw_enabled:
            self.pid_controller.reset()
            return 0.0, 0.0, 0.0

        curr_vel = self.vel_lpf.filt(curr_vel)
        steer = self.yaw_controller.get_steering(linear_vel, angular_vel, curr_vel)

        #rospy.logwarn("Angular velocity: {0}".format(angular_vel))
        #rospy.logwarn("Target velocity: {0}".format(linear_vel))
        #rospy.logwarn("Current velocity: {0}".format(curr_vel))
        #rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))
        #rospy.logwarn("Steering: {0}".format(steer))

        curr_time = rospy.get_time()
        sample_time = curr_time - self.last_time
        self.last_time = curr_time
        
        vel_error = linear_vel - curr_vel
        throttle = self.pid_controller.step(vel_error, sample_time)

        brake = 0.0
        if linear_vel == 0.0 and curr_vel < 0.1:
            throttle = 0.0
            brake = 700.0
        elif throttle < 0.1 and vel_error < 0.0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = min(700.0, (abs(decel) * self.vehicle_mass * self.wheel_radius))  # Torque N*m
            
        return throttle, brake, steer
