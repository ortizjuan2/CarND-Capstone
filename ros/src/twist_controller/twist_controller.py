import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from std_msgs.msg import Bool
from numpy import sqrt
from math import atan2, pi
import pid
import lowpass
import tf
import rospy
#from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
DT = 0.025
MAX_STEER = 0.4363

class Controller(object):
    def __init__(self):
        # TODO: Implement
        self.prev_linear_velocity = 0
        self.p_v = [1.007781147890, 0.00387903801, 0.0047809619]
        self.p_s = [0.002301151, 0.000001716, 0.005933]
        self._pid_v = pid.PID(self.p_v[0], self.p_v[1], self.p_v[2])
        self._pid_steer = pid.PID(self.p_s[0], self.p_s[1], self.p_s[2])
        self._filt_v = lowpass.LowPassFilter(.8, .2)
        self._filt_s = lowpass.LowPassFilter(.8, .3)
        wheel_base = rospy.get_param('~wheel_base')
        steer_ratio = rospy.get_param('~steer_ratio')
        max_lat_accel = rospy.get_param('~max_lat_accel')
        max_steer_angle = rospy.get_param('~max_steer_angle')
        #self.yawController = YawController(wheel_base, steer_ratio, 5, max_lat_accel, max_steer_angle)


    def control(self, twist_cmd, current_velocity, pose, dbw_is_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        print("pose: [%2.5f, %2.5f, %2.5f]\n" % (pose.position.x, pose.position.y, yaw))

        # v = 0
        # if current_velocity.twist.linear.x < twist_cmd.twist.linear.x:
        #     v = self.prev_linear_velocity + 0.1
        #     if v > twist_cmd.twist.linear.x:
        #         v = twist_cmd.twist.linear.x
        #     brake = 0
        #     self.prev_linear_velocity = v
        #
        # else:
        #     brake = self.prev_linear_velocity - 0.1
        #     if brake < twist_cmd.twist.linear.x:
        #         brake = twist_cmd.twist.linear.x
        #     self.prev_linear_velocity = brake

        v = 0
        brake = 0
        e = twist_cmd.twist.linear.x - current_velocity.twist.linear.x
        v = self._pid_v.step(e, DT)

        if abs(v) > 1.0:
            if v > 0: v = 1.0
            else: v = -1.0


        v = self._filt_v.filt(v)

        if v < 0:
            brake = abs(v)
            v = 0


        # e = twist_cmd.twist.angular.z - current_velocity.twist.angular.z
        # steer = self._pid_steer.step(e, DT)
        # steer = self._filt_s.filt(steer)

        # steer = self.yawController.get_steering(twist_cmd.twist.linear.x,
        #                         current_velocity.twist.angular.z,
        #                         current_velocity.twist.linear.x)
        #
        # eyaw = twist_cmd.twist.angular.z - steer
        # steer = self._pid_steer.step(eyaw, DT)
        # steer = self._filt_s.filt(steer)

        steer = self._filt_s.filt(twist_cmd.twist.angular.z)


        return v, brake, steer





'''
geometry_msgs/TwistStamped
--------------------------
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
'''
