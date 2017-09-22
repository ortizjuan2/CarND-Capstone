import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from std_msgs.msg import Bool
from numpy import sqrt
from math import atan2, pi
import pid
import lowpass
import tf
import rospy
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
DT = 0.025
MAX_STEER = 0.4363

class Controller(object):
    def __init__(self):
        # TODO: Implement
        self.prev_linear_velocity = 0
        self.prev_steer = 0
        #self.p_v = [1.007781147890, 0.00387903801, 0.0047809619]
        self.p_v = [1.187355162, 0.044831144, 0.00295747]
        self.p_s = [0.0023011, 0.0000017, 0.005933]
        self._pid_v = pid.PID(self.p_v[0], self.p_v[1], self.p_v[2])
        self._pid_steer = pid.PID(self.p_s[0], self.p_s[1], self.p_s[2])
        self._filt_v = lowpass.LowPassFilter(.8, 1.)
        self._filt_s = lowpass.LowPassFilter(.8, 1.)
        wheel_base = rospy.get_param('~wheel_base')
        steer_ratio = rospy.get_param('~steer_ratio')
        max_lat_accel = rospy.get_param('~max_lat_accel')
        max_steer_angle = rospy.get_param('~max_steer_angle')
        self.yawController = YawController(wheel_base, steer_ratio, 1.0, max_lat_accel, max_steer_angle)
        self.tflistener = tf.TransformListener()


    def control(self, twist_cmd, current_velocity, pose, dbw_is_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # print("-------------")
        # try:
        #     (trans,rot) = self.tflistener.lookupTransform('/base_link', '/world', rospy.Time(0))
        #     newPose = self.tflistener.transformPose('/base_link', pose)
        #     #print("transform: "),
        #     #print(trans)
        #     #print("rotation: "),
        #     #print(rot)
        #     yaw = atan2(trans[1], trans[0])
        #     print("pose: [%2.5f, %2.5f, %2.5f]" % (pose.pose.position.x, pose.pose.position.y, yaw))
        #     #print("new poase: %4.6f, %4.6f, %4.6f," % (newPose.pose.position.x,
        #     #                                newPose.pose.position.y,
        #     #                                newPose.pose.position.z))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.loginfo("error with tflistener")
        #     pass

        # try:
        #     (trans,rot) = self.tflistener.lookupTransform('/base_link', '/world', rospy.Time(0))
        #     print("transform: "),
        #     print(trans)
        #     print("rotation: "),
        #     print(rot)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass




        '''
        ------------- from "world" to "base_link" ----------
        transform:  [1365.884, 1179.837, 0.02573395]
        rotation:  [0.0, 0.0, 0.043137258079278956, -0.9990691552467235]
        pose: [1365.88400, 1179.83700, -0.08630]

        twist linear: [8.9408, 0.0000, 0.0000]

        twist angular: [0.0000, 0.0000, 0.0116]

        velocity linear: [9.7465, 0.0000, 0.0000]

        velocity angular: [0.0000, 0.0000, 0.0000]

        throttle: 0.0000 brake: 0.0826 steer: 0.0578

        void tf::TransformListener::lookupTransform (const std::string &target_frame,
                                const ros::Time &target_time,
                                const std::string &source_frame,
                                const ros::Time &source_time,
                                const std::string &fixed_frame,
                                StampedTransform &transform) const


        ------------- from "base_link" to "world" ----
        transform:  [-1214.4147099048728, -1234.6505308222502, -0.026125650000000004]
        rotation:  [0.0, 0.0, -0.01889555521281985, -0.99982146305888]
        pose: [1260.19800, 1187.88300, -0.03779]

        twist linear: [8.9408, 0.0000, 0.0000]

        twist angular: [0.0000, 0.0000, -0.0163]

        velocity linear: [9.6070, 0.0000, 0.0000]

        velocity angular: [0.0000, 0.0000, -0.1125]

        throttle: 0.0000 brake: 0.1149 steer: -0.0807


        '''

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

        v = brake = steer = 0
        if dbw_is_enabled == True:

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

            # steer = self.yawController.get_steering(current_velocity.twist.linear.x,
            #                           twist_cmd.twist.angular.z,
            #                           current_velocity.twist.linear.x)
            steer = self.yawController.get_steering(twist_cmd.twist.linear.x,
                                      twist_cmd.twist.angular.z,
                                      current_velocity.twist.linear.x)
            steer = self._filt_s.filt(steer)
            #
            # e = steer_desired - self.prev_steer
            # u = self._pid_steer.step(e, 0.025)
            # steer = self._filt_s.filt(u)
            # self.prev_steer = steer
            #

            # steer_current = self.yawController.get_steering(current_velocity.twist.linear.x,
            #                          current_velocity.twist.angular.z,
            #                          current_velocity.twist.linear.x)
            #
            # e = steer_desired - steer_current
            # u = self._pid_steer.step(e, 0.025)
            # steer = self._filt_s.filt(u)

            #
            # eyaw = twist_cmd.twist.angular.z - steer
            # steer = self._pid_steer.step(eyaw, DT)
            # steer = self._filt_s.filt(steer)

            #steer = self._filt_s.filt(twist_cmd.twist.angular.z)
            #throttle: 0.0000 brake: 0.0000 steer: 0.0000
        else:
            self._pid_steer.reset()
            self._pid_v.reset()
            self._filt_s.reset()
            self._filt_v.reset()

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
