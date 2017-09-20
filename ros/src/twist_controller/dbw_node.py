#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
import math
from styx_msgs.msg import Lane, Waypoint
#from geometry_msgs.msg import PoseStamped, Quaternion

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        #self.dbw_is_enabled = False
        self.dbw_is_enabled = False
        self.current_velocity = None
        self.current_velocity_set = False
        self.twist_cmd = None
        self.twist_cmd_set = False
        self.pose = None
        self.pose_set = False

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.final_waypoints = None
        self.final_waypoints_set = False


        # TODO: Create `TwistController` object
        self.controller = Controller()

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.loop()

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints
        self.final_waypoints_set = True
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

    def dbw_enabled_cb(self, msg):
        rospy.loginfo('%s' % (msg.data))
        self.dbw_is_enabled = msg.data

    def current_velocity_cb(self, msg):
        self.current_velocity = msg
        self.current_velocity_set = True

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg
        self.twist_cmd_set = True

    def pose_cb(self, msg):
        # TODO: Implement
        #pass
        self.pose = msg
        self.pose_set = True

        '''
        [geometry_msgs/PoseStamped]:
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        '''
    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            if self.twist_cmd_set and self.current_velocity_set and self.pose_set:
                # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
                #                                                      <proposed angular velocity>,
                #                                                      <current linear velocity>,
                #                                                      <dbw status>,
                #                                                      <any other argument you need>)
                throttle, brake, steering = self.controller.control(self.twist_cmd,
                                                                     self.current_velocity,
                                                                     self.pose,
                                                                     self.dbw_is_enabled)
                # if self.dbw_is_enabled:
                #     print ("atomatic mode !!!\n")
                # else:
                #     print("manual mode !!!\n")
                #
                if self.dbw_is_enabled == True:
                    # print("twist linear: [%2.4f, %2.4f, %2.4f]\n" %(self.twist_cmd.twist.linear.x,
                    #                                             self.twist_cmd.twist.linear.y,
                    #                                             self.twist_cmd.twist.linear.z))
                    # print("twist angular: [%2.4f, %2.4f, %2.4f]\n" %(self.twist_cmd.twist.angular.x,
                    #                                             self.twist_cmd.twist.angular.y,
                    #                                             self.twist_cmd.twist.angular.z))
                    # print("velocity linear: [%2.4f, %2.4f, %2.4f]\n" %(self.current_velocity.twist.linear.x,
                    #                                             self.current_velocity.twist.linear.y,
                    #                                             self.current_velocity.twist.linear.z))
                    # print("velocity angular: [%2.4f, %2.4f, %2.4f]\n" %(self.current_velocity.twist.angular.x,
                    #                                             self.current_velocity.twist.angular.y,
                    #                                             self.current_velocity.twist.angular.z))
                    #
                    # print("throttle: %2.4f brake: %2.4f steer: %2.4f\n" % (throttle, brake, steering))
                    self.publish(throttle, brake, steering)
                    #self.publish(0.3, 0.0, 0.0)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
