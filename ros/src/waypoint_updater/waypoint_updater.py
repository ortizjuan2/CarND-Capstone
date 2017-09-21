#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint
import tf
#from geometry_msgs.msg import Quaternion


import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.tflistener = tf.TransformListener()

        # TODO: Add other member variables you need below
        self.next_wp = None
        self.pose = None
        self.waypoints = None
        self.pose_set = False
        self.waypoints_set = False
        self.ref_vel = 1e-3
        self.MAX_SPEED = 30 * 0.44704
        self.MAX_ACCEL = 0.1

        ### TESTING
        self.waypoints_printed = False
        self.count_printed = 0

        ##

        self.publish()

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        #pass
        self.pose = msg
        self.pose_set = True


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #pass
        self.waypoints = waypoints.waypoints
        self.waypoints_set = True


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def ClosestWaypoint(self, x, y, waypoints):
        closestLen = 99999
        closesWaypoint = 0
        for i in range(len(waypoints)):
            wpx = waypoints[i].pose.pose.position.x
            wpy = waypoints[i].pose.pose.position.y
            dist = self.euclidean_distance(x, y, wpx, wpy)
            if dist < closestLen:
                closestLen = dist
                closesWaypoint = i
        return closesWaypoint

    def NextWaypoint(self, x, y, theta, waypoints):
        closestWaypoint = self.ClosestWaypoint(x, y, waypoints)
        wpx = waypoints[closestWaypoint].pose.pose.position.x
        wpy = waypoints[closestWaypoint].pose.pose.position.y

        heading = math.atan2(wpy - y, wpx - x)
        angle = abs(theta - heading)

        if angle > (np.pi/4.):
            closestWaypoint += 1

        return closestWaypoint


    def get_next_wp(self):

        quaternion = (
        self.pose.pose.orientation.x,
        self.pose.pose.orientation.y,
        self.pose.pose.orientation.z,
        self.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        yaw = euler[2]

        # if yaw < 0 :
        #     yaw = yaw + 2 * np.pi
        next_wp = self.NextWaypoint(self.pose.pose.position.x, self.pose.pose.position.y, yaw, self.waypoints)
        '''
        p = Waypoint()
        p.pose.pose.position.x = float(wp['x'])
        p.pose.pose.position.y = float(wp['y'])
        p.pose.pose.position.z = float(wp['z'])
        q = self.quaternion_from_yaw(float(wp['yaw']))
        p.pose.pose.orientation = Quaternion(*q)
        p.twist.twist.linear.x = float(self.velocity*0.27778)
        waypoints.append(p)
        TODO: define waypoints
        '''
        ## set car current position
        xc = []
        yc = []
        zc = []


        try:
            newPose = self.tflistener.transformPose('/base_link', self.pose)
            xc.append(newPose.pose.position.x)
            yc.append(newPose.pose.position.y)
            zc.append(newPose.pose.position.z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return []

        ## use next waypoints to fit polynomial

        wp_end = next_wp + 200
        more_wps = 0
        if wp_end >= len(self.waypoints):
            wp_end = len(self.waypoints)
            more_wps = 200 - (wp_end - next_wp)
            rospy.logwarm("more waypoints needed %d" % (more_wps))
        elif next_wp == wp_end:
            next_wp = 0
            wp_end = 200

        for i in range(next_wp, wp_end):
            try:
                newPose = self.tflistener.transformPose('/base_link', self.waypoints[i].pose)
                xc.append(newPose.pose.position.x)
                yc.append(newPose.pose.position.y)
                zc.append(newPose.pose.position.z)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return []

        if more_wps != 0:
            for i in range(more_wps):
                try:
                    newPose = self.tflistener.transformPose('/base_link', self.waypoints[i].pose)
                    xc.append(newPose.pose.position.x)
                    yc.append(newPose.pose.position.y)
                    zc.append(newPose.pose.position.z)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    return []

        ## convert to car frame


        # try:
        #     (trans,rot) = self.tflistener.lookupTransform('/base_link', '/world', rospy.Time(0))
        #     print("transform: "),
        #     print(trans)
        #     print("rotation: "),
        #     print(rot)
        #     #yaw = atan2(trans[1], trans[0])
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass

        # for i in range(len(xc)):
        #     shiftx = xc[i] - posx
        #     shifty = yc[i] - posy
        #     xc[i] = shiftx * np.cos(0 - yaw) - shifty * np.sin(0 - yaw)
        #     yc[i] = shiftx * np.sin(0 - yaw) + shifty * np.cos(0 - yaw)
        #
        # ## fit 5th order polynomial
        fit = np.polyfit(xc,yc, 5)
        fy = np.poly1d(fit)


        targetx = 30.0
        targety = fy(targetx)
        targetdist = np.sqrt(targetx**2 + targety**2)

        x_add_on = 0

        nextx = []
        nexty = []
        waypoints = []
        posx = self.pose.pose.position.x
        posy = self.pose.pose.position.y
        prevx = posx
        prevy = posy
        dt = 0.025

        for j in range(LOOKAHEAD_WPS):
            N = targetdist / (dt * self.ref_vel)
            xpoint = x_add_on + targetx/N
            ypoint = fy(xpoint)
            x_add_on = xpoint
            ## rotate back to world frame
            npose = PoseStamped()
            npose.header.stamp = rospy.Time(0)
            npose.header.frame_id = '/base_link'
            npose.pose.position.x = xpoint
            npose.pose.position.y = ypoint
            npose.pose.position.z = 0.
            npose.pose.orientation.x = 0.
            npose.pose.orientation.y = 0.
            npose.pose.orientation.z = math.atan2(ypoint, xpoint)

            newPose = self.tflistener.transformPose('/world', npose)

            xref = xpoint
            yref = ypoint
            xpoint = xref * np.cos(yaw) - yref * np.sin(yaw)
            ypoint = xref * np.sin(yaw) + yref * np.cos(yaw)
            xpoint += posx
            ypoint += posy


            rospy.loginfo("old pose: %3.4f, %3.4f, %3.4f" % (xpoint,
                                            ypoint,
                                            yaw))



            rospy.loginfo("new pose: %3.4f, %3.4f, %3.4f" % (newPose.pose.position.x,
                                            newPose.pose.position.y,
                                            newPose.pose.orientation.z))
            '''
            [INFO] [1505965263.675728]: old pose: 1232.3648, 1188.3714, 0.0429
            [INFO] [1505965263.676147]: new pose: 62.6939, -0.2726, -0.0043
            [INFO] [1505965263.677297]: old pose: 1232.7002, 1188.3743, 0.0429
            [INFO] [1505965263.677650]: new pose: 63.0291, -0.2842, -0.0045
            [INFO] [1505965263.678792]: old pose: 1233.0357, 1188.3771, 0.0429
            [INFO] [1505965263.679199]: new pose: 63.3644, -0.2958, -0.0047
            [INFO] [1505965263.680272]: old pose: 1233.3711, 1188.3797, 0.0429
            [INFO] [1505965263.680660]: new pose: 63.6996, -0.3076, -0.0048
            [INFO] [1505965263.681770]: old pose: 1233.7066, 1188.3822, 0.0429
            [INFO] [1505965263.682346]: new pose: 64.0349, -0.3194, -0.0050
            [INFO] [1505965263.683203]: old pose: 1234.0421, 1188.3846, 0.0429
            [INFO] [1505965263.683471]: new pose: 64.3702, -0.3314, -0.0051
            [INFO] [1505965263.684647]: old pose: 1234.3775, 1188.3869, 0.0429
            [INFO] [1505965263.685133]: new pose: 64.7054, -0.3436, -0.0053
            [INFO] [1505965263.686531]: old pose: 1234.7130, 1188.3891, 0.0429
            [INFO] [1505965263.687051]: new pose: 65.0407, -0.3558, -0.0055
            [INFO] [1505965263.688472]: old pose: 1235.0485, 1188.3911, 0.0429
            [INFO] [1505965263.689000]: new pose: 65.3760, -0.3682, -0.0056
            [INFO] [1505965263.690336]: old pose: 1235.3840, 1188.3930, 0.0429
            [INFO] [1505965263.690838]: new pose: 65.7112, -0.3806, -0.0058
            [INFO] [1505965263.692205]: old pose: 1235.7195, 1188.3948, 0.0429
            [INFO] [1505965263.692722]: new pose: 66.0465, -0.3932, -0.0060
            [INFO] [1505965263.694077]: old pose: 1236.0550, 1188.3965, 0.0429
            [INFO] [1505965263.694580]: new pose: 66.3817, -0.4060, -0.0061
            [INFO] [1505965263.695901]: old pose: 1236.3905, 1188.3980, 0.0429
            [INFO] [1505965263.696423]: new pose: 66.7170, -0.4188, -0.0063
            [INFO] [1505965263.697721]: old pose: 1236.7260, 1188.3995, 0.0429
            [INFO] [1505965263.698249]: new pose: 67.0523, -0.4318, -0.0064


            [INFO] [1505965752.690799]: old pose: 1320.0167, 1184.4527, -0.0725
            [INFO] [1505965752.691052]: new pose: 1320.0167, 1184.4527, 0.9993
            [INFO] [1505965752.691727]: old pose: 1320.3503, 1184.4186, -0.0725
            [INFO] [1505965752.692062]: new pose: 1320.3503, 1184.4186, 0.9993
            [INFO] [1505965752.692710]: old pose: 1320.6838, 1184.3845, -0.0725
            [INFO] [1505965752.693016]: new pose: 1320.6838, 1184.3845, 0.9993
            [INFO] [1505965752.693680]: old pose: 1321.0174, 1184.3504, -0.0725
            [INFO] [1505965752.693982]: new pose: 1321.0174, 1184.3504, 0.9993
            [INFO] [1505965752.694692]: old pose: 1321.3510, 1184.3163, -0.0725
            [INFO] [1505965752.695017]: new pose: 1321.3510, 1184.3163, 0.9993
            [INFO] [1505965752.695732]: old pose: 1321.6845, 1184.2823, -0.0725
            [INFO] [1505965752.696024]: new pose: 1321.6845, 1184.2823, 0.9993
            [INFO] [1505965752.696716]: old pose: 1322.0181, 1184.2482, -0.0725
            [INFO] [1505965752.697002]: new pose: 1322.0181, 1184.2482, 0.9993
            [INFO] [1505965752.697638]: old pose: 1322.3517, 1184.2142, -0.0725
            [INFO] [1505965752.697935]: new pose: 1322.3517, 1184.2142, 0.9993
            [INFO] [1505965752.698572]: old pose: 1322.6852, 1184.1802, -0.0725
            [INFO] [1505965752.698866]: new pose: 1322.6852, 1184.1802, 0.9993
            [INFO] [1505965752.699549]: old pose: 1323.0188, 1184.1462, -0.0725
            [INFO] [1505965752.699922]: new pose: 1323.0188, 1184.1462, 0.9993
            [INFO] [1505965752.700683]: old pose: 1323.3524, 1184.1122, -0.0725
            [INFO] [1505965752.701056]: new pose: 1323.3524, 1184.1122, 0.9993
            [INFO] [1505965752.701745]: old pose: 1323.6859, 1184.0783, -0.0725
            [INFO] [1505965752.702126]: new pose: 1323.6859, 1184.0783, 0.9993
            [INFO] [1505965752.702907]: old pose: 1324.0195, 1184.0443, -0.0725
            [INFO] [1505965752.703246]: new pose: 1324.0195, 1184.0443, 0.9993

            '''
            nextx.append(xpoint)
            nexty.append(ypoint)
            # make one waypoint
            p = Waypoint()
            #p.pose.pose.position.x = float(xpoint)
            #p.pose.pose.position.y = float(ypoint)
            #p.pose.pose.position.z = float(0.0)
            p.pose = newPose
            #newyaw = math.atan2(ypoint - prevy, xpoint - prevx)
            #if newyaw < 0:
            #    newyaw = newyaw + 2 * np.pi
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, newPose.pose.orientation.z)
            p.pose.pose.orientation = Quaternion(*q)
            p.pose.header.frame_id = "/world"
            p.twist.header.frame_id = '/world'
            #newvel = np.sqrt((prevx - xpoint)**2 + (prevy - ypoint)**2)/dt
            #p.twist.twist.linear.x = float(newvel)
            p.twist.twist.linear.x = float(self.ref_vel)
            p.twist.twist.angular.z = newPose.pose.orientation.z
            waypoints.append(p)

            prevx = xpoint
            prevy = ypoint

            if self.ref_vel < self.MAX_SPEED:
                self.ref_vel += self.MAX_ACCEL
            else:
                self.ref_vel = self.MAX_SPEED



        return waypoints

    '''
    [styx_msgs/Lane]:
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    styx_msgs/Waypoint[] waypoints
      geometry_msgs/PoseStamped pose
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
      geometry_msgs/TwistStamped twist
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



    def publish(self):
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            if (self.pose_set == True) and (self.waypoints_set == True):
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time(0)
                lane.waypoints = self.get_next_wp()
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
