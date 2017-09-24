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
        self.MAX_SPEED = 20 * 0.44704
        self.MAX_ACCEL = 0.1

        ### TESTING
        self.waypoints_printed = False
        self.count_printed = 0

        ##
        self.prev_time = rospy.Time.now()
        self.prev_time = self.prev_time.secs + self.prev_time.nsecs * 1e-9
        self.dt = 0.02

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
        #---
        # cur_time = rospy.Time.now()
        # cur_time = cur_time.secs + cur_time.nsecs * 1e-9
        # dt =  cur_time - self.prev_time
        # self.prev_time = cur_time
        # if dt > 0. :
        #     self.dt = dt
        #     rospy.loginfo("dt set to: %f" % dt)


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
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        # if yaw < 0 :
        #     yaw = yaw + 2 * np.pi
        next_wp = self.NextWaypoint(self.pose.pose.position.x, self.pose.pose.position.y, yaw, self.waypoints)
        # dst = self.euclidean_distance(self.pose.pose.position.x, self.pose.pose.position.y,
        #                         self.waypoints[next_wp].pose.pose.position.x, self.waypoints[next_wp].pose.pose.position.y)
        #
        # while dst < 1.0:
        #     rospy.loginfo("not enough distance: %f" % dst)
        #     next_wp += 1
        #     dst = self.euclidean_distance(self.pose.pose.position.x, self.pose.pose.position.y,
        #                             self.waypoints[next_wp].pose.pose.position.x, self.waypoints[next_wp].pose.pose.position.y)
        #

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
        posx = self.pose.pose.position.x
        posy = self.pose.pose.position.y
        posz = self.pose.pose.position.z

        xc = []
        yc = []
        yw = []

        xc.append(posx)
        yc.append(posy)
        yw.append(yaw)

        ## use next waypoints to fit polynomial

        wp_end = next_wp + 200
        if wp_end >= len(self.waypoints):
            wp_end = len(self.waypoints)

        if next_wp == wp_end:
            next_wp = 0
            wp_end = 200

        for i in range(next_wp, wp_end):
            xc.append(self.waypoints[i].pose.pose.position.x)
            yc.append(self.waypoints[i].pose.pose.position.y)
            wyaw = tf.transformations.euler_from_quaternion([self.waypoints[i].pose.pose.orientation.x,
                                                self.waypoints[i].pose.pose.orientation.y,
                                                self.waypoints[i].pose.pose.orientation.z,
                                                self.waypoints[i].pose.pose.orientation.w])[2]
            yw.append(wyaw)

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

        for i in range(len(xc)):
            shiftx = xc[i] - posx
            shifty = yc[i] - posy
            xc[i] = shiftx * np.cos(0 - yaw) - shifty * np.sin(0 - yaw)
            yc[i] = shiftx * np.sin(0 - yaw) + shifty * np.cos(0 - yaw)

        ## fit 5th order polynomial
        fit = np.polyfit(xc,yc, 5)
        fy = np.poly1d(fit)


        targetx = 30.0
        targety = fy(targetx)
        targetdist = np.sqrt(targetx**2 + targety**2)

        x_add_on = 0

        nextx = []
        nexty = []
        waypoints = []
        prevx = posx
        prevy = posy
        prevyaw = yaw
        #dt = 0.033
        dt = self.dt

        for j in range(LOOKAHEAD_WPS):
            N = targetdist / (dt * self.ref_vel)
            xpoint = x_add_on + targetx/N
            ypoint = fy(xpoint)
            x_add_on = xpoint
            ## rotate back to world frame
            xref = xpoint
            yref = ypoint
            xpoint = xref * np.cos(yaw) - yref * np.sin(yaw)
            ypoint = xref * np.sin(yaw) + yref * np.cos(yaw)
            xpoint += posx
            ypoint += posy
            nextx.append(xpoint)
            nexty.append(ypoint)
            # make one waypoint
            p = Waypoint()
            p.pose.pose.position.x = float(xpoint)
            p.pose.pose.position.y = float(ypoint)
            p.pose.pose.position.z = float(0.0)
            newyaw = math.atan2(ypoint - prevy, xpoint - prevx)

            # if newyaw < 0:
            #     newyaw = newyaw + 2 * np.pi
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, newyaw)
            p.pose.pose.orientation = Quaternion(*q)
            p.pose.header.frame_id = "/world"
            p.twist.header.frame_id = '/world'
            #newvel = np.sqrt((prevx - xpoint)**2 + (prevy - ypoint)**2)/dt
            #p.twist.twist.linear.x = float(newvel)
            p.twist.twist.linear.x = float(self.ref_vel)

            p.twist.twist.angular.z = (newyaw - prevyaw) / dt
            prevyaw = newyaw

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
