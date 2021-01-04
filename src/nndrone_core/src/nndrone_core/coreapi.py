#!/usr/bin/env python
from mavros_msgs.srv import CommandBool,SetMode,WaypointPush
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from geometry_msgs.msg import PoseStamped, Twist, Point
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix

from nndrone_msgs.srv import Arm, ArmResponse
from nndrone_msgs.srv import TakeOff, TakeOffResponse
from nndrone_msgs.srv import PositionSetLocal, PositionSetLocalResponse
from nndrone_msgs.srv import PositionSetGlobal, PositionSetGlobalResponse
from nndrone_msgs.srv import FlightMode, FlightModeResponse

import rospy
import time
import math

from tf.transformations import quaternion_from_euler

class NdroneCommander(object):
    def __init__(self):
        self.current_heading = 0
        self.delta_x = 0
        self.delta_y = 0
        self.delta_z = 0
        self.delta_lat = 0
        self.delta_long = 0
        self.delta_alt = 0
        self.FLU_x = 0
        self.FLU_y = 0
        self.FLU_z = 0
        self.target = None
        self.is_bodyframe = False

        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.global_pose_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.global_pose_callback)

        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.global_target_pub = rospy.Publisher('mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)


    def arm(self, req):
        rospy.loginfo("Arm_called")
        response = res("Arm")
        self.is_arm = self.armService(True)
        if self.is_arm:
            response.success = True
            response.message = "Armed"
            return response()
        else:
            print("Vehicle arming failed!")
            response.success = False
            response.message = "Unable to Armed"
            return response()

    def takeoff(self, takeoff_alt):
        response = res("TakeOff")
        self.alt = self.global_pose.altitude + self.takeoff_alt - 47
        rospy.loginfo(self.alt)
        self.takeoffService(0,0,self.global_pose.latitude,self.global_pose.longitude,self.alt)

        while (self.global_pose.altitude-47) < (self.alt):
            if (self.global_pose.altitude-47) > (self.alt - 1):
                response.success = True
                response.message = "tookoff"
                return response()
            else:
                response.success = False
                response.message = "Takeoff Failed"
        return response()

    def flight_mode(self,mode):
        rospy.loginfo(self.mode)
        for i in range(10):
            self.is_mode_set = self.flightModeService(custom_mode=self.mode)
            time.sleep(0.2)
        rospy.loginfo(self.is_mode_set)
        response = res("flight_mode")
        if self.is_mode_set:
            response.success = True
            response.message = "Mode Set"
            return response()
        else:
            response.success = False
            response.message = "Unable to change"
        return response()


    def takeoff_depricated(self, req):
        rospy.loginfo(req.takeoff_alt)
        response = res("takeoff")
        self.cur_target_pose = self.construct_target(0,0,req.takeoff_alt,0,0)
        print(self.cur_target_pose)
        for i in range(20):                                                                             
            self.local_target_pub.publish(self.cur_target_pose)
            self.armService(True)
            self.flightModeService(custom_mode='OFFBOARD')
            time.sleep(0.2)

        if self.local_pose.pose.position.z > 1 and self.flightModeService(custom_mode='OFFBOARD') and self.armService(True):
            response.success = True
            response.message = "tookoff"
            return response()
        else:
            response.success = False
            response.message = "Takeoff Failed"
            return response()


    def moveto_position_local(self,x,y,z, tolerance, is_bodyframe):
        req = Point()
        req.x = x
        req.y = y
        req.z = z
        response = res("moveto_position_local")
        if self.is_arm:
            rospy.loginfo("Vehicle armed")
        else:
            response.success = False
            response.message = "Please arm the vehicle"
            return response()
        self.set_target_position(req,is_bodyframe)
        while self.armService(True) and self.flightModeService(custom_mode='OFFBOARD') and (rospy.is_shutdown() is False):
            self.local_target_pub.publish(self.target)
            rospy.loginfo(self.delta_x + self.delta_y + self.delta_z)
            self.delta_x = math.fabs(self.local_pose.pose.position.x - req.x)
            self.delta_y = math.fabs(self.local_pose.pose.position.y - req.y)
            self.delta_z = math.fabs(self.local_pose.pose.position.z - req.z)

            if (self.delta_x + self.delta_y + self.delta_z < tolerance):
                response.success = True
                response.message = "Position Reached"
                return response()
            else:
                response.success = False
                response.message = "Position Reached Failed"
        return response()

    def moveto_position_global(self,lat_x, long_y, rel_alt_z, yaw):
        response = res("moveto_position_global")
        #self.flightModeService(custom_mode='OFFBOARD')
        #self.armService(True)
        self.alt = alt
        req = Point()
        req.x = lat_x
        req.y = long_y
        req.z = rel_alt_z
        gpose = GeoPoseStamped()
        gpose.header.frame_id = 'base_link'
        gpose.header.stamp = rospy.Time.now()
        gpose.pose.position.latitude = lat_x
        gpose.pose.position.longitude = long_y
        gpose.pose.position.altitude = rel_alt_z
        q = quaternion_from_euler(0, 0, yaw)
        gpose.pose.orientation.x = q[0]
        gpose.pose.orientation.y = q[1]
        gpose.pose.orientation.z = q[2]
        gpose.pose.orientation.w = q[3]

        #self.global_target_pub.publish(gpose)
        while self.flightModeService(custom_mode='OFFBOARD') and (rospy.is_shutdown() is False):
            dp,altd = self.distance_to_global_wp(req)
            rospy.loginfo("d: {0}".format(dp))
            self.global_target_pub.publish(gpose)
            if (dp<1):
                response.success = True
                response.message = "Position Reached"
                return response()
            else:
                response.success = False
                response.message = "Position Reached Failed"
                #return response
    """
    Support Function
    """
    def distance_to_global_wp(self,req):
        """alt(amsl): meters"""
        R = 6371000  # metres
        rlat1 = math.radians(req.lat_x)
        rlat2 = math.radians(self.global_pose.latitude)

        rlat_d = math.radians(self.global_pose.latitude - req.lat_x)
        rlon_d = math.radians(self.global_pose.longitude - req.long_y)
        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
        alt_d = abs(req.lat_x +  - self.global_pose.altitude - 47)
        return d, alt_d
   
    def FLU2ENU(self, msg):

        self.FLU_x = msg.x * math.cos(self.current_heading) - msg.y * math.sin(self.current_heading)
        self.FLU_y = msg.x * math.sin(self.current_heading) + msg.y * math.cos(self.current_heading)
        self.FLU_z = msg.z


    def set_target_position(self, req, is_bodyframe):
        print("Received New Position Task!")
        if is_bodyframe:
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU 
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            self.FLU2ENU(req)

            self.FLU_x = self.FLU_x + self.local_pose.pose.position.x
            self.FLU_y = self.FLU_y + self.local_pose.pose.position.y
            self.FLU_z = self.FLU_z + self.local_pose.pose.position.z
            self.target = self.construct_target(self.FLU_x,self.FLU_y,self.FLU_z,self.current_heading)
        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU 
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X
            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.target = self.construct_target(req.x,req.y,req.z,self.current_heading)



    def construct_target(self, x, y, z, yaw, yaw_rate = 0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        target_raw_pose.coordinate_frame = 9
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z
        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE
        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate
        return target_raw_pose

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    def global_pose_callback(self, msg):
        self.global_pose = msg


def run():
    rospy.init_node('nndone_service_server')
    api_server = NdroneCommander()
    arm = rospy.Service('nndone/arm',Arm, api_server.arm)
    takeoff = rospy.Service('nndone/takeoff',TakeOff, api_server.takeoff)
    flight_mode = rospy.Service('nndone/flight_mode',FlightMode, api_server.flight_mode)
    moveto_position_local = rospy.Service('nndone/moveto_position_local',PositionSet, api_server.moveto_position_local)
    moveto_position_global = rospy.Service('nndone/moveto_position_global',PositionSetGlobal, api_server.moveto_position_global)
    rospy.spin()

if __name__ == "__main__":
    run()
