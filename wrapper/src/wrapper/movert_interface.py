#! /usr/bin/env python3


import rospy
import copy

from move_rt.msg import *
from move_rt.srv import *

import actionlib

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose

import numpy as np
import quaternion

class MoveRTInterfacer(object) :

    def __init__(self, robot = '', control_switch = False) :

        
        self.arm = robot
        print('Robot {} Initialization Start'.format(self.arm))


        self.traj_point = {'orient_w': None, 'orient_x': None, 'orient_y': None, 'orient_z':  None, 'pos_x': None, 'pos_y': None, 'pos_z': None}
        self.traj_list = []


        # MOVE-RT ACTION CLIENT/SERVICES
        self.move_client = actionlib.SimpleActionClient('{}ee_execute_trajectory'.format(self.arm), ExecutingTrajectoryAction)
        self.move_client.wait_for_server()

        rospy.wait_for_service('{}EePosition/clearTrajectory'.format(self.arm))
        self.clearTraj = rospy.ServiceProxy('{}EePosition/clearTrajectory'.format(self.arm), TaskParamUpdate)

        rospy.wait_for_service('{}EmergencyStop/setEnable'.format(self.arm))
        self.emergencyEnable = rospy.ServiceProxy('{}EmergencyStop/setEnable'.format(self.arm), TaskParamUpdate)

        rospy.wait_for_service('{}EePosition/setEnable'.format(self.arm))
        self.eeEnable = rospy.ServiceProxy('{}EePosition/setEnable'.format(self.arm), TaskParamUpdate)

        rospy.wait_for_service('{}EePosition/setToolFrame'.format(self.arm))
        self.setToolFrame = rospy.ServiceProxy('{}EePosition/setToolFrame'.format(self.arm), UpdateFrame)
        print('MoveRT Connected')

        self.moveClientGoal = ExecutingTrajectoryGoal()
        self.moveClientGoal.trajectory_name = '/{}traj_wires'.format(self.arm)
        self.moveClientGoal.ee_error_th = 0.00001


        ### Switch sensor ####

        self.active_control_switch = control_switch
        if self.active_control_switch :
            print('Connection to tool switch sensor: Waiting')
            rospy.wait_for_message("/switch_sensor", UInt8)
            print('Connection to tool switch sensor: OK\n')
            self.sensor_subscribe = rospy.Subscriber("switch_sensor", UInt8, self.switch_sensor_cb)   

        print('Robot {} Initialization Completed'.format(self.arm))



    def switch_sensor_cb(self, data) :
        if data.data == 1 :
            rospy.loginfo_throttle(1, "!! contact !!")
            self.contact_signal = True
        else :
            self.contact_signal = False


    def execute_trajectory(self, poses_list = None, poses_array = None, ee_frame = None, switch_sensor = False, trans_vel = 0.02, rot_vel = 0.02 ,trajectory_name = 'move')

        if poses_list is None and poses_array is None:
            print('No point passed')
            return False

        del self.traj_list [:]



        # copy to list of poses
        final_poses_list = []
        if poses_list is not None :
            final_poses_list = [pose for pose in poses_list]
        elif poses_array is not None:
            final_poses_list = [pose for pose in poses_array.poses]
            
     

        # Copy to dictionary for parameters server
        for pose in final_poses_list :
            self.traj_point['pos_x'] = pose.position.x
            self.traj_point['pos_y'] = pose.position.y
            self.traj_point['pos_z'] = pose.position.z
            self.traj_point['orient_w'] = pose.orientation.w
            self.traj_point['orient_x'] = pose.orientation.x
            self.traj_point['orient_y'] = pose.orientation.y
            self.traj_point['orient_z'] = pose.orientation.z
            self.traj_list.append(copy.deepcopy(self.traj_point))





        self.emergencyEnable([0])
        self.eeEnable([1])
        self.clearTraj([0])   
        if ee_frame is not None : self.setTraj(ee_frame)

        self.set_velocity(trans_vel, rot_vel)

        traj_name = '/' + self.arm + trajectory_name
        self.moveClientGoal.trajectory_name = traj_name
        rospy.set_param(self.moveClientGoal.trajectory_name, self.traj_list)

        print('Execution start')
        self.move_client.send_goal(self.moveClientGoal)
        contact_reached = False
        if self.active_control_switch :
            while  not self.contact_signal and not contact_reached:
                if self.contact_signal :
                    self.move_client.cancel_all_goals()
                    contact_reached = True
                    break
        else :
            self.move_client.wait_for_result()
        print('Execution completed')

        self.emergencyEnable([1])
        self.eeEnable([0])      

        return True




    def get_EE_position(self, frame) :
        self.clearTraj()
        self.setToolFrame(frame) 
        ee_vector_pos = rospy.wait_for_message('{}ee_position'.format(self.arm), Float64MultiArray)

        EE_pose = Pose()
        EE_pose.position.x = ee_vector_pos.data[0]
        EE_pose.position.y = ee_vector_pos.data[1]
        EE_pose.position.z = ee_vector_pos.data[2]
        EE_pose.orientation.w = ee_vector_pos.data[3]
        EE_pose.orientation.x = ee_vector_pos.data[4]
        EE_pose.orientation.y = ee_vector_pos.data[5]
        EE_pose.orientation.z = ee_vector_pos.data[6]

        return EE_pose



    def set_velocity(self, trans_vel, rot_vel) :
        rospy.set_param('/{}EePosition/translation_velocity'.format(self.arm), trans_vel)
        rospy.get_param('/{}EePosition/rotation_velocity'.format(self.arm), rot_vel)
        return 

if __name__ == '__main__' :


    print('rip')