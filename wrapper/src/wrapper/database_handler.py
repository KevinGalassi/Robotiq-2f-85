#! /usr/bin/env python3


import rospy


from database_wires.srv import *
from database_wires.msg import *

from std_srvs.srv import Trigger
from std_msgs.msg import  Bool


class databaseHandler(object) :


    def __init__(self) :  
        # DATABASE SERVICES
        print('Database Service: Waiting\n')
        rospy.wait_for_service('gearbox_handler/reset_connection_list')
        self.resetList = rospy.ServiceProxy('gearbox_handler/reset_connection_list', Trigger)                       # ask for next connection pin in the list

        rospy.wait_for_service('gearbox_handler/reset_to_connection_list')
        self.resetToList = rospy.ServiceProxy('gearbox_handler/reset_to_connection_list', DataSrv) 

        rospy.wait_for_service('gearbox_handler/connection_provider_service')
        self.nextConnection = rospy.ServiceProxy('gearbox_handler/connection_provider_service', connection_data)    # ask for next connection pin in the list

        rospy.wait_for_service('gearbox_handler/get_item_screw_targets')
        self.pinPoses = rospy.ServiceProxy('gearbox_handler/get_item_screw_targets', pin_data)                      # get screw poses for a component w.r.t its real pose

        rospy.wait_for_service('gearbox_handler/rail_scanner')
        self.new_rail = rospy.ServiceProxy('gearbox_handler/rail_scanner', rail_scan)

        rospy.wait_for_service('/gearbox_handler/reset_rail_scanner')
        self.reset_rail = rospy.ServiceProxy('/gearbox_handler/reset_rail_scanner', Trigger)
        print('Database Service: OK\n')


    def reset_connection_list(self) :
        self.resetList()
        return True

    def reset_to_connection_list(self, req) :
        self.resetToList(req)
        return True

    def new_connection_provider(self) :
        return self.nextConnection()

    def get_item_screw_targets(self, req) :
        return self.pinPoses(req)

    def rail_scanner(self, req) :
        return self.new_rail(req)
    
    def reset_rail_scanner(self) :
        return self.reset_rail()

    def test(self) :
        print('test ok')
        return True

if __name__ == '__main__' :

    rospy.init_node('database_handler_test')

