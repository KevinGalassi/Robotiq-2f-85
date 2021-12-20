#! /usr/bin/env python3


from time import sleep
import rospy

# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver

def test_gripper() :

    print('Stat test!')
    # Control that the action server existss (Optional)
    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    print('Wait {}'.format(action_name))
    robotiq_client.wait_for_server()
    print('Gripper action server exist')



    gripper = Robotiq2FingerGripperDriver(comport='/dev/ttyUSB0', init_requested=False)

    input('Enter to start')

    gripper.open()

    print('Second mode, block expected')
    input('Enter to start')

    gripper.goto(pos=0.00, speed=0.01, force=100, block=False)
    gripper.goto(pos=0.08, speed=0.01, force=100, block=False)

    print('Third mode')
    input('Enter to start, non block expected')
    gripper.close(speed=0.01, block = False)
    gripper.open(block=False)



if __name__ == '__main__' :

    rospy.init_node('robotiq_2f_client_example_command')


    result = test_gripper()


    print('Test concluded')