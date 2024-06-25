#!/usr/bin/env python3
import rospy
import pygame
from sensor_msgs.msg import JointState
from dobot_py import DobotClient as dc
from dobot_py.jogMode import JOGMode

def handle_joystick_input(joystick, pub):
    isJoint = False
    axis0 = joystick.get_axis(0)
    axis1 = joystick.get_axis(1)
    axis2 = joystick.get_axis(2)
    axis3 = joystick.get_axis(3)
    
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3', 'magician_joint_4']  # Use exact joint names from URDF
    joint_state.position = [0.0, 0.0, 0.0, 0.0]  # Initialize with current positions

    if axis1 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.FOWARD.value)
        joint_state.position[0] += 0.01
    elif axis1 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.BACK.value)
        joint_state.position[0] -= 0.01
    elif axis0 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.LEFT.value)
        joint_state.position[1] += 0.01
    elif axis0 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.RIGHT.value)
        joint_state.position[1] -= 0.01
    elif axis2 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.UP.value)
        joint_state.position[2] += 0.01
    elif axis2 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.DOWN.value)
        joint_state.position[2] -= 0.01
    elif axis3 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.LEFT_ROLL.value)
        joint_state.position[3] += 0.01
    elif axis3 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.RIGHT_ROLL.value)
        joint_state.position[3] -= 0.01
    else:
        dc.set_jog_cmd(isJoint, JOGMode.STOP.value)
    
    pub.publish(joint_state)

def main():
    rospy.init_node('dobot_joystick_control')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        rospy.logerr("No joystick detected!")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("Joystick quit event")
            elif event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(0):  # Button 0 for homing
                    dc.set_home_cmd()
                elif joystick.get_button(1):  # Button 1 for get pose
                    rospy.loginfo(dc.get_pose())
        
        handle_joystick_input(joystick, pub)
        rate.sleep()
    
    pygame.quit()

if __name__ == "__main__":
    main()

