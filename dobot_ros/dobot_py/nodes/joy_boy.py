#!/usr/bin/env python3
import rospy
import pygame
from sensor_msgs.msg import JointState
from dobot_py import DobotClient as dc, JOGMode

def handle_joystick_input(joystick, pub, is_real):
    axis0 = joystick.get_axis(0)
    axis1 = joystick.get_axis(1)
    axis2 = joystick.get_axis(2)
    axis3 = joystick.get_axis(3)

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3', 'magician_joint_4']
    joint_state.position = [0.0, 0.0, 0.0, 0.0]

    if axis1 < -0.5:
        increment = 0.01
    elif axis1 > 0.5:
        increment = -0.01
    elif axis0 < -0.5:
        increment = 0.01
    elif axis0 > 0.5:
        increment = -0.01
    elif axis2 < -0.5:
        increment = 0.01
    elif axis2 > 0.5:
        increment = -0.01
    elif axis3 < -0.5:
        increment = 0.01
    elif axis3 > 0.5:
        increment = -0.01
    else:
        increment = 0
        dc.set_jog_cmd(False, JOGMode.STOP.value)

    for i in range(4):
        joint_state.position[i] += increment

    pub.publish(joint_state)
    
    if is_real:
        # Pengaturan khusus untuk robot Dobot nyata
        dc.set_jog_cmd(True, increment > 0 and JOGMode.FOWARD.value or JOGMode.BACK.value)

def main():
    rospy.init_node('dobot_joystick_control')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    is_real = rospy.get_param('~real_hardware', False)

    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        rospy.logerr("No joystick detected!")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("Joystick quit event")
            elif event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(0):  # Homing
                    if is_real:
                        dc.set_home_cmd()
                elif joystick.get_button(1):  # Get pose
                    if is_real:
                        rospy.loginfo(dc.get_pose())

        handle_joystick_input(joystick, pub, is_real)
        rate.sleep()

    pygame.quit()

if __name__ == "__main__":
    main()
