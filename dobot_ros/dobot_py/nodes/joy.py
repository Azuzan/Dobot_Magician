#!/usr/bin/env python3
from __future__ import print_function

import pygame
from dobot_py import DobotClient as dc
from dobot_py.jogMode import JOGMode

def handle_joystick_input(joystick):
    isJoint = False
    axis0 = joystick.get_axis(0)
    axis1 = joystick.get_axis(1)
    axis2 = joystick.get_axis(2)
    axis3 = joystick.get_axis(3)
    
    if axis1 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.FOWARD.value)
    elif axis1 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.BACK.value)
    elif axis0 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.LEFT.value)
    elif axis0 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.RIGHT.value)
    elif axis2 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.UP.value)
    elif axis2 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.DOWN.value)
    elif axis3 < -0.5:
        dc.set_jog_cmd(isJoint, JOGMode.LEFT_ROLL.value)
    elif axis3 > 0.5:
        dc.set_jog_cmd(isJoint, JOGMode.RIGHT_ROLL.value)
    else:
        dc.set_jog_cmd(isJoint, JOGMode.STOP.value)

def main():
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No joystick detected!")
        return
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(0):  # Button 0 for homing
                    dc.set_home_cmd()
                elif joystick.get_button(1):  # Button 1 for get pose
                    print(dc.get_pose())
        
        handle_joystick_input(joystick)
        
    pygame.quit()

if __name__ == "__main__":
    main()

