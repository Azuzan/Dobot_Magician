#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import pygame
import sys

msg = """
Control Your Robot with Joystick!
---------------------------
Move the joystick or press buttons to control the robot.
"""

# Define joystick mappings to joint movements
axis_bindings = {
    0: ('magician_joint_1', 0.01),  # Assume axis 0 controls magician_joint_1
    1: ('magician_joint_2', 0.01),  # Assume axis 1 controls magician_joint_2
}
button_bindings = {
    0: ('magician_joint_3', 0.01),  # Assume button 0 controls magician_joint_3
    # Removed magician_joint_4 bindings
}

# Initialize current positions for each joint
current_positions = {name: 0.0 for _, name in axis_bindings.values()}

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("No joystick detected")
    sys.exit()
joystick = pygame.joystick.Joystick(0)
joystick.init()

def get_joystick_inputs():
    pygame.event.pump()
    inputs = {}
    # Read axes
    for axis, (joint, increment) in axis_bindings.items():
        axis_value = joystick.get_axis(axis)
        inputs[joint] = axis_value * increment
    # Read buttons
    for button, (joint, increment) in button_bindings.items():
        if joystick.get_button(button):
            inputs[joint] = increment
    return inputs

if __name__ == "__main__":
    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    try:
        print(msg)
        while not rospy.is_shutdown():
            inputs = get_joystick_inputs()
            for joint, value in inputs.items():
                current_positions[joint] += value

            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = list(current_positions.keys())
            joint_state.position = list(current_positions.values())
            pub.publish(joint_state)
            rospy.loginfo('Updated positions: {}'.format(list(zip(joint_state.name, joint_state.position))))

    except Exception as e:
        print(e)

    finally:
        pygame.quit()
