#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

CTRL-C to quit
"""

moveBindings = {
    'i': ('magician_joint_1', 0.01),
    'k': ('magician_joint_1', -0.01),
    'j': ('magician_joint_2', 0.01),
    'l': ('magician_joint_2', -0.01),
    'u': ('magician_joint_3', 0.01),
    'o': ('magician_joint_3', -0.01),
}

# Define joint limits
joint_limits = {
    'magician_joint_1': (-1.57, 1.57),  # Example limits (-90 degrees, 90 degrees)
    'magician_joint_2': (0.3, 1.0122),
    'magician_joint_3': (-1.57, 1.2),
}

# Initialize current positions for each joint
current_positions = {name: 0.0 for name, _ in moveBindings.values()}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def enforce_limits(joint, value):
    lower, upper = joint_limits[joint]
    return max(lower, min(upper, value))

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('robot_teleop')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings:
                joint_name, increment = moveBindings[key]
                current_positions[joint_name] += increment
                current_positions[joint_name] = enforce_limits(joint_name, current_positions[joint_name])  # Enforce limits
                joint_state = JointState()
                joint_state.header.stamp = rospy.Time.now()
                joint_state.name = list(current_positions.keys())
                joint_state.position = list(current_positions.values())
                pub.publish(joint_state)
                rospy.loginfo('Updated positions: {}'.format(list(zip(joint_state.name, joint_state.position))))
            elif key == '\x03':
                break

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
