#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from dobot_py import DobotClient as dc
from dobot_py.jogMode import JOGMode
import sys, select, termios, tty

# Define joint limits
joint_limits = {
    'magician_joint_1': (-1.57, 1.57),  # Limits equivalent to (-90 degrees, 90 degrees)
    'magician_joint_2': (0.3, 1.0122),
    'magician_joint_3': (-1.57, 1.2),
}

# Global variable to store the last positions
last_positions = [0.0, 0.0, 0.0]

def enforce_limits(joint, value):
    lower, upper = joint_limits[joint]
    return max(lower, min(upper, value))

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def handle_keyboard_input(pub, settings):
    global last_positions
    isJoint = False

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['magician_joint_1', 'magician_joint_2', 'magician_joint_3']
    joint_state.position = list(last_positions)  # Use last known positions

    key = getKey(settings)
    if key == 'w':
        dc.set_jog_cmd(isJoint, JOGMode.FOWARD.value)
        joint_state.position[0] = enforce_limits('magician_joint_1', joint_state.position[0] - 0.01)
    elif key == 's':
        dc.set_jog_cmd(isJoint, JOGMode.BACK.value)
        joint_state.position[0] = enforce_limits('magician_joint_1', joint_state.position[0] + 0.01)
    elif key == 'a':
        dc.set_jog_cmd(isJoint, JOGMode.LEFT.value)
        joint_state.position[1] = enforce_limits('magician_joint_2', joint_state.position[1] - 0.01)
    elif key == 'd':
        dc.set_jog_cmd(isJoint, JOGMode.RIGHT.value)
        joint_state.position[1] = enforce_limits('magician_joint_2', joint_state.position[1] + 0.01)
    elif key == 'r':
        dc.set_jog_cmd(isJoint, JOGMode.UP.value)
        joint_state.position[2] = enforce_limits('magician_joint_3', joint_state.position[2] - 0.01)
    elif key == 'f':
        dc.set_jog_cmd(isJoint, JOGMode.DOWN.value)
        joint_state.position[2] = enforce_limits('magician_joint_3', joint_state.position[2] + 0.01)
    else:
        dc.set_jog_cmd(isJoint, JOGMode.STOP.value)
    rospy.loginfo(f"After update: {joint_state.position}")
    # Update global last positions
    last_positions = joint_state.position
    pub.publish(joint_state)

def main():
    rospy.init_node('dobot_keyboard_control')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    settings = termios.tcgetattr(sys.stdin)  # Capture initial terminal settings

    try:
        while not rospy.is_shutdown():
            handle_keyboard_input(pub, settings)
            rospy.Rate(10).sleep()  # Sleep to maintain 10 Hz publish rate

    except rospy.ROSInterruptException as e:
        rospy.logerr(f"ROS interruption: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore the terminal settings
        rospy.signal_shutdown("Keyboard interrupted")

if __name__ == "__main__":
    main()
