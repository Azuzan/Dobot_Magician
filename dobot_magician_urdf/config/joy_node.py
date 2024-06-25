#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

# Publisher untuk setiap sendi
publishers = {
    'joint1': rospy.Publisher('/dobot/joint1_position_controller/command', Float64, queue_size=10),
    'joint2': rospy.Publisher('/dobot/joint2_position_controller/command', Float64, queue_size=10),
    'joint3': rospy.Publisher('/dobot/joint3_position_controller/command', Float64, queue_size=10),
    'joint4': rospy.Publisher('/dobot/joint4_position_controller/command', Float64, queue_size=10)
    # Tambahkan lebih banyak jika ada lebih banyak sendi
}

def joy_callback(data):
    # Dapatkan nilai dari joystick
    # Anggap sumbu X mengontrol joint1, Y untuk joint2, dsb.
    positions = {
        'joint1': data.axes[0] * 1.0,  # Skalakan dan sesuaikan nilai
        'joint2': data.axes[1] * 1.0,
        'joint3': data.axes[2] * 1.0,
        'joint4': data.axes[3] * 1.0
        # Tambahkan lebih banyak jika ada lebih banyak sendi
    }

    # Terbitkan nilai ke setiap sendi
    for joint, pos in positions.items():
        publishers[joint].publish(pos)

def main():
    rospy.init_node('dobot_joy_control')
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

