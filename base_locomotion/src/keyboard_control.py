#!/usr/bin/env python

# import rospy
# from std_msgs.msg import Float32
# import sys
# import select
# import termios
# import tty

# key = 0
# pressed = False

# def get_keyboard_input():
#     global key, pressed
#     if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
#         key = sys.stdin.read(1)
#         pressed = True
#     else:
#         key = ''
#         pressed = False


# def keyboard_input():
#     pub = rospy.Publisher('cmd_input', Float32, queue_size=10)
#     rospy.init_node('keyboard_control', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
    
#     while not rospy.is_shutdown():
#         get_keyboard_input()
#         if pressed:
#             try:
#                 value = float(key)
#                 pub.publish(value)
#             except ValueError:
#                 pass
#         else:
#             pub.publish(0.0)

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         keyboard_input()
#     except rospy.ROSInterruptException:
#         pass


import rospy
from std_msgs.msg import Float32
import curses

def main(stdscr):
    # Initialize ROS node and publisher
    rospy.init_node('keyboard_input')
    pub = rospy.Publisher('cmd_input', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    # Turn off input echoing and enable special keys
    curses.noecho()
    stdscr.keypad(True)

    while not rospy.is_shutdown():
        # Read a single character of keyboard input
        char = stdscr.getch()

        # Convert the character to a float value and publish to topic
        try:
            value = float(chr(char))
        except ValueError:
            value = 0.0
        pub.publish(value)

        rate.sleep()

if __name__ == '__main__':
    # Initialize the curses library and run the main function
    curses.wrapper(main)