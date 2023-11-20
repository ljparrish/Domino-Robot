#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32


def pressureSignalCallback(message):
    print('Vacuum Pressure = ',message)


def pressureSensorListener():
    rospy.Subscriber('pressure',Float32,pressureSignalCallback)
    rospy.spin()

def vacuumController():
    ctrl_pub = rospy.Publisher('vacuum_state',Int8,queue_size=10)
    r = rospy.Rate(100) # 100 Hz Sample Frequency

    while not rospy.is_shutdown():
        cmd = input("Enter a 1 to turn on the pump!")
        if cmd == '1':
            ctrl_pub.publish(1)
        else:
            ctrl_pub.publish(0)
        r.sleep()

if __name__ == '__main__':
    
    # Run this program as a new node in the ROS computation graph called.
    rospy.init_node('vacuum_ctrl', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # method.
    try:
        vacuumController()
        pressureSensorListener()
    except rospy.ROSInterruptException: pass
