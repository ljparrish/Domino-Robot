#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32

class VacuumGripper():
    def __init__(self):
        # Parameter Initialization
        self.currentPressure = 0
        self.pressureThreshold = -25.0

        # Setup Publisher and subscriber Objects
        # Pressure Sensor Subscriber
        self.pressureSubscriber = rospy.Subscriber('pressure',Float32,self.pressureCallback)

        # Motor Control Publisher
        self.vacuumControler = rospy.Publisher('vacuum_state',Int8,queue_size=10)
        self.off()
    
    def pressureCallback(self,pressureData):
        self.currentPressure = pressureData.data

    def getCurrentPressure(self):
        return self.currentPressure
    
    def on(self):
        self.vacuumControler.publish(1)
    
    def off(self):
        self.vacuumControler.publish(0)

    def isGripping(self):
        if self.getCurrentPressure() < self.pressureThreshold:
            return True
        else:
            return False

if __name__ == "__main__":
    rospy.init_node("gripper_test")
    # Creates a simple object of VacuumGripper and runs through a basic self test
    testGripper = VacuumGripper()
    print('Gripper Object Initialized...')
    print('Basic Self Test Started')
    print('Gripper Turning on, ')
    testGripper.on()
    rospy.sleep(5)
    print('Gripper Turning Off')
    testGripper.off()
    print('Testing Pressure Feedback, Attach Item to Gripper')
    testGripper.on()
    while not testGripper.isGripping():
        print(f"Pressure : {testGripper.currentPressure} kPa")
        rospy.sleep(0.1)

    print(f"Grasp Successful at {testGripper.currentPressure} for threshold of {testGripper.pressureThreshold}")
    print('Testing Max Vacuum Capability ... ')
    rospy.sleep(10)
    print(f"Max Vacuum Recorded as {testGripper.getCurrentPressure()}")
    testGripper.off()
    print("Basic Self Test Complete!") 