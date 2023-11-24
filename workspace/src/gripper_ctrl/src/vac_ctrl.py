#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32

class VacuumGripper():
    def __init__(self):
        # Initialize Pressure Data
        self.currentPressure = 0

        # Setup Publisher and subscriber Objects
        # Pressure Sensor Subscriber
        self.pressureSubscriber = rospy.Subscriber('pressure',Float32,self.pressureCallback)

        # Motor Control Publisher
        self.vacuumControler = rospy.Publisher('vacuum_state',Int8,queue_size=10)
    
    def pressureCallback(self,pressureData):
        self.currentPressure = pressureData.data

    def getCurrentPressure(self):
        return self.currentPressure
    
    def on(self):
        self.vacuumControler.publish(1)
    
    def off(self):
        self.vacuumControler.publish(0)
