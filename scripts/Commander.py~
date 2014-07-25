#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p4.msg import *
from cs1567p4.srv import *

# coordinates the robots, handles state transitions 
class Commander(object):
    def __init__(self):
        super(Commander, self).__init__()
        # array of robot data
        self.robots = [RobotData(), RobotData(), RobotData()]
        
        rospy.init_node('Commander', anonymous=True)
        rospy.Service('potential_field', PotentialField, self.calc_potential)
        # todo: which topic is localization publishing
        rospy.Subscriber('/robots/location', LocationList, self.cache_locations)
        # todo: which topic should we publish bumps on
        self.bump_sub = rospy.Subscriber('/robots/bumps', Bump, self.handle_bump)     
        

    def calc_potential(self, request):
        robot = self.robots[request.robotID]

        # calculate net potential
        return [0.0, 0.0]

    def handle_bump(self, msg):
        robot = self.robots[msg.robotID]
        print 'bump: ' + str(msg.robotID)
        rospy.loginfo('Bump: ' + str(msg.robotID))
        # handle bump

    def cache_locations(self, msg):
        for location in msg.robots:
            # store the location in the robot object
            self.robots[location.robot_num].location = location

# holds data about a robot
class RobotData(object):
    ZOMBIE = 1
    HUMAN = 2
    
    def __init__(self):    
        super(RobotData, self).__init__()   
        self.location = None
        self.status = None


if __name__ == "__main__":
    try:
        cmdr = Commander()
        rospy.spin()
    except rospy.ROSInterruptException: 
        print 'dead'
