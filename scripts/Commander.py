#!/usr/bin/env python
import rospy
import operator
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p4.msg import *
from cs1567p4.srv import *

# coordinates the robots, handles state transitions 
class Commander(object):
    def __init__(self):
        super(Commander, self).__init__()
        # array of robot data
        self.robots = [Robot(Robot.HUMAN), Robot(Robot.HUMAN), Robot(Robot.HUMAN)]
        rospy.init_node('Commander', anonymous=True)
        rospy.Service('potential_field', PotentialField, self.calc_potential)
        rospy.Subscriber('/robots/location', LocationList, self.cache_locations)
        self.bump_sub = rospy.Subscriber('/robots/bumps', Bump, self.handle_bump)

    def calc_potential(self, request):
        # robot that requested potential
        robot = self.robots[request.robotID]

        # gather potentials from other robots
        potentials = []
        for other_robot in self.robots:
            potentials.append(robot.get_potential(other_robot))

        # take list of 2-element tuples and add them
        potential = reduce(lambda x, y: (x[0]+y[0], x[1]+y[1]), potentials)
        norm = math.sqrt(potential[0]**2 + potential[1]**2)
        normalized_potential = tuple(val/norm for val in potential)
        return PotentialFieldResponse(normalized_potential)

    def handle_bump(self, msg):
        robot = self.robots[msg.robotID]
        rospy.loginfo('Bump: ' + str(msg.robotID))
        # handle bump

    def cache_locations(self, msg):
        for location in msg.robots:
            # store the location in the robot object
            self.robots[location.robot_num].location = location

# holds data about a robot
class Robot(object):
    HUMAN = 1
    ZOMBIE = 2
    def __init__(self, status):    
        super(Robot, self).__init__()   
        self.location = None
        self.status = status

    def collide(self, other):
        if self.status is ZOMBIE and other.status is HUMAN:
            other.status = ZOMBIE

    def get_potential(self, other):
        # same robot, no repulsion
        if self is other:
            return (0,0)
        dx = self.getX() - other.getX()
        dy = self.getY() - other.getY()
        r = numpy.array([dx, dy])
        r_hat = r/numpy.linalg.norm(direction)
        force = 1/numpy.linalg.norm(r)**2
        force_vector = force*r_hat
        if self.status == self.ZOMBIE and robot.status == self.HUMAN:
            force = -1*force                    #By default, this vector will repel. But zombies must pursue humans.
        if self.status == other.status:
            pass                                #Consider scaling down the effect of like-robots
        return (force_vector[0], force_vector[1])

if __name__ == "__main__":
    try:
        cmdr = Commander()
        rospy.spin()
    except rospy.ROSInterruptException: 
        print 'dead'
