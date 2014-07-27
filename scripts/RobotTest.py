#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
from cs1567p4.msg import *
from cs1567p4.srv import *
import numpy

class RobotTest(object):
    def __init__(self):
        super(RobotTest, self).__init__()
        rospy.init_node('RobotTest', anonymous=True)
        
        self.id = rospy.get_param('~robot_id')
        self.name = rospy.get_param('~robot_name')

        self.bump_sub = rospy.Subscriber('/' + self.name + '_base/events/bumper', BumperEvent, self.on_bump)
        self.bump_pub = rospy.Publisher('/robots/bumps', Bump, queue_size=10)
        self.odom_pub = rospy.Publisher('/' + self.name + '_base/commands/velocity', Twist, queue_size=10)
        
        rospy.wait_for_service('potential_field')
        self.potential = rospy.ServiceProxy('potential_field', PotentialField)

        self.loc_sub = rospy.Subscriber('/robots/location', LocationList, self.update_location)

    def update_location(self, msg):
        #print('received new location.')
        for location in msg.robots:
            if location.robot_num is self.id:
                self.location = location
                break
        self.update_potential()
        

    def on_bump(self, msg):
        if msg.state is BumperEvent.PRESSED:
            self.bump_pub.publish(self.id)
            rospy.loginfo('{} has bumped.'.format(self.name))

    def update_potential(self):
        potential = self.potential(self.id).potential
        rospy.loginfo("potential - x: {} y: {}".format(potential[0], potential[1]))
        self.act(potential)

    def act(self,potential):
        twist = Twist()
        twist.linear.x = potential[0]
        self.odom_pub.publish(twist)

if __name__ == "__main__":
    r = RobotTest()
    rospy.spin()
