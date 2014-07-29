#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
from cs1567p4.msg import *
from cs1567p4.srv import *
import math
import numpy

class RobotTest(object):
    def __init__(self):
        super(RobotTest, self).__init__()
        rospy.init_node('RobotTest', anonymous=True)
        
        self.id = rospy.get_param('~robot_id')
        self.name = rospy.get_param('~robot_name')

        self.bump_sub = rospy.Subscriber('/' + self.name + '_base/events/bumper', BumperEvent, self.on_bump)
        self.bump_pub = rospy.Publisher('/robots/bumps', Bump, queue_size=10)

        rospy.wait_for_service('potential_field')
        self.potential = rospy.ServiceProxy('potential_field', PotentialField)

        rospy.wait_for_service('constant_command_' + self.name)
        self.const_cmd = rospy.ServiceProxy('constant_command_' + self.name, ConstantCommand)

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
        twist.linear.x = .2
        epsilon = 1E-3

        wanted_angle = math.atan2(potential[1], potential[0])
        curr_angle = self.location.theta

        diff_angle = wanted_angle - curr_angle

        if diff_angle > math.pi:
            diff_angle -= 2*math.pi
        if diff_angle < -math.pi:
            diff_angle += 2*math.pi

        print 'diff_angle: {}'.format(diff_angle)

        if math.fabs(diff_angle) > epsilon:
            #twist.linear.x *= math.fabs((diff_angle - math.pi) / math.pi)
            twist.angular.z = 0.7 * (diff_angle / math.pi)
            
        self.const_cmd(twist)

if __name__ == "__main__":
    r = RobotTest()
    rospy.spin()
