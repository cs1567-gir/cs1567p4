#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
from cs1567p4.msg import *
from cs1567p4.srv import *

class RobotTest(object):
    def __init__(self, id, name):
        super(RobotTest, self).__init__()
        rospy.init_node('RobotTest', anonymous=True)
        self.id = 0
        self.name = name        
        self.bump_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.on_bump)
        self.bump_pub = rospy.Publisher('/robots/bumps', Bump, queue_size=10)
        rospy.wait_for_service('potential_field')
        self.potential = rospy.ServiceProxy('potential_field', PotentialField)

    def on_bump(self, msg):
        if msg.state is BumperEvent.PRESSED:
            self.bump_pub.publish(self.id)
            rospy.loginfo('bump in robot')

    def run(self):
        while not rospy.is_shutdown():
            resp = self.potential(self.id)
            #rospy.loginfo("x: {} y: {}".format(resp.x, resp.y))

if __name__ == "__main__":
    r = RobotTest(0, 'rosie')
    r.run()