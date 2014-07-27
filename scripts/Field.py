#!/usr/bin/env python
import rospy
import operator
import math
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p4.msg import *
from cs1567p4.srv import *

# coordinates the robots, handles state transitions 
class Commander(object):
    def __init__(self):
     ## TODO : MEASURE DISTANCE FROM THE CENTER
        NORTH_WALL = (0,0)
        RIGHT_WALL = (0,0)
        BOTTOM_WALL = (0,0)
        LEFT_WALL = (0,0)
        
        ## OFFSET FOR WHEN 
        X_WALL_OFFSET = (0,0)
        Y_WALL_OFFSET = (0,0)
        
        super(Commander, self).__init__()
        # array of robot data
        self.robots = [Robot(Robot.HUMAN), Robot(Robot.HUMAN), Robot(Robot.HUMAN)]
        self.robotDistances = list()
        
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
        
        return PotentialFieldResponse(potential)

    def handle_bump(self, msg):
        robot = self.robots[msg.robotID]
        rospy.loginfo('Bump: ' + str(msg.robotID))
        # handle bump

    def cache_locations(self, msg):
        for location in msg.robots:
            # store the location in the robot object
            self.robots[location.robot_num].location = location


    
    def calc_potential(self,other):
           for x in range(0, 2):
                        
                if self.robots[x] is robot: ## case 1 : robot is the one calling calc_potential.
                    continue                           ## do nothing.
                
                ## handle robots
                dx = robots.getX() - self.robots[x].getX         ##case 2: get robot x and y coord.
                dy = robots.getY() - self.robots[y].getY         
                
                distance = math.sqrt(math.pow(dx,2) + math.pow(dy,2))     ## calc distance
                force = 1/math.pow(distance,2)
                
                if self.robots[x].state is 0 and robot.state is 0:      ## make negative if robot called is in the same state at current robot
                    force = -distance
                
                if self.robots[x].state is 1 and robot.state is 1:
                    force = -distance    
           
                self.robotDistances.append(force)                    ## append to list
                        
               
               
                    
    return        
    
    def wallForces(self,current):
        
        
            if current.getX < NORTH_WALL - Y_WALL_OFFSET:
                pass
           
            if current.getY < NORTH_WALL - Y_WALL_OFFST:
                pass
    
            if current.getY > NORTH_WALL - Y_WALL_OFFSET:
                robotDistances.append(-1/(NORTH_WALL - current.getX))
            
            if current.getY > RIGHT_WALL - Y_WALL_OFFSET:
                robotDistances.append(1/(NORTH_WALL - current.getX))
            
            if current.getY > NORTH_WALL - Y_WALL_OFFSET:
                robotDistances.append(1/(NORTH_WALL - current.getX))
            
            
            
                return
                
    return


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
            pass
        # same status, slight repulsion
        elif self.status is other.status:
            pass
        # zombie chases human, strong attraction
        elif self.status is ZOMBIE and other.status is HUMAN:
            pass
        # human runs from zombie, strong repulsion
        else:
            pass
        # todo: actually calculate potential
        return (0,0)
    
        self.name = name  ## name of the robot
        self.state = state    ## int 0 = human 1 =  zombie
        self.currDirection = currDirection # current direction vector
        self.x = x    # current x coordinate
        self.y = y    # current y coordinate
        self.speed = speed;
      
      
   ## setters n getters   
    def getName(self):
        return self.name
    
    def getState(self): 
        return self.state
    
    def getCurrDirection(self):
        return self.currDirection
    
    def getState(self): 
        return self.state
   
    def getSpeed(self): 
        return self.speed
  
    def getX(self): 
        return self.x
  
    def getY(self): 
        return self.y
   
    def setName(self):
        self.name = name
    
    def setState(self,state): 
        self.state = state
    
    def setCurrDirection(self,newDirection):
         self.currDirection = newDirection
    
    def setState(self,newState): 
        self.state = newState
   
    def setSpeed(self,newSpeed): 
        self.speed = newSpeed
     
    # calculates the net force of all other robots and the 4 walls on this robot
    def getNetForce(self):
       return  
     
   
   # calculates 1/x^2 distance
    def getAttraction(self,object):
        return
   
    
    
            
if __name__ == "__main__":
    try:
        cmdr = Commander()
        rospy.spin()
    except rospy.ROSInterruptException: 
        print 'dead'