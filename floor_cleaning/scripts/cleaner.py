#! /usr/bin/env python
'''
Citatation: the velocity controller class and 
the odemetryReader classes were taken from chapter 5 with minor tweaks

on line 98 the factor that I change the radius was adapted from the provided solution 
but using 0.0030 was my own idea
'''
import rospy, math, numpy as np
from utilities import set_position, cleaned_area
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

rospy.init_node('vaccum_cleaner', anonymous=True)

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        self.max_vel = 0.65
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        abs_v = abs(linear_velocity)
        if abs_v <= self.max_vel:
            vx = linear_velocity
            wz = angular_velocity
        else:
            vx = linear_velocity / abs_v * self.max_vel
            wz = angular_velocity / abs_v * self.max_vel
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel.publish(msg)
    #in case you need to stop the robot
    def stop_turtle_bot(self):
        msg = Twist()
        #stop the turtle bot
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.z = 0
        
        self.cmd_vel.publish(msg)
        # rospy.loginfo("cmd_vel stopped")
        
    
class OdometryReader():
    def __init__(self, topic):
        self.pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.pose['x'], self.pose['y']))
        (_, _, self.pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.subscriber.unregister()

############## Feed back control functions #######################
def normalize(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))

def go_to(xg, yg, thetag_degrees, constant_vel = None):
    k_rho = 0.3 # Kp value
    k_alpha = 0.8 #Ka value
    k_beta = -0.15 # Kb Value
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    while rho>0.01:
        dx = xg - odometry.pose['x']
        dy = yg - odometry.pose['y']
        rho = np.sqrt(dx**2 + dy**2)
        theta = odometry.pose['theta']
        alpha = normalize(np.arctan2(dy, dx) - theta)
        beta = normalize(thetag - np.arctan2(dy, dx))
        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta
        velocity.move(v, w)
        rospy.sleep(0.01)
################################################################
#### open loop controller#########
def spiral(r):
    v = 0.65
    r_initial = 0.1
    while r_initial <= r:
        w = v/r_initial
        velocity.move(v,w)
        r_initial += 0.0030/r_initial # this number determines how distance between each spiral
        # print(r_initial)
        rospy.sleep(0.1)
####### Main steps ##############
set_position(0,0,0)
velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')
init_time = rospy.get_time()

try:
    predefined_positions = [(-2,-1,180),(-3,0,90),(3,0,90),(2,-1,-90),(2,-5,-90),(-1,-7,-90),(3,-7,-90),(3,-10,-90)]
    r = [1.3,1.3,1.0,1.3,1.3,3,1.3,1.4]
    spiral(1.3)
    for r, pos in zip(r,predefined_positions):
        xg,yg,thetag = pos
        go_to(xg,yg,thetag)
        spiral(r)
    spiral(1.3)
except KeyboardInterrupt:
    pass

  
end_time = rospy.get_time()
velocity.stop_turtle_bot()
odometry.unregister()
t = end_time-init_time
m = int(t/60)
s = t - m*60
print("You cleaned %.2f m2 in %d minutes and %d seconds." % 
      (cleaned_area(odometry.trajectory), m, s))