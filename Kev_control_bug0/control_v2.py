#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32
import numpy as np 

class RobotControl(): 
    #This class implements the differential drive model of the robot 
    def __init__(self): 

        rospy.on_shutdown(self.cleanup) 

        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 
        
        ############ Variables ############### 
        self.pos_act = Point()
        self.pos_target = Point()
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad]

        self.x_target = 2.5 #x position of the goal 
        self.y_target = 5 #y position of the goal 

        self.goal_received = 0 #flag to indicate if the goal has been received 

        self.wr = 0.0
        self.wl = 0.0
        self.wr_n = 0.0
        self.wl_n = 0.0

        self.current_state = 'Moving Robot' #Robot's current state 
        
        ###******* INIT PUBLISHERS *******###  
        
        self.pub_gtg = rospy.Publisher('gtg', Twist, queue_size=1)
        self.pub_ed = rospy.Publisher('ed', Float32, queue_size=1)
        self.pub_et = rospy.Publisher('et', Float32, queue_size=1) 
        
        self.pub_pos_target = rospy.Publisher('pos_target', Point, queue_size=1)
        self.pub_pos_act = rospy.Publisher('pos_act', Point, queue_size=1)

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)
        #rospy.Subscriber("Aruco_pos_real", Vector3, self.aruco_cb)

        #********** INIT NODE **********###  
        freq = 50 
        rate = rospy.Rate(freq) #freq Hz  
        self.dt = 1.0/float(freq) #Dt is the time between one calculation and the next one 

        ################ MAIN LOOP ################  
        while not rospy.is_shutdown():
            
            v = self.r*(self.wr+self.wl)/2 
            w = self.r*(self.wr-self.wl)/self.L

            vx = v*np.cos(self.theta) 
            vy = v*np.sin(self.theta) 

            self.x = vx*self.dt + self.x 
            self.y = vy*self.dt + self.y

            self.theta = w*self.dt + self.theta 
            self.theta = np.arctan2(np.sin(self.theta),np.cos(self.theta))

            self.et = (np.arctan2(self.y_target-self.y, self.x_target-self.x)) - self.theta
            self.et = np.arctan2(np.sin(self.et),np.cos(self.et)) 

            ed = abs(np.sqrt((self.x_target-self.x)**2 + (self.y_target-self.y)**2))
            
            alpha = 1
            kd_max = 0.2 #kv
            kd = kd_max * ((1.0-np.exp(-alpha*(abs(ed))**2))/(abs(ed)))
            kt = 0.7 #kw

            self.v_n = kd * ed
            self.w_n = kt * self.et

            v = Twist()

            if (ed >= 0 and ed <= 0.30):
                self.current_state = 'Stopped' # 0.30
            else:
                v.linear.x = self.v_n
                v.angular.z = self.w_n
            
            if (self.current_state == 'Stopped'):
                v, self.v_n, self.w_n = Twist(), 0.0, 0.0
            
            self.pos_act.x, self.pos_act.y, self.pos_act.z = self.x, self.y, 0.0
            self.pos_target.x, self.pos_target.y, self.pos_target.z = self.x_target, self.y_target, 0.0

            self.pub_pos_target.publish(self.pos_target)
            self.pub_pos_act.publish(self.pos_act)
            
            self.pub_gtg.publish(v)
            self.pub_ed.publish(ed)
            self.pub_et.publish(self.et)

            print("Target (x, y): ", self.x_target, self.y_target)
            print("v: ", self.v_n) 
            print("w: ", self.w_n)
            print("etheta: ", self.et)
            print("ed: ", ed)

            rate.sleep()  

    def wl_cb(self, wl):  
        self.wl = wl.data 

    def wr_cb(self, wr):   
        self.wr = wr.data  

    def cleanup(self):      
        
        v=Twist()
        self.pub_gtg.publish(v)

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("Control_node", anonymous=True)  
    RobotControl()