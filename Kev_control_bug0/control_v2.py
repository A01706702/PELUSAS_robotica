#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
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
        self.odom = Odometry()
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad]
        
        self.Pv, self.Pw = 0.0, 0.0
        self.Iv, self.Iw = 0.0, 0.0
        self.Dv, self.Dw = 0.0, 0.0
        self.Dv_ant, self.Dw_ant = 0.0, 0.0
        self.Dv_diff, self.Dw_diff = 0.0, 0.0

        self.x_target = 2 #x position of the goal 
        self.y_target = 9 #y position of the goal 

        self.goal_received = 0 #flag to indicate if the goal has been received 

        self.wr = 0.0
        self.wl = 0.0
        self.wr_n = 0.0
        self.wl_n = 0.0

        #DeadReckoning
        self.miu_1 = 0.0
        self.sigma_1 = 0.0
        self.H_1 = 0.0

        #DataDeadReckoning
        #Datos
        self.miu_ant = [0.0,0.0,0.0] #Anterior
        sigma_ant = np.zeros([3,3]) #Anterior
        self.Q = np.zeros([3,3])

        self.current_state = 'Moving Robot' #Robot's current state 
        
        ###******* INIT PUBLISHERS *******###  
        
        self.pub_gtg = rospy.Publisher('gtg', Twist, queue_size=1)
        self.pub_ed = rospy.Publisher('ed', Float32, queue_size=1)
        self.pub_et = rospy.Publisher('et', Float32, queue_size=1)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1) 
        
        self.pub_pos_target = rospy.Publisher('pos_target', Point, queue_size=1)
        self.pub_pos_act = rospy.Publisher('pos_act', Point, queue_size=1)

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)
        #rospy.Subscriber("Aruco_pos_real", Point, self.aruco_cb)

        #********** INIT NODE **********###
        freq = 50 
        rate = rospy.Rate(freq)  
        last_time = rospy.get_time()
        self.dt = 1/float(freq) 

        ################ MAIN LOOP ################  
        while not rospy.is_shutdown():

            # current_time = rospy.get_time()
            # self.dtDR = current_time - last_time
            
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
            
            # alpha = 1
            # kd_max = 0.2 #kv
            # kd = kd_max * ((1.0-np.exp(-alpha*(abs(ed))**2))/(abs(ed)))
            # kt = 0.7 #kw

            P, I = 0.05, 0.0000005
            P2, I2 = 0.3, 0.000001
            
            # P
            self.Pv = P * ed
            self.Pw = P2 * self.et

            # I
            self.Iv = self.Iv + I * ed
            self.Iw = self.Iw + I2 * self.et


            # P + I + D
            c_kv = (self.Pv+self.Iv)
            c_kw = (self.Pw+self.Iw)

            self.v_n = c_kv
            self.w_n = c_kw

            self.pos_act.x, self.pos_act.y, self.pos_act.z = self.x, self.y, 0.0
            self.pos_target.x, self.pos_target.y, self.pos_target.z = self.x_target, self.y_target, 0.0

            DR = self.deadReckoning(self.dt, v, w, sigma_ant)
            self.odom = self.fill_odom(self.x, self.y, self.theta, DR, v, w)

            v = Twist()
            if (ed >= 0 and ed <= 0.15):
                self.current_state = 'Stopped' # 0.30
            else:
                v.linear.x = self.v_n
                v.angular.z = self.w_n
            
            if (self.current_state == 'Stopped'):
                v, self.v_n, self.w_n = Twist(), 0.0, 0.0

            self.odom_pub.publish(self.odom)
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

#            last_time = current_time
            rate.sleep()

    def deadReckoning(self, dt, v, w, sigma_ant):
                        
        miu_act = [self.miu_ant[0]+(dt*v*np.cos(self.miu_ant[2])),
        self.miu_ant[1]+(dt*v*np.sin(self.miu_ant[2])),
        self.miu_ant[2]+(dt*w)]

        #Step 2
        H_1 = np.array([[1, 0.0, -(dt*v*np.sin(self.miu_ant[2]))],
                        [0, 1, dt*v*np.cos(self.miu_ant[2])],
                        [0, 0, 1]], dtype = float)

        #Step 3
        aux1 = np.dot(H_1, sigma_ant)
        self.sigma_1 = np.dot(aux1, np.transpose(H_1))+self.Q

        # Nondeterministic error matrix
        kr = 0.01
        kl = 0.01

        if dt != 0:

            E_s = [[kr*abs(self.wr),0],
                    [0,kl*abs(self.wl)]]

            E_w = (1/(2*self.r*dt))*np.array([[np.cos(self.miu_ant[2]),np.cos(self.miu_ant[2])],
                                [np.sin(self.miu_ant[2]),np.sin(self.miu_ant[2])],
                                [2/self.L,-2/self.L]])

            self.Q = E_w.dot(E_s).dot(np.transpose(E_w))

        # Calculate Covariance Matrix Sigma
        Sigma_pose = np.zeros([3,3]) #Creates the Covariance matrix (3x3) for x, y and theta
        Sigma_pose[0,0] = self.sigma_1[0][0] #cov(xx)
        Sigma_pose[0,1] = self.sigma_1[0][1]  #cov(xy)
        Sigma_pose[0,2] = self.sigma_1[0][2]  #cov(x, theta)
        Sigma_pose[1,0] = self.sigma_1[1][0]  #cov(y,x)
        Sigma_pose[1,1] = self.sigma_1[1][1]  #cov(y,y)
        Sigma_pose[1,2] = self.sigma_1[1][2]  #cov(y,theta)
        Sigma_pose[2,0] = self.sigma_1[2][0]  #cov(theta,x)
        Sigma_pose[2,1] = self.sigma_1[2][1]  #cov(theta,y)
        Sigma_pose[2,2] = self.sigma_1[2][2] #cov(thetat,theta)

        #last_time =  current_time
        self.miu_ant = miu_act

        return Sigma_pose

    def fill_odom(self,x, y, theta, Sigma_pose, v, w):
        # (x,y) -> robot position
        # theta -> robot orientation
        # Sigma_pose -> 3x3 pose covariance matrix
        odom = Odometry()
        odom.header.stamp =rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        quat=quaternion_from_euler(0.0, 0.0, theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.pose.covariance = [0.0]*36
        # Fill the covariance matrix
        odom.pose.covariance[0] = Sigma_pose[0,0]
        odom.pose.covariance[1] = Sigma_pose[0,1]
        odom.pose.covariance[5] = Sigma_pose[0,2]
        odom.pose.covariance[6] = Sigma_pose[1,0]
        odom.pose.covariance[7] = Sigma_pose[1,1]
        odom.pose.covariance[11] = Sigma_pose[1,2]
        odom.pose.covariance[30] = Sigma_pose[2,0]
        odom.pose.covariance[31] = Sigma_pose[2,1]
        odom.pose.covariance[35] = Sigma_pose[2,2]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        return odom

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