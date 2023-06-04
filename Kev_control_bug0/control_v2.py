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
        self.y_target = 8 #y position of the goal 

        self.goal_received = 0 #flag to indicate if the goal has been received 

        self.wr, self.wl = 0.0, 0.0
        self.wr_n, self.wl_n = 0.0, 0.0

        self.errorKF, self.thetaKF = 0.0, 0.0

        ########################## Filtro de Kalman ##########################
        # self.M = {
        #     "117": [0.716661, -2.015600],
        #     "217": [-2.015600, -2.779200],
        #     "317": [4.960400, 0.496495],
        #     "417": [6.432060, -5.704600],
        #     "517": [8.999960, -1.664390]
        # }

        self.miu_ant = [0.0,0.0,0.0] # Anterior
        sigma_ant = np.zeros([3,3]) # Anterior
        self.Q = np.zeros([3,3]) # Ruido sensores
        self.Rk = np.array([[0.1,0],    # Ruido ambiente
                            [0, 0.02]])

        self.Miu_hat = np.array([0.0, 0.0, 0.0]) #Sk contains Sx, Sy and Stheta but starts in zeroes.
        self.Sigma_hat = 0.0
        self.Hk = 0.0

        self.Sxk_ant, self.Syk_ant, self.Stheta_ant = 0.0, 0.0, 0.0
        self.E_w = [] 
        self.E_s = []
        
        self.dx, self.dy, self.p = 0.0, 0.0, 0.0
        self.mx, self.my = 0.0, 0.0
        self.ar_x, self.ar_y = 0.0, 0.0

        self.I = np.identity(3)
        self.Gk = np.array([[0, 0, 0], [0, 0, 0]])
        self.Mk = np.zeros([3,3])
        self.SigmaK = np.zeros([3,3])

        self.Zk = []
        self.e_Zhat = 0.0
        self.t_Zhat = 0.0

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
        rospy.Subscriber("Aruco_pos_real", Point, self.aruco_cb)
        rospy.Subscriber("ZetaKF", Point, self.zetaKF_cb)


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

            P, I = 0.03, 0.0000008
            P2, I2 = 0.5, 0.000001
            
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
        
        self.get_noise()

        #Step 3
        aux1 = np.dot(H_1, sigma_ant)
        self.sigma_1 = np.dot(aux1, np.transpose(H_1))+self.Q

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
    
    def get_noise(self):
        # Nondeterministic error matrix
        self.kr = 0.02
        self.kl = 0.02

        self.E_s = [[self.kr * abs(self.wr), 0],
                   [0, self.kl * abs(self.wl)]]

        self.E_w = (1 / (2 * self.r * self.dt)) * np.array([[np.cos(self.Stheta_ant), np.cos(self.Stheta_ant)],
                                                            [np.sin(self.Stheta_ant), np.sin(self.Stheta_ant)],
                                                            [2 / self.L, -2 / self.L]])

        self.Q = self.E_w.dot(self.E_s).dot(np.transpose(self.E_w))

        return self.Q
    
    def predict_pose(self, v, w): 

        self.Sxk_ant = self.sk[0]
        self.Syk_ant = self.sk[1]
        self.Stheta_ant = self.sk[2]
        
        self.motion_model( v, w)

        self.Hk = np.array([[1, 0, -self.dt * self.vk * np.sin(self.Stheta_ant)],
                            [0, 1, self.dt * self.vk * np.cos(self.Stheta_ant)],
                            [0, 0, 1]])
        
        self.get_noise()

        self.P = np.dot(np.dot(self.Hk, self.SigmaK), self.Hk.T) + self.Q #Double dot multiplication of the 3x3 matrixes. Finally we add the noise covariance matrix

        self.dx = self.mx - self.Sxk_act
        self.dy = self.my - self.Syk_act
        self.pg = ((self.dx)**2) + ((self.dy)**2)

        self.Zeta_KF_vini = [self.Z11[self.i][0], self.Z11[self.i][1]]
        self.Zeta_KF_vini = np.transpose(self.Zeta_KF_vini) #Z normal
        
        self.error_Zik = np.sqrt(((self.dx)**2) + ((self.dy)**2)) #z hat
        self.theta_Zik = (np.arctan2(self.dy, self.dx))-self.Stheta_act
        self.Zik = [self.error_Zik, self.theta_Zik]
        self.Zik = np.transpose(self.Zik)

        self.Gk = [[-(self.dx) / np.sqrt(self.pg), -(self.dy) / np.sqrt(self.pg), 0],
                    [(self.dy) / self.pg, -(self.dx) / self.pg, -1]]

        self.Gk = np.array(self.Gk)
        self.P = np.array(self.P)
        self.Zk = self.Gk.dot(self.P).dot(np.transpose(self.Gk))+self.Rk

        self.Kk = self.P.dot(np.transpose(self.Gk)).dot(np.linalg.inv(self.Zk))

        self.Mk = [self.Sxk_act, self.Syk_act, self.Stheta_act] + (self.Kk.dot((self.Zeta_KF_vini - self.Zik)))

        self.SigmaK = (self.I - self.Kk.dot(self.Gk)).dot(self.P)

        #Update next iteration
        self.sk[0] = self.Mk[0]
        self.sk[1] = self.Mk[1]
        self.sk[2] = self.Mk[2]

        return self.Mk, self.SigmaK
    
    def motion_model(self, v, w):
        self.Sxk_act = self.Sxk_ant + self.dt * v * np.cos(self.Stheta_ant) 
        self.Syk_act = self.Syk_ant + self.dt * v * np.sin(self.Stheta_ant) 
        self.Stheta_act = self.Stheta_ant + self.dt * w

        return self.Sxk_act, self.Syk_act, self.Stheta_act

    def aruco_cb(self, coordis):
        self.ar_x = coordis.x
        self.ar_y = coordis.y

    def zetaKF_cb(self, zeta):
        self.errorKF = zeta.x
        self.thetaKF = zeta.y

    def motion_model(self, vk, wk):
        Sxk_act = self.Sxk_ant + self.dt * vk * np.cos(self.Stheta_ant) 
        Syk_act = self.Syk_ant + self.dt * vk * np.cos(self.Stheta_ant) 
        Stheta_act = self.Stheta_ant + self.dt * wk

        self.Sxk_ant = Sxk_act
        self.Syk_ant = Syk_act
        self.Stheta_ant = Stheta_act

        return self.Sxk_ant, self.Syk_ant, self.Stheta_ant

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