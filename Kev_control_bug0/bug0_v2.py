#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 

class Bug0():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 

        ############ Variables ############### 
        self.xi = 0.0
        self.yi = 0.0

        self.lidar_received = False #flag to indicate if the laser scan has been received 
        self.target_position_tolerance = 0.15 #target position tolerance [m] 
        fw_distance = 0.36 # distance  [m]
        w_distance = 0.32
        self.avoid = 0

        self.v_msg = Twist() #Robot's desired speed
        self.ed_msg = Float32()
        self.et_msg = Float32()
        self.gtg_msg = Twist()

        self.x_msg, self.y_msg = 0.0, 0.0
        self.targetx_msg, self.targety_msg = 0.0, 0.0
        self.theta_ant = 0.0
         
        self.current_state = 'GoToGoal' #Robot's current state 
        
        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)

        rospy.Subscriber('gtg', Twist, self.gtg_cb)
        rospy.Subscriber('ed', Float32, self.ed_cb)
        rospy.Subscriber('et', Float32, self.et_cb)

        rospy.Subscriber('pos_act', Point, self.pos_act_cb)
        rospy.Subscriber('pos_target', Point, self.pos_target_cb)
 
        #********** INIT NODE **********###  
        freq = 50 
        rate = rospy.Rate(freq)
        self.dt = 1.0/float(freq)
        print("Node initialized") 
        print("Please send a Goal from rviz using the button: 2D Nav Goal") 
        print("You can also publish the goal to the (move_base_simple/goal) topic.") 

        ################ MAIN LOOP ################  
        while not rospy.is_shutdown():

            if self.lidar_received:

                self.xf = self.targetx_msg
                self.yf = self.targety_msg
                self.x_act = self.x_msg
                self.y_act = self.y_msg

                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle
                self.closest_ang = closest_angle

                v_gtg  = self.gtg_msg.linear.x
                w_gtg  = self.gtg_msg.angular.z
                err_d = self.ed_msg.data

                ####### STATE MACHINE #######
                
                if self.current_state == 'GoToGoal':
                    if self.at_goal():
                            self.current_state = 'Stop'
                    elif self.NearToObstacle(fw_distance, closest_range):
                            self.current_state = 'FollowWalls'
                    else:
                            self.v_msg.linear.x = v_gtg
                            self.v_msg.angular.z = w_gtg
                            print("Going to Goal")
                        
                elif self.current_state == 'FollowWalls':

                    if self.at_goal():  
                        self.current_state = 'Stop'
                    elif self.ObstacleCleared(fw_distance, closest_range):
                        self.current_state = 'GoToGoal'
                    else:

                        v_fw, w_fw = self.compute_follow_walls(self.closest_ang, closest_range, fw_distance, w_distance) 
                        self.v_msg.linear.x = v_fw 
                        self.v_msg.angular.z = w_fw
                        err_d = self.ed_msg.data

                elif self.current_state == 'Stop': 
                    print("Stopping...")
                    print ('Final position X: ', self.x_act)
                    print ('Final position Y: ', self.y_act) 
                    self.v_msg.linear.x = 0 
                    self.v_msg.angular.z = 0

                print("State: ", self.current_state)
                print("Distance to objetive: ", err_d)
                print("Velocidad lineal: ", self.v_msg.linear.x)
                print("Velocidad angular: ", self.v_msg.angular.z)
                print("Closest Angle: ", closest_angle)
                print("Closest Range: ", closest_range)
            
            self.pub_cmd_vel.publish(self.v_msg)
            self.avoid = 0
            rate.sleep()  
     
    def at_goal(self): 
        return np.sqrt((self.xf-self.x_act)**2+(self.yf-self.y_act)**2)<self.target_position_tolerance
    
    def ObstacleCleared(self, fw_dist, closest_rang): 
            return closest_rang > fw_dist

    def NearToObstacle(self, fw_dist, closest_rang):
        if (closest_rang < fw_dist):
            return closest_rang < fw_dist
    
    def get_closest_object(self, lidar_msg): 

        min_idx = np.argmin(lidar_msg.ranges) 

        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 
    
    def compute_follow_walls(self, closest_angle, closest_range, fw_distance, w_distance):
        
        theta = closest_angle
        theta = np.arctan2(np.sin(theta),np.cos(theta))

        theta_ao = theta - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao),np.cos(theta_ao))

        the_fwcc = np.pi/2 + theta_ao
        theta_fwcc = np.arctan2(np.sin(the_fwcc),np.cos(the_fwcc))
        
        kw = 1.5
        v_fwcc = 0.13

        if closest_range<w_distance:
            kw = 1.8 - 0.7*(closest_range/w_distance)
            v_fwcc = 0.13 * (closest_range/w_distance)

        w_fwcc = kw*theta_fwcc
        return v_fwcc, w_fwcc
     
    def laser_cb(self, msg):    
        self.lidar_msg = msg  
        self.lidar_received = True  

    def pos_act_cb(self, msg):
        self.x_msg = msg.x
        self.y_msg = msg.y
        self.x_received = True
        self.y_received = True

    def pos_target_cb(self, msg):
        self.targetx_msg = msg.x
        self.targety_msg = msg.y
        self.targetx_received = True
        self.targety_received = True

    def gtg_cb(self, msg):
        self.gtg_msg = msg
        self.control_received = True
        
    def ed_cb(self, msg):
        self.ed_msg = msg
        self.ed_received = True
        
    def et_cb(self, msg):
        self.et_msg = msg
        self.et_received = True
        
    def cleanup(self):    
        v_msg = Twist() 
        self.pub_cmd_vel.publish(v_msg) 
 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("Bug0_node", anonymous=True)  
    Bug0()  