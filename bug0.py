#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 

#este codigo equivale al bug 0 de manchester
# se corre con el otro nodo del lidar (transform lidar data)

class Robot(): 
    #This class implements the differential drive model of the robot 
    def __init__(self): 
        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 
        self.d=0
        ############ Variables ############### 
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad]
     
    def update_state(self, wr, wl, delta_t): 
        #This function returns the robot's state 
        #This functions receives the wheel speeds wr and wl in [rad/sec]  
        # and returns the robot's state 
        v=self.r*(wr+wl)/2 
        w=self.r*(wr-wl)/self.L 
        self.theta=self.theta + w*delta_t 
        #Crop theta_r from -pi to pi 
        self.theta=np.arctan2(np.sin(self.theta),np.cos(self.theta)) 
        vx=v*np.cos(self.theta) 
        vy=v*np.sin(self.theta) 
        self.x=self.x+vx*delta_t  
        self.y=self.y+vy*delta_t 
 
#This class will make the puzzlebot move to a given goal 
class GoToGoal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 

        self.robot=Robot() #create an object of the Robot class 
        ############ Variables ############### 
        self.posi_x = 0.0
        self.posi_y = 0.0
        self.x_target = 3.0 #x position of the goal 
        self.y_target = 5.0 #y position of the goal 
        self.goal_received = 0 #flag to indicate if the goal has been received 
        self.lidar_received = False #flag to indicate if the laser scan has been received 
        self.target_position_tolerance = 0.15 #target position tolerance [m] 
        fw_distance = 0.32 # distance  [m] 
        self.v_msg = Twist() #Robot's desired speed
        self.goal_msg = Twist()
        self.wr = 0 #right wheel speed [rad/s] 
        self.wl = 0 #left wheel speed [rad/s]
         
        self.current_state = 'GoToGoal' #Robot's current state 
        
        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
 
        #********** INIT NODE **********###  
        freq = 50 
        rate = rospy.Rate(freq) #freq Hz  
        self.dt = 1.0/float(freq) #Dt is the time between one calculation and the next one 
        print("Node initialized") 
        print("Please send a Goal from rviz using the button: 2D Nav Goal") 
        print("You can also publish the goal to the (move_base_simple/goal) topic.") 

        ################ MAIN LOOP ################  
        while not rospy.is_shutdown():
            self.robot.update_state(self.wr, self.wl, self.dt) #update the robot's state IMPORTANT!! CALL IT every loop  
            if self.lidar_received:
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle
                self.closest_ang = closest_angle
                
                ####### STATE MACHINE #######
                
                if self.current_state == 'GoToGoal':
                    if self.at_goal():
                        self.current_state = 'Stop'
                    elif self.NearToObstacle(fw_distance, closest_range):
                        self.current_state = 'FollowWalls'
                    else: 
                        
                        v_gtg, w_gtg, th_gtg, err_d = self.compute_gtg_control(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta)
                        self.v_msg.linear.x = v_gtg 
                        self.v_msg.angular.z = w_gtg
                        self.warning_dist = err_d

                elif self.current_state == 'FollowWalls':

                    if self.at_goal():  
                        self.current_state = 'Stop'
                    elif self.ObstacleCleared(fw_distance, closest_range):
                        self.current_state = 'GoToGoal'
                    else:
                        #esta linea no lo mueve solo es para el error
                        f_var,f_var, f_var, err_d = self.compute_gtg_control(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta)
                        v_fw, w_fw = self.compute_follow_walls(self.closest_ang) 
                        self.v_msg.linear.x = v_fw 
                        self.v_msg.angular.z = w_fw

                elif self.current_state == 'Stop': 
                    print("Stop! We're here!")
                    print ('coord X: ', self.robot.x)
                    print ('coord Y: ', self.robot.y) 
                    self.v_msg.linear.x = 0 
                    self.v_msg.angular.z = 0

                print(self.current_state)
                print("Distance to objetive: ", err_d)
                print("Closest Angle: ", closest_angle)
                print("Closest Range: ", closest_range)
                print(" ")
            
            self.pub_cmd_vel.publish(self.v_msg)  
            self.pub_cmd_vel.publish(self.v_msg)  
            rate.sleep()  
     
    def at_goal(self): 
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        return np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2)<self.target_position_tolerance
    
    def ObstacleCleared(self, fw_dist, closest_rang):
        #This function returns true if 
        return closest_rang > fw_dist
    
    def NearToObstacle(self, fw_dist, closest_rang):
        #This function returns true if the robot is too close to an obstacle
        return closest_rang < fw_dist

    
    def get_closest_object(self, lidar_msg): 
        #This function returns the closest object to the robot 
        #This functions receives a ROS LaserScan message and returns the distance and direction to the closest object 
        #returns  closest_range [m], closest_angle [rad], 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        # limit the angle to [-pi, pi] 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 
     
    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the linear and angular speed to reach a given goal 
        #This functions receives the goal's position (x_target, y_target) [m] 
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s] 

        ################ YOUR CODE HERE ###########################
        kd = 0.05 #kv
        kt = 0.6 #kw
              
        et = (np.arctan2(self.y_target-self.robot.y, self.x_target-self.robot.x)) - self.robot.theta
        ed = abs(np.sqrt((self.x_target-self.robot.x)**2 + (self.y_target-self.robot.y)**2))
        v = kd * ed
        w = kt * et
        self.theta_gtg = et
        ################ END OF YOUR CODE ########################### 
        return v, w, self.theta_gtg, ed
    
    def compute_follow_walls(self, closest_angle):
        theta = closest_angle
        theta = np.arctan2(np.sin(theta),np.cos(theta))

        theta_ao = theta - np.pi
        theta_ao = np.arctan2(np.sin(theta_ao),np.cos(theta_ao))     # theta_ao

#        the_fwc = -np.pi/2 + theta_ao #clockwise
        the_fwcc = np.pi/2 + theta_ao #counter clockwise
        kw = 1.5  # Angular speed gain
        
        #limit to -pi and pi
#        theta_fwc = np.arctan2(np.sin(the_fwc),np.cos(the_fwc))
        theta_fwcc = np.arctan2(np.sin(the_fwcc),np.cos(the_fwcc))
        
        v_fw = 0.08

        w_fwcc = kw*theta_fwcc
        return v_fw, w_fwcc
        
#         if ((theta_fwc - self.theta_gtg) <= np.pi/2):
#             w_fwc = kw*theta_fwc
# #            print (theta_fwc)
#             return v_fw, w_fwc
#         else:
#             w_fwcc = kw*theta_fwcc
# #            print (theta_fwcc)
#             return v_fw, w_fwcc
 
    def get_angle(self, idx, angle_min, angle_increment):  
        ## This function returns the angle for a given element of the object in the lidar's frame  
        angle = angle_min + idx * angle_increment  
        # Limit the angle to [-pi,pi]  
        angle = np.arctan2(np.sin(angle),np.cos(angle))  
        return angle  
     
    def polar_to_cartesian(self, r, theta):  
        ## This function converts polar coordinates to cartesian coordinates  
        x = r * np.cos(theta)  
        y = r * np.sin(theta)  
        return (x, y)  
     
    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.lidar_received = True  
 
    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 
         
    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  
     
    def goal_cb(self, goal):  
        ## This function receives a the goal from rviz.  
        print("Goal received I'm moving") 
        self.current_state = "GoToGoal" 
        # reset the robot's state 
        self.robot.x = 0 
        self.robot.y = 0  
        self.robot.theta = 0 
        # assign the goal position 
        self.x_target = goal.pose.position.x 
        self.y_target = goal.pose.position.y 
        self.goal_received = 1 
        print("Goal x: ", self.x_target) 
        print("Goal y: ", self.y_target) 
        
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 
 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("go_to_goal_with_obstacles", anonymous=True)  
    GoToGoal()  
