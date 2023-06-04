#!/usr/bin/env python 
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import geometry_msgs.msg
from geometry_msgs.msg import Point
# from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
import tf_conversions
import tf2_ros
import numpy as np

#My marker is 712 (letter J)
class TransformFiducial(): 
    def __init__(self): 
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_cb)
        self.Fiducial = FiducialTransformArray()
        self.coords_pub = rospy.Publisher("Aruco_pos_real", Point, queue_size=1)
        self.zetaKF_pub = rospy.Publisher("ZetaKF", Point, queue_size=1)
        self.coords = Point()
        self.Zeta = Point() #Z para KF (Z = [e, theta])
        self.received_fiducial = 0
        rate = rospy.Rate(10)

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0

        #esto se manda al kalmanFilter
        self.X_aruco = 0.0
        self.Y_aruco = 0.0
        self.error = 0.0
        self.theta = 0.0

        self.delta_x = 0.075 #8 cm de la camara al centro del robot

        while not rospy.is_shutdown():
            if self.received_fiducial:
                self.received_fiducial = 0
                self.only_transforms = self.Fiducial.transforms #el mensaje es muy largo y aqui lo reducimos a solo las transformadas que tiene como tipo una lista
                #aqui se reduce el mensaje a solo el ID
                if len(self.only_transforms) != 0:
                    if self.only_transforms[0].fiducial_id == 117: #if fiducial_id = 117
                        print ("Estas en el primer aruco (%s)\n" % self.only_transforms[0].fiducial_id)
                        
                        self.msg_y_calculos()

                        # self.print_data()
                        #TRANSFORMADA EN RVIZ:
                        # self.handle_turtle_pose(self.only_transforms[0].transform.translation, self.only_transforms[0].transform.rotation, "marker_" + str(self.only_transforms[0].fiducial_id))
                    
                    elif self.only_transforms[0].fiducial_id == 217:
                        print ("Estas en el segundo aruco (%s)\n" % self.only_transforms[0].fiducial_id)
                        self.msg_y_calculos()
                    
                    elif self.only_transforms[0].fiducial_id == 317:
                        print ("Estas en el tercer aruco (%s)\n" % self.only_transforms[0].fiducial_id)
                        self.msg_y_calculos()
                    
                    elif self.only_transforms[0].fiducial_id == 417:
                        print ("Estas en el cuarto aruco (%s)\n" % self.only_transforms[0].fiducial_id)
                        self.msg_y_calculos()

                    elif self.only_transforms[0].fiducial_id == 517:
                        print ("Estas en el ultimo aruco (%s)\n" % self.only_transforms[0].fiducial_id)
                        self.msg_y_calculos()
                    
                    else:
                        print("no es correcto")

                    self.coords.x, self.coords.y, self.coords.z = self.X_aruco, self.Y_aruco, 0
                    self.Zeta.x, self.Zeta.y, self.Zeta.z = self.error, self.theta, 0 
                    self.coords_pub.publish(self.coords) # X Y Z = 0
                    self.zetaKF_pub.publish(self.Zeta) # Z para KF (Z = [e, theta])
                else:
                    print("no hay nada")

            rate.sleep()

    def msg_y_calculos(self):
        # Translation X, Y, Z (wrt camera)
        self.pos_x = self.only_transforms[0].transform.translation.x
        self.pos_y = self.only_transforms[0].transform.translation.y
        self.pos_z = self.only_transforms[0].transform.translation.z
        # Rotation in euler (RPY) (wrt camera)
        self.rot_x, self.rot_y, self.rot_z = self.transformed_fiducial(self.only_transforms[0].transform.rotation)

        #coordinates wrt Robot
        self.X_aruco = self.pos_z + self.delta_x
        self.Y_aruco = - self.pos_x
        print "Aurco coords wrt Robot's center: ", self.X_aruco, self.Y_aruco
        self.error = np.sqrt((self.X_aruco*self.X_aruco)+(self.Y_aruco*self.Y_aruco))
        self.theta = np.arctan2(self.Y_aruco, self.X_aruco)
        self.ZKF=[self.error, self.theta]
        print "error y theta: ", self.ZKF

    def print_data(self):
        print "\nArUco Marker ID: ", self.only_transforms[0].fiducial_id
        print "Position: \n", self.only_transforms[0].transform.translation
        # print("Rotation: ", self.only_transforms[0].transform.rotation) #en quaternion
        print "Transformed Rotation: \n", self.transformed_fiducial(self.only_transforms[0].transform.rotation) #en euler

    def handle_turtle_pose(self, transl, rot, turtlename):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera"
        t.child_frame_id = turtlename
        t.transform.translation.x = transl.x
        t.transform.translation.y = transl.y
        t.transform.translation.z = transl.z
        q = rot
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        br.sendTransform(t)
    
    def transformed_fiducial(self, fiducial_rotation):
        # global roll, pitch, yaw
        orientation_list = [fiducial_rotation.x, fiducial_rotation.y, fiducial_rotation.z, fiducial_rotation.w]
        (R, P, Y) = euler_from_quaternion(orientation_list)
        return R, P, Y

    def fiducial_cb(self, msg):
        self.Fiducial = msg
        self.received_fiducial = 1

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    # first thing, init a node!
    rospy.init_node('camera_aruco_final') 
    TransformFiducial()
