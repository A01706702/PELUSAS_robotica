#!/usr/bin/env python 
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
import tf_conversions
import tf2_ros
import numpy as np

#My marker is 712 (letter J)
class TransformFiducial(): 
    def __init__(self): 
        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_cb)
        self.Fiducial = FiducialTransformArray()
        self.received_fiducial = 0
        rate = rospy.Rate(10)

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0

        while not rospy.is_shutdown():
            if self.received_fiducial:
                self.received_fiducial = 0
                self.only_transforms = self.Fiducial.transforms #el mensaje es muy largo y aqui lo reducimos a solo las transformadas que tiene como tipo una lista
                #aqui se reduce el mensaje a solo el ID
                if len(self.only_transforms) != 0:
                    if self.only_transforms[0].fiducial_id == 117: #if fiducial_id = 117
                        # AQUI CALCULAR CON LA FORMULA
                        print ("Estas en el primer aruco (%s)" % self.only_transforms[0].fiducial_id)
                        self.pos_x = self.only_transforms[0].transform.translation.x
                        self.pos_y = self.only_transforms[0].transform.translation.y
                        self.pos_z = self.only_transforms[0].transform.translation.z
                        print "\nX: ", self.pos_x
                        print "Y: ", self.pos_y
                        print "Z: ", self.pos_z

                        self.rot_x, self.rot_y, self.rot_z = self.transformed_fiducial(self.only_transforms[0].transform.rotation)
                        print "\nRoll: ", self.rot_x
                        print "Pitch: ", self.rot_y
                        print "Yaw: ", self.rot_z

                        #TODO CALCULAR LA FORMULAAAA ALV EN TODOS ESTOS IFSSS AAAAAAAAAAAAAAAAAAAAAAA

                        # self.print_data()
                        #TRANSFORMADA EN RVIZ:
                        # self.handle_turtle_pose(self.only_transforms[0].transform.translation, self.only_transforms[0].transform.rotation, "marker_" + str(self.only_transforms[0].fiducial_id))
                    
                    elif self.only_transforms[0].fiducial_id == 217:
                        print ("Estas en el segundo aruco (%s)" % self.only_transforms[0].fiducial_id)
                        # print "IMPRIMIR LA COORDENADA CALCULADA CON RESPECTO A LA CAMARA Y EL ARUCO"
                        self.pos_x = self.only_transforms[0].transform.translation.x
                        self.pos_y = self.only_transforms[0].transform.translation.y
                        self.pos_z = self.only_transforms[0].transform.translation.z
                        print "\nX: ", self.pos_x
                        print "Y: ", self.pos_y
                        print "Z: ", self.pos_z

                        self.rot_x, self.rot_y, self.rot_z = self.transformed_fiducial(self.only_transforms[0].transform.rotation)
                        print "\nRoll: ", self.rot_x
                        print "Pitch: ", self.rot_y
                        print "Yaw: ", self.rot_z
                    
                    elif self.only_transforms[0].fiducial_id == 317:
                        print ("Estas en el tercer aruco (%s)" % self.only_transforms[0].fiducial_id)
                        # print "IMPRIMIR LA COORDENADA CALCULADA CON RESPECTO A LA CAMARA Y EL ARUCO"
                        self.pos_x = self.only_transforms[0].transform.translation.x
                        self.pos_y = self.only_transforms[0].transform.translation.y
                        self.pos_z = self.only_transforms[0].transform.translation.z
                        print "\nX: ", self.pos_x
                        print "Y: ", self.pos_y
                        print "Z: ", self.pos_z

                        self.rot_x, self.rot_y, self.rot_z = self.transformed_fiducial(self.only_transforms[0].transform.rotation)
                        print "\nRoll: ", self.rot_x
                        print "Pitch: ", self.rot_y
                        print "Yaw: ", self.rot_z
                    
                    elif self.only_transforms[0].fiducial_id == 417:
                        print ("Estas en el cuarto aruco (%s)" % self.only_transforms[0].fiducial_id)
                        # print "IMPRIMIR LA COORDENADA CALCULADA CON RESPECTO A LA CAMARA Y EL ARUCO"
                        self.pos_x = self.only_transforms[0].transform.translation.x
                        self.pos_y = self.only_transforms[0].transform.translation.y
                        self.pos_z = self.only_transforms[0].transform.translation.z
                        print "\nX: ", self.pos_x
                        print "Y: ", self.pos_y
                        print "Z: ", self.pos_z

                        self.rot_x, self.rot_y, self.rot_z = self.transformed_fiducial(self.only_transforms[0].transform.rotation)
                        print "\nRoll: ", self.rot_x
                        print "Pitch: ", self.rot_y
                        print "Yaw: ", self.rot_z

                    elif self.only_transforms[0].fiducial_id == 517:
                        print ("Estas en el ultimo aruco (%s)" % self.only_transforms[0].fiducial_id)
                        # print "IMPRIMIR LA COORDENADA CALCULADA CON RESPECTO A LA CAMARA Y EL ARUCO"
                        self.pos_x = self.only_transforms[0].transform.translation.x
                        self.pos_y = self.only_transforms[0].transform.translation.y
                        self.pos_z = self.only_transforms[0].transform.translation.z
                        print "\nX: ", self.pos_x
                        print "Y: ", self.pos_y
                        print "Z: ", self.pos_z

                        self.rot_x, self.rot_y, self.rot_z = self.transformed_fiducial(self.only_transforms[0].transform.rotation)
                        print "\nRoll: ", self.rot_x
                        print "Pitch: ", self.rot_y
                        print "Yaw: ", self.rot_z
                    
                    else:
                        print("no es correcto")
                else:
                    print("no hay nada")

            rate.sleep()

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
