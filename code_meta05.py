#!/usr/bin/env python3

######################################################### Librerias################################################3
#librerias reconomiento
from matplotlib import pyplot as plt
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2, Image
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped
from cv_bridge import CvBridge
import os
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#Librerias tuto
import sys
import copy
import rospy
import moveit_commander ##So important to run moveit
import moveit_msgs.msg ##So important to run moveit
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import cv2 
import tf as tf
import tf2_ros as tf2
import rospy
import numpy as np
import ros_numpy
from std_msgs.msg import String
from tmc_msgs.msg import Voice
from geometry_msgs.msg import Twist, WrenchStamped, TransformStamped
from sensor_msgs.msg import Image as ImageMsg, PointCloud2
import tmc_control_msgs.msg
import trajectory_msgs.msg
from moveit_commander import roscpp_initialize
#Librerias para uso de potfields
import numpy as np
from geometry_msgs.msg import PoseStamped
import time
from sensor_msgs.msg import LaserScan
import math
import os
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global estadoDeLaMeta
estadoDeLaMeta='pointreachednot'

class TF_MANAGER():
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation.x = pos[0]
        TS.transform.translation.y = pos[1]
        TS.transform.translation.z = pos[2]
        TS.transform.rotation.x = rot[0]
        TS.transform.rotation.y = rot[1]
        TS.transform.rotation.z = rot[2]
        TS.transform.rotation.w = rot[3]
        return TS

    def pub_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name = '', new_frame = 'map'):
        try:
            traf = self._tfbuff.lookup_transform(new_frame, point_name, rospy.Time(0))
            translation, rotational = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos = translation, rot = rotational, point_name = point_name, ref = new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(ref_frame, target_frame, rospy.Time(0))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False,False]

    def tf2_obj_2_arr(self, transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)
    
        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]

class GRIPPER():
    def __init__(self):
        self._grip_cmd_pub = rospy.Publisher('/hsrb/gripper_controller/command',
                               trajectory_msgs.msg.JointTrajectory, queue_size=100)
        self._grip_cmd_force = rospy.Publisher('/hsrb/gripper_controller/grasp/goal',
        			tmc_control_msgs.msg.GripperApplyEffortActionGoal, queue_size=100)
        			
        self._joint_name = "hand_motor_joint"
        self._position = 0.5
        self._velocity = 0.5
        self._effort = 0.0
        self._duration = 1

    def _manipulate_gripper(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = [self._joint_name]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self._position]
        p.velocities = [self._velocity]
        p.accelerations = []
        p.effort = [self._effort]
        p.time_from_start = rospy.Duration(self._duration)
        traj.points = [p]
        self._grip_cmd_pub.publish(traj)
        
    def _apply_force(self):
        app_force = tmc_control_msgs.msg.GripperApplyEffortActionGoal()
        app_force.goal.effort = -0.5
        self._grip_cmd_force.publish(app_force)
        
    def change_velocity(self, newVel):
        self._velocity = newVel
    
    def open(self):
        self._position = 1.23
        self._effort = 0
        self._manipulate_gripper()

    def steady(self):
        self._position = -0.82
        self._effort = -0.3
        self._manipulate_gripper()
        
    def close(self):
        self._position = -0.82
        self._effort = -0.3
        self._manipulate_gripper()
        self._apply_force()
        rospy.sleep(0.8)
############################################# Metodos para usar potfields ###################################

######### Recepcion de LaserScan
def callback_scan(msg): 

    global obstacle_detected1
    global obstacle_detected2
    global obstacle_detected3
    global rango1#derecha del robot
    global rango2#izquierda del robot
    global rango3#frente del robot
    rango1=msg.ranges[90]
    rango2=msg.ranges[629]
    
    n= int((msg.angle_max-msg.angle_min)/msg.angle_increment/2)
    rango3=msg.ranges[n]
    obstacle_detected1 = rango1<2
    obstacle_detected2 = rango2<2
    obstacle_detected3 = rango3<2
    return

        
############################### Funciones de movimiento   ##############################
#Gira hacia una orientacion deseada
def girar(msg_cmd_vel,tf_man,future_position):

    rospy.sleep(0.5) #tiempo de espera
    trans,rot=tf_man.getTF(target_frame='base_link', ref_frame='map')

    orientation_list=[rot[0], rot[1], rot[2], rot[3]]
    roll, pitch, yaw= euler_from_quaternion(orientation_list)
    actual_position= math.degrees(yaw)
    
    #Valor de orientacion predeterminado
    orientation=1

    #convertimos a 360 la posicion actual
    if actual_position<0:
        actual_position= 360+actual_position 
    print(actual_position)
    #Direccion del movimiento
    if(future_position-actual_position<0): 
        orientation=-1
    print ('>>> Orientacion: ', orientation)
    #tiempo para llegar a la posicion deseada] 
    timeToPosition= abs(future_position-actual_position)
    print(">> Tiempo de giro ", timeToPosition)

    inicio_giro=rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec()-inicio_giro<(timeToPosition/17)) and not rospy.is_shutdown(): #20.943 completo, 1/4 5.235
        msg_cmd_vel.angular.z=orientation*0.3
        pub_cmd_vel.publish(msg_cmd_vel)
    return

def girar_completo(msg_cmd_vel):

    inicio_giro=rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec()-inicio_giro<20.943) and not rospy.is_shutdown(): #20.943 completo, 1/4 5.235
        msg_cmd_vel.angular.z=-0.3
        pub_cmd_vel.publish(msg_cmd_vel)
        
    return

# Metodo para enviar mensaje PoseStamped
def objetivo(goal, ox, oy):
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = ox
    goal.pose.position.y = oy
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 1
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 0
    rospy.sleep(1)
    goal_publisher.publish(goal)

def stateOfPath(data): 
    global estadoDeLaMeta
    estadoDeLaMeta=data.data

def gaze_point(x,y,z):
      
    head_pose = head.get_current_joint_values()
    head_pose[0]=0.0
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    head.go()
    
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #
    
    e =tf.transformations.euler_from_quaternion(rot)

    x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]

    D_x=x_rob-x
    D_y=y_rob-y
    D_z=z_rob-z

    D_th= np.arctan2(D_y,D_x)
    print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

    pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

    if(pan_correct > np.pi):
        pan_correct=-2*np.pi+pan_correct
    if(pan_correct < -np.pi):
        pan_correct=2*np.pi+pan_correct

    if ((pan_correct) > .5 * np.pi):
        print ('Exorcist alert')
        pan_correct=.5*np.pi
    head_pose[0]=pan_correct
    tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

    head_pose [1]=-tilt_correct
    
    
    
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ

def correct_points(low_plane=.0,high_plane=0.2):

    #Corrects point clouds "perspective" i.e. Reference frame head is changed to reference frame map
    data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = data.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(data, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)

    img= np.copy(corrected['y'])

    img[np.isnan(img)]=2
    #img3 = np.where((img>low)&(img< 0.99*(trans[2])),img,255)
    img3 = np.where((img>0.99*(trans[2])-high_plane)&(img< 0.99*(trans[2])-low_plane),img,255)
    return img3

def cloud_cb(msg):    
    global points_data
    global image_data
    points_data = ros_numpy.numpify(msg)
    image_data = points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
      

############################################# Inicializacion de nodo, publicadores y suscriptores ####################################


rospy.init_node("Tutorial", anonymous=True) #Inicializacion del Nodo maestro
roscpp_initialize(sys.argv)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5) #Publicador de tipo PoseStamped
goal = PoseStamped()
rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2, cloud_cb)

################################################### Definimos los grupos y sus nombres ###############################

head = moveit_commander.MoveGroupCommander('head')
whole_body = moveit_commander.MoveGroupCommander('whole_body_light')
arm =  moveit_commander.MoveGroupCommander('arm')


############################################ Publicador de trayectorias ########################################333
#Display trajectory publisher on rviz
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)
####################################### Obtenemos el marco de referencia ############################
arm_planning_frame = arm.get_planning_frame()
head_planning_frame = head.get_planning_frame()
wb_planning_frame = whole_body.get_planning_frame()

################################################# Inicializacion de objetos ##############################
tf_man = TF_MANAGER()
msg_cmd_vel=Twist()
listener = tf.TransformListener()
broadcaster= tf.TransformBroadcaster()
grip = GRIPPER()

################################################## Movement to pickup point 1 ##########################
#Cierra la mano
rospy.sleep(1)
grip.close()

#Lleva el brazo a una posicion que facilita la busqueda de objetos
arm.set_named_target('go') 
arm.go()

#Movimiento al punto de recoleccion 1
#objetivo(goal,0,0)
objetivo(goal,1,0)

#suscripcion al publicador de potfiels para sincronizacion
rospy.Subscriber("goalState", String, stateOfPath)
rospy.sleep(1)

#Espera de llegada al punto de recoleccion
while (not estadoDeLaMeta=='pointreached') and not rospy.is_shutdown():
    rospy.sleep(0.5)
print('ya termine')
rospy.sleep(1)
estadoDeLaMeta='pointreachednot'

#Orientacion del robot
desired_orientation= 135
girar (msg_cmd_vel,tf_man, desired_orientation)

#movimiento de cabeza
gaze_point (0.1798,1.08909, 0.0394)

####################################### Reconocimiento punto 1 #############################

#Asiganamos la imagen a los objetos points e image_data
rospy.sleep(5)
points= points_data #Obtenemos una nube de puntos, similar a la nube de puntos corregida
image= image_data  #Obtenemos la imagen


#Cambiamos la imagen a BRG
im_bgr=cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
plt.imshow(im_bgr)
plt.show()

#Cambiamos la imagen a HSV
im_hsv= cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)
plt.imshow(im_hsv)
plt.show()

#Se realiza utiliza determinado canal de HSV y se acotan los valores
h_min=1
h_max=255

#Se aplica la mascara
region = (im_hsv > h_min) & (im_hsv < h_max)
idx,idy=np.where(region[:,:,0] )
mask= np.zeros((480,640))
mask[idx,idy]=255
plt.imshow(mask ,cmap='gray')
plt.show()

kernel = np.ones((5, 5), np.uint8)
eroded_mask=cv2.erode(mask,kernel,iterations=2)
#dilated_mask=eroded_mask #Se salto la aplicacion de la dilatacion
dilated_mask=cv2.dilate(eroded_mask,kernel)

plt.imshow(dilated_mask,cmap='gray')
plt.show()

## Se definen los contornos
contours, hierarchy = cv2.findContours(dilated_mask.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
print(len(contours))

for contour in contours:
    M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
    
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    boundRect = cv2.boundingRect(contour)
    image2=cv2.rectangle(im_hsv,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,255,255), 2)
    cv2.circle(image2, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(image2, "centroid_"+str(cX)+','+str(cY)    ,    (cX - 50, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

plt.imshow(cv2.cvtColor( image2, cv2.COLOR_HSV2RGB))
plt.show()

coordenadas=[]

#Centroides y momentos estadisticos de la imagen
for contour in contours:
    xyz=[]
    M = cv2.moments(contour) #MOMENTOS ESADISTICOS DE LA IMAGEN
    
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    boundRect = cv2.boundingRect(contour)

    for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
        for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
            aux=(np.asarray((points['x'][ix,jy],points['y'][ix,jy],points['z'][ix,jy])))
            if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                'reject point'
            else:
                xyz.append(aux)

    xyz=np.asarray(xyz)
    cent=xyz.mean(axis=0)
    coordenadas.append(cent)

################################## Publicacion de las TFS ####################################################
coordenadas=np.array(coordenadas)
print(coordenadas)


#Publicacion y lectura de Tf objeto 1, respecto a la cabeza
tf_man.pub_static_tf(pos=coordenadas[0], rot = (0,0,0,1), point_name='Object1', ref='head_rgbd_sensor_link') 
rospy.sleep(1)
trans,rot = tf_man.getTF(target_frame='Object1', ref_frame='map')


#Publicacion  de tf Objeto 1respecto al mapa 
tf_man.pub_static_tf(pos=trans, rot = rot, point_name='Object1Odom', ref='map') 
rospy.sleep(2)
trans,rot = tf_man.getTF(target_frame='Object1Odom', ref_frame='map')
print('<< Valor de la tf respecto al mapa')
print(trans)
####################################### Movimiento menor punto 1 ######################################################
#Cordenadas menos 15 cm
x,y,z = trans
print('<< Valores de x y y tratados')
print(x)
print(x)
x=(x*((abs(x)-0.40)/abs(x)))
y=(y*((abs(y)-0.40)/abs(y)))
print('>> Value')
print(x)
print(y)
print('>> Type')
print(type(x))
print(type(y))

#Movimiento al punto de recoleccion 1
objetivo(goal,x,y)



#Espera de llegada al punto de recoleccion
while (not estadoDeLaMeta=='pointreached') and not rospy.is_shutdown():
    rospy.sleep(0.5)
print('ya termine')
rospy.sleep(1)
estadoDeLaMeta='pointreachednot'

#Orientacion del robot
desired_orientation= 150
girar (msg_cmd_vel,tf_man, desired_orientation)

######################################################### Movimiento del brazo ###########################3333#######
#Abrir mano
grip.open()

#Posicion neutral
arm.set_named_target('neutral') 
arm.go()

#Publicacion de transformada
rospy.sleep(0.5)
whole_body.set_num_planning_attempts(100)
whole_body.set_planning_time(100)

print('>>Ruta para el brazo')
print(rot)
print(trans)
#Cinematica inversa
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = rot[3]
pose_goal.orientation.x = rot[0]
pose_goal.orientation.y = rot[1]
pose_goal.orientation.z = rot[2]

pose_goal.position.x = trans[0]
pose_goal.position.y = trans[1]
pose_goal.position.z = trans[2]
'''
arm.set_start_state_to_current_state()
arm.set_pose_target(pose_goal)

arm.go(wait=True)
arm.stop()
arm.clear_pose_targets()
'''

whole_body.set_start_state_to_current_state()
whole_body.set_pose_target(pose_goal)

whole_body.go(wait=True)
whole_body.stop()
whole_body.clear_pose_targets()













'''
print ('<<< Posicion actual de la cabeza')
eef_link = arm.get_end_effector_link()
print(f">>>End effector link: {eef_link}")
'''
#joint_goal = arm.get_current_joint_values()
#gaze_point(-0.0328,0.97111,0.039, head)






"""
rospy.sleep(0.5) #tiempo de espera
trans,rot=tf_man.getTF(target_frame='base_link', ref_frame='odom')
orientation_list=[rot[0], rot[1], rot[2], rot[3]]
roll, pitch, yaw= euler_from_quaternion(orientation_list)
msg_cmd_vel=Twist()
desired_orientation= 135
girar(msg_cmd_vel,math.degrees(yaw),desired_orientation) #Giro (Twist, AnguloActual, OrientacionDeseada)
"""

'''
########################################## cinematica inversa (damos una coordenada) #########################
#Posicion neutral
arm.set_named_target('neutral') 
arm.go()

#Publicacion de transformada
tf_man = TF_MANAGER()
rospy.sleep(0.5)
_, rot = tf_man.getTF(target_frame='hand_palm_link', ref_frame='odom')
rospy.sleep(0.5)
tf_man.pub_static_tf(pos=[0.4,0.1,0.8], rot = rot, point_name='goal', ref='odom')
rospy.sleep(0.5)
trans, rot = tf_man.getTF(target_frame='goal', ref_frame='odom')
print(trans)

#Cinematica inversa
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = rot[3]
pose_goal.orientation.x = rot[0]
pose_goal.orientation.y = rot[1]
pose_goal.orientation.z = rot[2]

pose_goal.position.x = trans[0]
pose_goal.position.y = trans[1]
pose_goal.position.z = trans[2]

whole_body.set_start_state_to_current_state()
whole_body.set_pose_target(pose_goal)

whole_body.go(wait=True)
whole_body.stop()
whole_body.clear_pose_targets()

'''
