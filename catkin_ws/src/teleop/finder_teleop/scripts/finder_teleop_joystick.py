#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import Joy

#--------------------La funcion angs nos permite devolver un mensaje con el valor de posicion que tiene el flipper seleccionado--------------------
def angs(target_linear_vel):
    return "currently:\tvalue Flippers %s\t " % (target_linear_vel)

#--------------------La funcion velocidad nos permite devolcer el valor lineal y angular que tiene la base--------------------
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

#--------------------La funcion makeSimpleProfile nos permite mantentener un control entre los avances y limites permitidos de velocidades o posiciones ya sea de los flippers o la base-------------------------
def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

#---------------------La funcion constrain evita que evadamos el valor maximo o minimo configurado para los valores de velocidad y posicion------------------------
def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la velocidad, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros --------------------
def checkLinearLimitVelocity(vel):
    
    vel = constrain(vel, -FINDER_MAX_LIN_VEL, FINDER_MAX_LIN_VEL)
    

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros --------------------
def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -FINDER_MAX_ANG_VEL, FINDER_MAX_ANG_VEL)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper(vel):
    vel = constrain(vel, -FLIPPER_MAX_VALUE, FLIPPER_MAX_VALUE)

    return vel
#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper5(vel):
    vel = constrain(vel, 0, SHOULDER_MAX_VALUE)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------    
def checkAngularLimitFlipper6(vel):
    vel = constrain(vel, ELBOW_MIN_VALUE, 0)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------    
def checkAngularLimitFlipper7(vel):
    vel = constrain(vel, -ROLL_ROTATION_MAX_VALUE, ROLL_ROTATION_MAX_VALUE)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper8(vel):
    vel = constrain(vel, -PITCH_ROTATION_MAX_VALUE, PITCH_ROTATION_MAX_VALUE)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper9(vel):
    vel = constrain(vel, -ROLL_ROTATION2_MAX_VALUE, ROLL_ROTATION2_MAX_VALUE)

    return vel

#--------------------La funcion checkLinearVelocity nos permite pasar el valor actual de la posicion angulasr en los flippers dado en radianes, el valor minimo permitido y el maximo permitido para hacer la comprobacion, y devolverlo dentro de los parametros, estas funciones possen valores maximos y minimos distintos a los demas --------------------
def checkAngularLimitFlipper10(vel):
    vel = constrain(vel, 0.4, GRIPPER_ROTATION_MAX_VALUE)

    return vel

def joystick_callback(data):
    global target_angular_flipper10
    global target_angular_flipper9
    global target_angular_flipper8
    global target_angular_flipper7
    global target_angular_flipper6  
    global target_angular_flipper5
    global target_angular_flipper4
    global target_angular_flipper3
    global target_angular_flipper2
    global target_angular_flipper1
    global target_angular_flipper0
    global target_angular_vel
    global target_linear_vel
    global control_angular_vel
    global control_linear_vel
    global control_angular_flipper10
    global control_angular_flipper9
    global control_angular_flipper8
    global control_angular_flipper7
    global control_angular_flipper6
    global control_angular_flipper5
    global control_angular_flipper4
    global control_angular_flipper3
    global control_angular_flipper2
    global control_angular_flipper1
    global control_angular_flipper0
    global pub, pub1, pub2, pub3, pub4, pub5, pub6, pub7, pub8, pub9, pub10, pub11, msg, FLIPPER_MAX_VALUE, FINDER_MAX_ANG_VEL, FINDER_MAX_LIN_VEL,ANG_FLIPPER_STEP_SIZE,SHOULDER_MAX_VALUE, ELBOW_MIN_VALUE, PITCH_ROTATION_MAX_VALUE, GRIPPER_ROTATION_MAX_VALUE ,target_angular_flipper10, target_angular_flipper9, target_angular_flipper8, target_angular_flipper7, target_angular_flipper6, target_angular_flipper5, target_angular_flipper4, target_angular_flipper3, target_angular_flipper2, target_angular_flipper1, target_angular_flipper0, target_linear_vel, target_angular_vel, control_angular_flipper10, control_angular_flipper9, control_angular_flipper8, control_angular_flipper7, control_angular_flipper6, control_angular_flipper5, control_angular_flipper4, control_angular_flipper3, control_angular_flipper2, control_angular_flipper1, control_angular_flipper0, control_angular_vel, control_linear_vel, LIN_VEL_STEP_SIZE, ANG_VEL_STEP_SIZE, ANG_FLIPPER_STEP_SIZE, pos
    palancas=data.axes
    botones=data.buttons
    #------------------Move base of finder ------------
    if botones[4]==0 and botones[5]==0:

        if palancas[1]>0.1 :
            target_linear_vel = checkLinearLimitVelocity((target_linear_vel+palancas[1])*0.2) #------Palanca izquierda-----
            print(vels(target_linear_vel,target_angular_vel))
        if palancas[1]<-0.1  :
            target_linear_vel = checkLinearLimitVelocity((target_linear_vel+palancas[1])*0.2)
            print(vels(target_linear_vel,target_angular_vel))
            
        if palancas[3]>0.1 :
            target_angular_vel = checkAngularLimitVelocity((target_angular_vel+palancas[3])*0.2) #----- Palanca Derecha-----
            print(vels(target_linear_vel,target_angular_vel))
            
        if palancas[3]<-0.1 :
            target_angular_vel = checkAngularLimitVelocity((target_angular_vel+palancas[3])*0.2)
            print(vels(target_linear_vel,target_angular_vel))
        
        #--------------------Stop all movements --------------------   
        #--------------------Show commands--------------------    
        if botones[6]==1:
            print(vels(target_linear_vel,target_angular_vel))
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0
            print(vels(target_linear_vel, target_angular_vel))
            print(msg)

    if botones[4]==0 and botones[5]==1:

        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
        
    #--------------------Right front flipper--------------------
        if palancas[4]>0.1:
            target_angular_flipper0= checkAngularLimitFlipper(target_angular_flipper0+ANG_FLIPPER_STEP_SIZE)
            #status0 = status0 + 1
            print("Right_front_flipper "+angs(target_angular_flipper0))
            control_angular_flipper0 = makeSimpleProfile(control_angular_flipper0, target_angular_flipper0, (ANG_FLIPPER_STEP_SIZE))
            pos[0]=control_angular_flipper0
            
        if palancas[4]<-0.1:
            target_angular_flipper0= checkAngularLimitFlipper(target_angular_flipper0-ANG_FLIPPER_STEP_SIZE)
            #status0 = status0 + 1
            print("Right_front_flipper "+angs(target_angular_flipper0))
            control_angular_flipper0 = makeSimpleProfile(control_angular_flipper0, target_angular_flipper0, (ANG_FLIPPER_STEP_SIZE))
            pos[0]=control_angular_flipper0

        #--------------------Right back flipper--------------------
        if palancas[1]>0.1 :
            target_angular_flipper2= checkAngularLimitFlipper(target_angular_flipper2+ANG_FLIPPER_STEP_SIZE)
            #status2 = status2 + 1
            print("Right_back_flipper "+angs(target_angular_flipper2))
            control_angular_flipper2 = makeSimpleProfile(control_angular_flipper2, target_angular_flipper2, (ANG_FLIPPER_STEP_SIZE))
            pos[2]=control_angular_flipper2
        if palancas[1]<-0.1:
            target_angular_flipper2= checkAngularLimitFlipper(target_angular_flipper2-ANG_FLIPPER_STEP_SIZE)
            #status2 = status2 + 1
            print("Right_back_flipper "+angs(target_angular_flipper2))
            control_angular_flipper2 = makeSimpleProfile(control_angular_flipper2, target_angular_flipper2, (ANG_FLIPPER_STEP_SIZE))
            pos[2]=control_angular_flipper2

    if botones[4]==1 and botones[5]==0:
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
       
        #--------------------Left front flipper--------------------
        if palancas[4]>0.1 :
            target_angular_flipper1= checkAngularLimitFlipper(target_angular_flipper1+ANG_FLIPPER_STEP_SIZE)
            #status1 = status1 + 1
            print("Left_front_flipper "+angs(target_angular_flipper1))
            control_angular_flipper1 = makeSimpleProfile(control_angular_flipper1, target_angular_flipper1, (ANG_FLIPPER_STEP_SIZE))
            pos[1]=control_angular_flipper1

        if palancas[4]<-0.1 :
            target_angular_flipper1= checkAngularLimitFlipper(target_angular_flipper1-ANG_FLIPPER_STEP_SIZE)
            #status1 = status1 + 1
            print("Left_front_flipper "+angs(target_angular_flipper1))
            control_angular_flipper1 = makeSimpleProfile(control_angular_flipper1, target_angular_flipper1, (ANG_FLIPPER_STEP_SIZE))
            pos[1]=control_angular_flipper1


        #--------------------Left back flipper--------------------
        if palancas[1]>0.1  :
            target_angular_flipper3= checkAngularLimitFlipper(target_angular_flipper3+ANG_FLIPPER_STEP_SIZE)
            #status3 = status3 + 1
            print("Left_back_flipper "+angs(target_angular_flipper3))
            control_angular_flipper3 = makeSimpleProfile(control_angular_flipper3, target_angular_flipper3, (ANG_FLIPPER_STEP_SIZE))
            pos[3]=control_angular_flipper3

        if palancas[1]<-0.1 :
            target_angular_flipper3= checkAngularLimitFlipper(target_angular_flipper3-ANG_FLIPPER_STEP_SIZE)
            #status3 = status3 + 1
            print("Left_back_flipper "+angs(target_angular_flipper3))
            control_angular_flipper3 = makeSimpleProfile(control_angular_flipper3, target_angular_flipper3, (ANG_FLIPPER_STEP_SIZE))
            pos[3]=control_angular_flipper3

	    #-----Auxiliar arm-----
    if palancas[2]<0 and palancas[5]>=0:
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0

        #--------------------Arm base rotation--------------------
        if botones[1]==0 and botones[2]==1:
            target_angular_flipper4= checkAngularLimitFlipper(target_angular_flipper4 + ANG_FLIPPER_STEP_SIZE)
            #status4 = status4 + 1
            print("Arm_base_rotation "+angs(target_angular_flipper4))
            control_angular_flipper4 = makeSimpleProfile(control_angular_flipper4, target_angular_flipper4, (ANG_FLIPPER_STEP_SIZE))
            pos[4]=control_angular_flipper4
            
        if botones[1]==1 and botones[2]==0:
            target_angular_flipper4= checkAngularLimitFlipper(target_angular_flipper4 - ANG_FLIPPER_STEP_SIZE)
            #status4 = status4 + 1
            print("Arm_base_rotation "+angs(target_angular_flipper4))
            control_angular_flipper4 = makeSimpleProfile(control_angular_flipper4, target_angular_flipper4, (ANG_FLIPPER_STEP_SIZE))
            pos[4]=control_angular_flipper4

        #--------------------Arm Shoulder rotation--------------------
        
        if botones[3]==1 and botones[0]==0:
        
            target_angular_flipper5= checkAngularLimitFlipper5(target_angular_flipper5 + ANG_FLIPPER_STEP_SIZE)
            #status5 = status5 + 1
            print("Arm_Shoulder_rotation "+angs(target_angular_flipper5))
            control_angular_flipper5 = makeSimpleProfile(control_angular_flipper5, target_angular_flipper5, (ANG_FLIPPER_STEP_SIZE))
            pos[5]=control_angular_flipper5

        if botones[3]==0 and botones[0]==1:
            target_angular_flipper5= checkAngularLimitFlipper5(target_angular_flipper5 - ANG_FLIPPER_STEP_SIZE)
            #status5 = status5 + 1
            print("Arm_Shoulder_rotation "+angs(target_angular_flipper5))
            control_angular_flipper5 = makeSimpleProfile(control_angular_flipper5, target_angular_flipper5, (ANG_FLIPPER_STEP_SIZE))
            pos[5]=control_angular_flipper5

    if palancas[2]>=0 and palancas[5]<0:
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
        #--------------------Arm Elbow rotation--------------------
        if botones[3]==1 and botones[0]==0:
            target_angular_flipper6= checkAngularLimitFlipper6(target_angular_flipper6 - ANG_FLIPPER_STEP_SIZE)
            #status6 = status6 + 1
            print("Arm_Elbow_rotation "+angs(target_angular_flipper6))
            control_angular_flipper6 = makeSimpleProfile(control_angular_flipper6, target_angular_flipper6, (ANG_FLIPPER_STEP_SIZE))
            pos[6]=control_angular_flipper6
        if botones[3]==0 and botones[0]==1:
            target_angular_flipper6= checkAngularLimitFlipper6(target_angular_flipper6 + ANG_FLIPPER_STEP_SIZE)
            #status6 = status6 + 1
            print("Arm_Elbow_rotation "+angs(target_angular_flipper6))
            control_angular_flipper6 = makeSimpleProfile(control_angular_flipper6, target_angular_flipper6, (ANG_FLIPPER_STEP_SIZE))
            pos[6]=control_angular_flipper6
        #--------------------Roll rotation--------------------
        if botones[2]==1 and botones[1]==0:
            target_angular_flipper7= checkAngularLimitFlipper(target_angular_flipper7 - ANG_FLIPPER_STEP_SIZE)
            #status7 = status7 + 1
            print("Roll_rotation "+angs(target_angular_flipper7))
            control_angular_flipper7 = makeSimpleProfile(control_angular_flipper7, target_angular_flipper7, (ANG_FLIPPER_STEP_SIZE))
            pos[7]=control_angular_flipper7

        if botones[2]==0 and botones[1]==1:
            target_angular_flipper7= checkAngularLimitFlipper(target_angular_flipper7 + ANG_FLIPPER_STEP_SIZE)
            #status7 = status7 + 1
            print("Roll_rotation "+angs(target_angular_flipper7))
            control_angular_flipper7 = makeSimpleProfile(control_angular_flipper7, target_angular_flipper7, (ANG_FLIPPER_STEP_SIZE))
            pos[7]=control_angular_flipper7

        #--------------------Pitch rotation--------------------
        if palancas[1]>0.2:
            target_angular_flipper8= checkAngularLimitFlipper8(target_angular_flipper8 - ANG_FLIPPER_STEP_SIZE)
            #status8 = status8 + 1
            print("Pitch_rotation "+angs(target_angular_flipper8))
            control_angular_flipper8 = makeSimpleProfile(control_angular_flipper8, target_angular_flipper8, (ANG_FLIPPER_STEP_SIZE))
            pos[8]=control_angular_flipper8

        if palancas[1]<-0.2:
            target_angular_flipper8= checkAngularLimitFlipper8(target_angular_flipper8 + ANG_FLIPPER_STEP_SIZE)
            #status8 = status8 + 1
            print("Pitch_rotation "+angs(target_angular_flipper8))
            control_angular_flipper8 = makeSimpleProfile(control_angular_flipper8, target_angular_flipper8, (ANG_FLIPPER_STEP_SIZE))
            pos[8]=control_angular_flipper8

    #--------------------Roll rotation2--------------------

        if palancas[3]>0.2:                
            target_angular_flipper9= checkAngularLimitFlipper(target_angular_flipper9 - ANG_FLIPPER_STEP_SIZE)
            #status9 = status9 + 1
            print("Roll_rotation2 "+angs(target_angular_flipper9))
            control_angular_flipper9 = makeSimpleProfile(control_angular_flipper9, target_angular_flipper9, (ANG_FLIPPER_STEP_SIZE))
            pos[9]=control_angular_flipper9
        if palancas[3]<-0.2: 
            target_angular_flipper9= checkAngularLimitFlipper(target_angular_flipper9 + ANG_FLIPPER_STEP_SIZE)
            #status9 = status9 + 1
            print("Roll_rotation2 "+angs(target_angular_flipper9))
            control_angular_flipper9 = makeSimpleProfile(control_angular_flipper9, target_angular_flipper9, (ANG_FLIPPER_STEP_SIZE))
            pos[9]=control_angular_flipper9

    #--------------------Gripper_rotation--------------------
    if palancas[2]<0 and palancas[5]<0:
    	target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0

    	if palancas[1]>0.4:
	        
	        target_angular_flipper10= checkAngularLimitFlipper10(target_angular_flipper10 - ANG_FLIPPER_STEP_SIZE)
	        #status10 = status10 + 1
	        print("Gripper_rotation "+angs(target_angular_flipper10))
	        control_angular_flipper10 = makeSimpleProfile(control_angular_flipper10, target_angular_flipper10, (ANG_FLIPPER_STEP_SIZE))
	        pos[10]=control_angular_flipper10

    	if palancas[1]<-0.4:
	        
	        target_angular_flipper10= checkAngularLimitFlipper10(target_angular_flipper10 + ANG_FLIPPER_STEP_SIZE)
	        #status10 = status10 + 1
	        print("Gripper_rotation "+angs(target_angular_flipper10))
	        control_angular_flipper10 = makeSimpleProfile(control_angular_flipper10, target_angular_flipper10, (ANG_FLIPPER_STEP_SIZE))
	        pos[10]=control_angular_flipper10





    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
    pub.publish(twist)     
    pub1.publish(pos[0])
    pub2.publish(pos[1])
    pub3.publish(pos[2])
    pub4.publish(pos[3])
    pub5.publish(pos[4])
    pub6.publish(pos[5])
    pub7.publish(pos[6])
    pub8.publish(pos[7])
    pub9.publish(pos[8])
    pub10.publish(pos[9])
    pub11.publish(pos[10])

        
       

    #--------------------Manejo de excepciones--------------------


    #--------------------Se indica que el robot deje de moverse hasta que reciba otro comando de velocidad--------------------
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        



#--------------------Mensaje donde se describen os comandos de movimiento--------------------
#--------------------Variables para definir el tope de comandos o valores--------------------


msg = """
Control Your FINDER and Your Flippers!
---------------------------

Moving around:   

    LS AND RS
       

Right_front_flipper:   
    
    RB + RS

Right_back_flipper:   
    
    LB + RS

Left_front_flipper:   
    
    RB + LS

Left_back_flipper:   
    
    LB + LS
    

Arm_base_rotation:   
       
 Press LT + X ---> Turn Left

 Press LT + B ---> Turn Right

Arm_Shoulder_rotation:   
    
 Press LT + A  ---> Up

 Press LT + Y  ---> Down    

Arm_Elbow_rotation:   
     
 Press RT + Y  ---> Up

 Press RT + A  ---> Down

Roll rotation:   
     
 Press RT + X   ---> Turn Left

 Press RT + B   ---> Turn Right

Pitch_rotation:   
    
 Press RT + LS UP  ---> Up

 Press RT + LS DOWN --> Down  

Roll rotation2:   
     
 Press RT + RS LEFT----> Turn Left 

 Press RT + LS RIGHT ----> Turn Right 

Gripper rotation:   
        
    RT + LT + LS UP ---> Open

    RT + LT + LS DOWN  ---> Close    


Back button : force stop

CTRL-C to quit
"""
FINDER_MAX_LIN_VEL = 0.2
FINDER_MAX_ANG_VEL = 0.2
FLIPPER_MAX_VALUE= 3.14
SHOULDER_MAX_VALUE=0.9
ELBOW_MIN_VALUE=-2.3
ROLL_ROTATION_MAX_VALUE= 3.14
PITCH_ROTATION_MAX_VALUE=1.5
ROLL_ROTATION2_MAX_VALUE = 3.14
GRIPPER_ROTATION_MAX_VALUE=1.62
#--------------------Varaibles para definir los pasos--------------------
LIN_VEL_STEP_SIZE=0.1
ANG_VEL_STEP_SIZE=0.2
ANG_FLIPPER_STEP_SIZE=0.01
target_linear_vel= 0.0
target_angular_vel= 0.0
target_angular_flipper0=0.0
target_angular_flipper1 =0.0
target_angular_flipper2=0.0
target_angular_flipper3=0.0
target_angular_flipper4=0.02
target_angular_flipper5=0.01
target_angular_flipper6=0.01
target_angular_flipper7=0.01
target_angular_flipper8=0.01
target_angular_flipper9=0.01
target_angular_flipper10=0.01
#--------------------Variables de desplazamiento de la base--------------------

control_linear_vel  = 0.0
control_angular_vel = 0.0
#--------------------Variables para controlar las juntas--------------------
control_angular_flipper0 = 0.0
control_angular_flipper1 = 0.0
control_angular_flipper2 = 0.0
control_angular_flipper3 = 0.0
control_angular_flipper4 = 0.0
control_angular_flipper5= 0.0
control_angular_flipper6= 0.0
control_angular_flipper7= 0.0
control_angular_flipper8= 0.0
control_angular_flipper9= 1.6
control_angular_flipper10= 1.6
#--------------------Variables para guardar las velocidades de las juntas--------------------
""""""
pos=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.6,1.6]

rospy.init_node('finder_teleop_joystick')

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pub1=rospy.Publisher('/flipper1_out', Float64, queue_size=1)
pub2=rospy.Publisher('/flipper2_out', Float64, queue_size=1)
pub3=rospy.Publisher('/flipper3_out', Float64, queue_size=1)
pub4=rospy.Publisher('/flipper4_out', Float64, queue_size=1)
pub5=rospy.Publisher('/base_rotation_out',Float32,queue_size=1)
pub6=rospy.Publisher('/shoulder_rotation_out',Float32,queue_size=1)
pub7=rospy.Publisher('/elbow_rotation_out',Float32,queue_size=1)
pub8=rospy.Publisher('/roll_rotation_out',Float32,queue_size=1)
pub9=rospy.Publisher('/pitch_rotation_out',Float32,queue_size=1)
pub10=rospy.Publisher('/roll_rotation_2_out',Float32,queue_size=1)
pub11=rospy.Publisher('/gripper_rotation_out',Float32,queue_size=1)

#--------------------Variable que controla la alerta del mensaje--------------------
"""status=0
status0=0
status1=0
status2=0
status3=0
status4=0
status5=0
status6=0
status7=0
status8=0
status9=0
status10=0"""



#--------------------Aqui es donde se declaran todas las operaciones al presionar una de las teclas dentro del menu --------------------

print(msg)
rospy.Subscriber("/joy",Joy,joystick_callback)
rospy.spin()

    
#--------------------Se asignan los valores lineales y angulares de movimiento de la base--------------------    



