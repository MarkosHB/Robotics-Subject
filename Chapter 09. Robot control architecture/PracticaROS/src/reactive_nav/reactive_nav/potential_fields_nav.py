#####################################
####### Potential Fields Node #######
#### Autor: Marcos Hidalgo Baños ####
#####################################

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


###############################
#### Definiciones globales ####
###############################

KGOAL = 2.0
KOBSTACLE = 12.0
RADIUS_OF_INFLUENCE = 5
MAX_DIST = 1.0

goal_pose = Pose()
true_pose = Pose()
motion_command_msg = Twist()


##############################
#### Funciones auxiliares ####
##############################  
 
def repulsive_force(true_pose, true_angle, angle_inc, angle_min, dist_observ):
    
    # Distinguimos aquellos landmarks que son detectables...
    seen = [obj for obj in dist_observ if obj <= RADIUS_OF_INFLUENCE]
    
    if seen:
        # ... obtenemos sus angulos a partir de las mediciones del laser
        angles = [angle_min + i * angle_inc for i in range(len(seen))]
        landmark = []

        # Iteramos por cada observacion
        for idx in range(len(seen)):

            # Reconstruimos la observacion
            z = (seen[idx] * np.cos(angles[idx]),
                 seen[idx] * np.sin(angles[idx]))
            
            # Reconstruimos el landmark a partir de la pose del robot y el angulo de la observacion
            landmark.append((true_pose.position.x + z[0] * np.cos(true_angle) - z[1] * np.sin(true_angle),
                             true_pose.position.y + z[0] * np.sin(true_angle) + z[1] * np.cos(true_angle)))
            
        # Calculamos la fuerza repulsiva
        FRep = (KOBSTACLE * sum((1/d - 1/RADIUS_OF_INFLUENCE)/(d**2) * lx / d for d, lx in zip(seen, [true_pose.position.x - x[0] for x in landmark])),
                KOBSTACLE * sum((1/d - 1/RADIUS_OF_INFLUENCE)/(d**2) * ly / d for d, ly in zip(seen, [true_pose.position.y - y[1] for y in landmark])))
    else:
        FRep = (0,0) # No hay repulsividad si no hay obstaculos 
        
    return FRep
      
def attractive_force(goal_error, dist_goal):
    FAtt = ((-KGOAL * goal_error[0]) / dist_goal, 
            (-KGOAL * goal_error[1]) / dist_goal)
    
    return FAtt

###############################
#### Funciones de callback ####
###############################

def goal_pose_callback(msg):
    global goal_pose
    goal_pose = msg.pose


def base_pose_callback(msg):
    global true_pose, true_twist
    true_pose = msg.pose.pose
    true_twist = msg.twist.twist


def laser_callback(msg, motion_command_publisher):
    global goal_pose, motion_command_msg, true_pose, true_twist
    dist_goal = np.sqrt((true_pose.position.x - goal_pose.position.x)**2 + (true_pose.position.y - goal_pose.position.y)**2)
    
    # Comprobamos que no hemos llegado al destino...
    if dist_goal > MAX_DIST:
        
        dist_observ = msg.ranges
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        true_angle = np.arctan2(true_twist.linear.y, true_twist.linear.x)

        # Calculamos la fuerza de repulsion...
        FRep = repulsive_force(true_pose, true_angle, angle_inc, angle_min, dist_observ)
    
        # Calculamos la fuerza atractiva...
        goal_error = (true_pose.position.x - goal_pose.position.x, true_pose.position.y - goal_pose.position.y)
        FAtt = attractive_force(goal_error, dist_goal)

        # ... para obtener la total
        FTotal = FAtt + FRep
        
        # Obtenemos la velocidad y angulo
        v = ((FTotal[0] * np.cos(true_angle) + FTotal[1] * np.sin(true_angle)), 
             (-FTotal[0] * np.sin(true_angle) + FTotal[1] * np.cos(true_angle)))
        Theta = np.arctan2(v[1], v[0])

        # Le mandamos al robot los comandos de movimiento

        motion_command_msg.linear.x = np.sqrt(v[0]**2 + v[1]**2)
        motion_command_msg.angular.z = Theta
    
    else:

        # Mandamos un movimiento nulo (quedarse)
        motion_command_msg.linear.x = 0.0
        motion_command_msg.angular.z = 0.0

    motion_command_publisher.publish(motion_command_msg)


#######################
#### Programa Main ####
#######################

def main():

    # Paso 1: Establecer el nuevo nodo
    print("Creando e inicializando nodo...")
    rclpy.init()
    nodo = rclpy.create_node('motion_command_node')

    # Paso 2: Añadir las conexiones a topics necesarias
    print("Estableciendo subscripciones y publicaciones...")
    motion_command_publisher = nodo.create_publisher(Twist, 'cmd_vel', 10)

    nodo.create_subscription(LaserScan, 'laser1', lambda msg: laser_callback(msg, motion_command_publisher), 10)
    nodo.create_subscription(Odometry, 'base_pose_ground_truth', base_pose_callback, 10)
    nodo.create_subscription(PoseStamped, 'goal_pose', goal_pose_callback, 10)

    # Paso 3: Mantener el nodo
    print("Escuchando...")
    rclpy.spin(nodo)

    # Paso 4: Cierre del nodo
    print("Apagando nodo...")
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
