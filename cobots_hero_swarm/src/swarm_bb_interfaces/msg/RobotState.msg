#Estados posibles del robot
uint8 STATE_UNKNOWN=0
uint8 STATE_IDLE=1 
uint8 STATE_MOVING =2
uint8 STATE_CHARGING =3 
uint8 STATE_ERROR=4

#Instante  sistema de coordenadas de la medición
std_msgs/Header header

#Identificador dentro el enjambre
string robot_id

#Posición planar global: x,y, theta
geometry_msgs/Pose pose

# Velocidad lineal y angular 
geometry_msgs/Twist velocity

#Nivel entre 0.0 y 1.0 
float32 battery_percentage

#Estado Actual usando las constante anteriores
uint8 state

