#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, Twist


class ROSBridgeNode:
    def __init__(self):
        rospy.init_node('ros_bridge_node')

        # Dicionário para armazenar os dados mais recentes dos sensores
        self.data = {
            'pressure': None,
            'temperature': None,
            'battery': None,
            'imu': None,
            'gps': None,
            'local_position': None
        }

        # Subscribers para os tópicos de sensores do MAVROS/Pixhawk
        rospy.Subscriber('/mavros/imu/atm_pressure', Float32, self.pressure_cb)
        rospy.Subscriber('/mavros/imu/temperature_imu', Float32, self.temperature_cb)
        rospy.Subscriber('/mavros/battery', Float32, self.battery_cb)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        
        # Publisher que envia o JSON consolidado para a GUI via rosbridge
        self.gui_pub = rospy.Publisher('/rov/gui_data', String, queue_size=10)

        # Subscriber que recebe comandos da GUI (em formato JSON)
        rospy.Subscriber('/rov/joystick_cmd', String, self.joystick_cmd_cb)

        # RECOMENDAÇÃO: Publisher para enviar os comandos processados para o MAVROS
        self.cmd_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)

        self.rate = rospy.Rate(10)  # Publica os dados a 10 Hz

    # --- Callbacks para atualizar o dicionário de dados ---
    
    def pressure_cb(self, msg):
        self.data['pressure'] = msg.data

    def temperature_cb(self, msg):
        self.data['temperature'] = msg.data

    def battery_cb(self, msg):
        self.data['battery'] = msg.data

    def imu_cb(self, msg):
        self.data['imu'] = {
            'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y, 'z': msg.orientation.z, 'w': msg.orientation.w},
            'angular_velocity': {'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y, 'z': msg.angular_velocity.z},
            'linear_acceleration': {'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y, 'z': msg.linear_acceleration.z}
        }

    def gps_cb(self, msg):
        self.data['gps'] = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': msg.status.status
        }
    
    def local_position_cb(self, msg):
        self.data['local_position'] = {
            'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z},
            'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}
        }

    # --- Callback para processar comandos da GUI ---

    def joystick_cmd_cb(self, msg):
        """ Analisa o comando JSON da GUI e publica uma mensagem Twist. """
        try:
            cmd_data = json.loads(msg.data)
            
            twist_msg = Twist()
            twist_msg.linear.x = cmd_data.get('linear_x', 0.0)
            twist_msg.linear.y = cmd_data.get('linear_y', 0.0)
            twist_msg.angular.z = cmd_data.get('angular_z', 0.0)
            
            self.cmd_vel_pub.publish(twist_msg)
            rospy.loginfo(f"Comando de velocidade publicado: linear_x={twist_msg.linear.x}, angular_z={twist_msg.angular.z}")

        except json.JSONDecodeError:
            rospy.logwarn(f"JSON malformado recebido em /rov/joystick_cmd: {msg.data}")
        except Exception as e:
            rospy.logerror(f"Erro ao processar comando do joystick: {e}")

    # --- Loop Principal ---
    
    def run(self):
        """ Loop principal que publica os dados consolidados para a GUI. """
        rospy.loginfo("Node ROS Bridge iniciado.")
        while not rospy.is_shutdown():
            # Converte o dicionário de dados em uma string JSON
            json_data = json.dumps(self.data)
            # Publica os dados
            self.gui_pub.publish(json_data)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ROSBridgeNode()
        node.run()
    except rospy.ROSInterruptException:
        pass