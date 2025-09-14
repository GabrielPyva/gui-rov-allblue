#!/usr/bin/env python3

import rospy
import random
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3

def sensor_simulator():
    # Inicializa o nó do simulador
    rospy.init_node('sensor_simulator', anonymous=True)

    # Cria os publishers para cada tópico que o rosbridge_node.py escuta
    pressure_pub = rospy.Publisher('/mavros/imu/atm_pressure', Float32, queue_size=10)
    temp_pub = rospy.Publisher('/mavros/imu/temperature_imu', Float32, queue_size=10)
    battery_pub = rospy.Publisher('/mavros/battery', Float32, queue_size=10)
    imu_pub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
    gps_pub = rospy.Publisher('/mavros/global_position/global', NavSatFix, queue_size=10)
    local_pos_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=10)

    # Define a frequência de publicação (10 vezes por segundo)
    rate = rospy.Rate(10)

    rospy.loginfo("Nó de simulação de sensores iniciado. Publicando dados...")

    # Loop principal
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # 1. Publica dados simples (Float32)
        pressure_pub.publish(Float32(data=random.uniform(1010.0, 1015.0)))
        temp_pub.publish(Float32(data=random.uniform(20.0, 25.0)))
        battery_pub.publish(Float32(data=random.uniform(85.0, 99.0)))

        # 2. Publica dados da IMU
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu_link"
        # Simula uma rotação suave
        roll, pitch, yaw = 0.1, -0.2, math.sin(rospy.get_time() * 0.5)
        # (Simplificado, não é uma conversão real para quaternion)
        imu_msg.orientation = Quaternion(x=roll, y=pitch, z=yaw, w=1.0)
        imu_msg.angular_velocity = Vector3(x=0.01, y=-0.02, z=0.05)
        imu_msg.linear_acceleration = Vector3(x=0.1, y=0.2, z=9.8)
        imu_pub.publish(imu_msg)

        # 3. Publica dados do GPS
        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time
        gps_msg.latitude = -15.7797 + random.uniform(-0.0001, 0.0001)
        gps_msg.longitude = -47.9297 + random.uniform(-0.0001, 0.0001)
        gps_msg.altitude = 1172.0 + random.uniform(-0.5, 0.5)
        gps_msg.status.status = gps_msg.status.STATUS_FIX
        gps_pub.publish(gps_msg)

        # 4. Publica dados de Posição Local
        local_pos_msg = PoseStamped()
        local_pos_msg.header.stamp = current_time
        local_pos_msg.pose.position.x = 10.0 + math.cos(rospy.get_time() * 0.2)
        local_pos_msg.pose.position.y = 5.0 + math.sin(rospy.get_time() * 0.2)
        local_pos_msg.pose.position.z = -20.0
        local_pos_msg.pose.orientation.w = 1.0
        local_pos_pub.publish(local_pos_msg)

        # Espera o tempo necessário para manter a frequência de 10Hz
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_simulator()
    except rospy.ROSInterruptException:
        pass