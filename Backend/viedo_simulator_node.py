#!/usr/bin/env python3

import rospy
import cv2 # OpenCV
import numpy as np # Biblioteca para manipulação de arrays (imagens)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # A "ponte" entre OpenCV e ROS

def video_simulator():
    # Inicializa o nó
    rospy.init_node('video_simulator', anonymous=True)

    # Cria os publishers para as três câmeras
    left_pub = rospy.Publisher('/rov/camera/stereo/left/image_raw', Image, queue_size=10)
    right_pub = rospy.Publisher('/rov/camera/stereo/right/image_raw', Image, queue_size=10)
    down_pub = rospy.Publisher('/rov/camera/down/image_raw', Image, queue_size=10)

    # Instancia o objeto CvBridge
    bridge = CvBridge()

    # Define a taxa de quadros (frames por segundo)
    rate = rospy.Rate(30) # 30 FPS

    frame_count = 0
    rospy.loginfo("Nó de simulação de vídeo iniciado. Publicando frames...")

    while not rospy.is_shutdown():
        # --- Cria os frames para cada câmera ---
        # Usamos o contador de frames para criar um movimento simples
        frame_count += 1

        # Frame da Câmera Esquerda
        img_left = np.zeros((480, 640, 3), dtype=np.uint8) # Imagem preta 640x480
        text_left = f"Left Cam - Frame: {frame_count}"
        cv2.putText(img_left, text_left, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # Desenha um retângulo que se move
        rect_x = (frame_count * 5) % 640
        cv2.rectangle(img_left, (rect_x - 20, 200), (rect_x + 20, 240), (0, 255, 0), -1)

        # Frame da Câmera Direita
        img_right = np.zeros((480, 640, 3), dtype=np.uint8)
        text_right = f"Right Cam - Frame: {frame_count}"
        cv2.putText(img_right, text_right, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.circle(img_right, (320, 240), (frame_count % 100) + 10, (0, 0, 255), 2)

        # Frame da Câmera Inferior
        img_down = np.zeros((480, 640, 3), dtype=np.uint8)
        text_down = f"Down Cam - Frame: {frame_count}"
        cv2.putText(img_down, text_down, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.line(img_down, (0,0), (frame_count % 640, frame_count % 480), (255, 0, 0), 3)

        try:
            # --- Converte e Publica os frames ---
            # Usa o cv_bridge para converter a imagem do formato cv2 para a mensagem ROS
            ros_img_left = bridge.cv2_to_imgmsg(img_left, "bgr8")
            ros_img_right = bridge.cv2_to_imgmsg(img_right, "bgr8")
            ros_img_down = bridge.cv2_to_imgmsg(img_down, "bgr8")

            # Publica as mensagens
            left_pub.publish(ros_img_left)
            right_pub.publish(ros_img_right)
            down_pub.publish(ros_img_down)

        except Exception as e:
            rospy.logerr(f"Erro ao converter ou publicar frame: {e}")

        rate.sleep()

if __name__ == '__main__':
    try:
        video_simulator()
    except rospy.ROSInterruptException:
        pass