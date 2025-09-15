# Adicione a importação do nosso novo widget no topo do arquivo
from widgets import BatteryWidget, TemperatureWidget, DepthWidget, CompassWidget, RollIndicatorWidget

import sys
import json
import roslibpy
import os
import cv2 # OpenCV
import numpy as np
import urllib.request # Para abrir a URL do vídeo
import math

# --- NOVOS IMPORTS E AJUSTES ---
from PySide6.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QStackedLayout
from PySide6.QtCore import QThread, Signal, Slot, QObject, Qt, QPointF
from PySide6.QtGui import QFont, QPalette, QColor, QImage, QPixmap

# --- O ROSClientWorker permanece o mesmo ---
class ROSClientWorker(QObject):
    dados_recebidos = Signal(str)
    # ... (o código desta classe é idêntico ao anterior, não precisa mudar)
    def __init__(self, host='localhost', port=9090):
        super().__init__()
        self.host = host
        self.port = port
        self.client = None
        self.is_connected = False
    @Slot()
    def run(self):
        try:
            self.client = roslibpy.Ros(host=self.host, port=self.port)
            self.client.run()
            self.is_connected = True
            print(f"Conectado ao rosbridge em {self.host}:{self.port}")
            listener = roslibpy.Topic(self.client, '/rov/gui_data', 'std_msgs/String')
            listener.subscribe(self.callback_gui_data)
        except Exception as e:
            print(f"Erro ao conectar ou se inscrever no tópico: {e}")
    def callback_gui_data(self, message):
        json_data = message['data']
        self.dados_recebidos.emit(json_data)
    def stop(self):
        if self.client and self.is_connected:
            self.client.terminate()
            self.is_connected = False
            print("Desconectado do rosbridge.")


# --- NOVO WORKER DEDICADO PARA O VÍDEO COM OPENCV ---
class VideoWorker(QObject):
    frame_pronto = Signal(QPixmap)
    
    def __init__(self, url):
        super().__init__()
        self.url = url
        self.is_running = True

    @Slot()
    def run(self):
        # Abre o stream de vídeo com OpenCV
        cap = cv2.VideoCapture(self.url)
        if not cap.isOpened():
            print(f"ERRO: Não foi possível abrir o stream de vídeo em {self.url}")
            return

        print("Stream de vídeo iniciado com sucesso.")
        while self.is_running:
            ret, frame = cap.read() # Lê um frame
            if ret:
                # Converte a imagem BGR (padrão do OpenCV) para RGB
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                # Cria um QImage a partir dos dados do frame
                qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                # Cria um QPixmap a partir do QImage
                pixmap = QPixmap.fromImage(qt_image)
                # Emite o sinal com o frame pronto
                self.frame_pronto.emit(pixmap)
        
        cap.release()
        print("Stream de vídeo parado.")

    def stop(self):
        self.is_running = False

def quaternion_to_yaw_degrees(w, x, y, z):
    """Converte um quatérnion para um ângulo de guinada (yaw) em graus."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)

# --- NOVA FUNÇÃO DE CONVERSÃO PARA ROLL ---
def quaternion_to_roll_degrees(w, x, y, z):
    """Converte um quatérnion para um ângulo de rolagem (roll) em graus."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)
    return math.degrees(roll_rad)

# --- Janela Principal da GUI (Atualizada) ---
class ROV_GUI(QWidget):
    def __init__(self):
        super().__init__()
        
        self.inicializar_ui()
        self.conectar_ros()
        self.conectar_videos()

    def inicializar_ui(self):
        """
        Configura a UI com vídeo no fundo e dados sobrepostos.
        Esta versão garante que a camada de dados sempre cubra a de vídeo.
        """
        self.setGeometry(100, 100, 1280, 720)
        self.setWindowTitle("Painel de Controle do ROV")
        self.setStyleSheet("background-color: black;")

        # --- Camada de Vídeo Principal ---
        self.video_label = QLabel(self)
        self.video_label.setScaledContents(True)
        
        # --- NOVA SEÇÃO: Adiciona o widget de bateria ---
        self.battery_widget = BatteryWidget(self) # Define a janela principal como "pai"
        self.battery_widget.move(20, 20) # Posiciona no canto superior esquerdo com uma margem

        # --- NOVO: Adiciona o widget de temperatura ---
        self.temp_widget = TemperatureWidget(self)
        # Posiciona 10px à direita do widget de bateria (que tem 100px de largura)
        self.temp_widget.move(20 + 100 + 10, 20)

        # --- NOVO: Adiciona o widget de profundidade ---
        self.depth_widget = DepthWidget(self)
        # Posiciona ao lado do widget de temperatura
        self.depth_widget.move(20 + 100 + 10 + 120 + 10, 20)

        # --- NOVO: Adiciona o widget de bússola ---
        self.compass_widget = CompassWidget(self)
        # A posição será definida dinamicamente no resizeEvent para manter centralizado

        # --- NOVO: Adiciona o indicador de roll ---
        self.roll_indicator = RollIndicatorWidget(self)

        # --- NOVO: Widget do Miniplayer da Câmera Inferior ---
        self.down_camera_label = QLabel(self)
        self.down_camera_label.setFixedSize(320, 180) # Tamanho 16:9
        self.down_camera_label.setScaledContents(True)
        # Adiciona uma borda branca para destacar o miniplayer
        self.down_camera_label.setStyleSheet("border: 2px solid white;")
        
        self.show()
    
    # Adicione este novo método à classe ROV_GUI para lidar com o redimensionamento da janela
    def resizeEvent(self, event):
        """
        Este evento é chamado toda vez que a janela muda de tamanho.
        Garante que tanto o vídeo quanto a camada de dados se ajustem.
        ATUALIZADO: Agora também posiciona o miniplayer no canto inferior direito.
        """
        super().resizeEvent(event)
        # Redimensiona o vídeo de fundo e a camada de dados
        self.video_label.resize(self.size())
        
        # Calcula a nova posição do miniplayer com uma margem de 20px
        margin = 20
        new_x = self.width() - self.down_camera_label.width() - margin
        new_y = self.height() - self.down_camera_label.height() - margin
        self.down_camera_label.move(new_x, new_y)

        # --- NOVO: Posiciona a bússola ---
        new_x_compass = (self.width() - self.compass_widget.width()) / 2
        self.compass_widget.move(int(new_x_compass), margin)

        # --- NOVO: Posiciona o indicador de roll ---
        new_x_roll = (self.width() - self.roll_indicator.width()) / 2
        new_y_roll = (self.height() - self.roll_indicator.height()) / 2
        self.roll_indicator.move(int(new_x_roll), int(new_y_roll))

    def conectar_videos(self):
        ip_wsl = "172.30.55.191" # <-- COLOQUE SEU IP AQUI
        # **IMPORTANTE**: Voltamos para a URL original, sem Theora.
        # --- Conexão da Câmera Principal (Esquerda) ---
        url_stream_main = f"http://{ip_wsl}:8080/stream?topic=/rov/camera/stereo/left/image_raw"
        self.main_video_thread = QThread()
        self.main_video_worker = VideoWorker(url_stream_main)
        self.main_video_worker.moveToThread(self.main_video_thread)
        self.main_video_thread.started.connect(self.main_video_worker.run)
        self.main_video_worker.frame_pronto.connect(self.atualizar_frame_video)
        self.main_video_thread.start()

        # --- NOVA: Conexão da Câmera Inferior ---
        url_stream_down = f"http://{ip_wsl}:8080/stream?topic=/rov/camera/down/image_raw"
        self.down_video_thread = QThread()
        self.down_video_worker = VideoWorker(url_stream_down)
        self.down_video_worker.moveToThread(self.down_video_thread)
        self.down_video_thread.started.connect(self.down_video_worker.run)
        self.down_video_worker.frame_pronto.connect(self.atualizar_frame_down_camera)
        self.down_video_thread.start()

    @Slot(QPixmap)
    def atualizar_frame_video(self, pixmap):
        """Recebe o novo frame e o exibe no QLabel."""
        self.video_label.setPixmap(pixmap)

    @Slot(QPixmap)
    def atualizar_frame_down_camera(self, pixmap):
        """NOVO: Slot para atualizar o miniplayer da câmera inferior."""
        self.down_camera_label.setPixmap(pixmap)

    def conectar_ros(self):
        # Este método permanece o mesmo
        self.ros_thread = QThread()
        ip_wsl = "172.30.55.191" # <-- COLOQUE SEU IP AQUI TAMBÉM
        self.ros_worker = ROSClientWorker(host=ip_wsl, port=9090)
        self.ros_worker.moveToThread(self.ros_thread)
        self.ros_thread.started.connect(self.ros_worker.run)
        self.ros_worker.dados_recebidos.connect(self.atualizar_dados)
        self.ros_thread.start()

    @Slot(str)
    def atualizar_dados(self, json_data):
        # Este método permanece o mesmo
        try:
            dados = json.loads(json_data)
            self.setWindowTitle("Painel de Controle do ROV - Conectado")

            # --- ATUALIZAÇÃO: Pega o dado da bateria ---
            bateria = dados.get('battery', 0)
            if bateria is None: bateria = 0
            self.battery_widget.set_level(bateria)

            # --- NOVA LINHA: Pega e atualiza a temperatura ---
            temperatura = dados.get('temperature', 0)
            self.temp_widget.set_temperature(temperatura)

            # --- NOVA LINHA: Pega a pressão e atualiza o widget de profundidade ---
            pressao = dados.get('pressure', 0)
            self.depth_widget.set_pressure(pressao)

            # --- NOVA SEÇÃO: Atualiza a Bússola ---
            imu_data = dados.get('imu', {})
            orient_data = imu_data.get('orientation', {})
            if orient_data:
                w = orient_data.get('w', 1.0)
                x = orient_data.get('x', 0.0)
                y = orient_data.get('y', 0.0)
                z = orient_data.get('z', 0.0)
                
                # Converte o quatérnion para yaw e atualiza o widget
                yaw_angle = quaternion_to_yaw_degrees(w, x, y, z)
                self.compass_widget.set_yaw_angle(yaw_angle)

                # --- NOVA SEÇÃO: Atualiza o Indicador de Roll ---
                roll_angle = quaternion_to_roll_degrees(w, x, y, z)
                self.roll_indicator.set_roll_angle(roll_angle)

            imu_data = dados.get('imu', {}); accel_data = imu_data.get('linear_acceleration', {}); orient_data = imu_data.get('orientation', {})
            local_pos_data = dados.get('local_position', {}); pos_data = local_pos_data.get('position', {})
        except (json.JSONDecodeError, TypeError) as e:
            print(f"Erro ao processar dados: {e}")

    def closeEvent(self, event):
        """ATUALIZADO: Garante que a thread da segunda câmera também seja fechada."""
        print("Fechando a aplicação...")
        self.main_video_worker.stop()
        self.main_video_thread.quit()
        self.main_video_thread.wait()
        
        self.down_video_worker.stop()
        self.down_video_thread.quit()
        self.down_video_thread.wait()

        self.ros_worker.stop()
        self.ros_thread.quit()
        self.ros_thread.wait()
        
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    janela = ROV_GUI()
    sys.exit(app.exec())