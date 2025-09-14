# main.py

import sys
import json
import roslibpy
# Precisamos de mais componentes: QGridLayout para o layout e QObject para o worker
from PySide6.QtWidgets import QApplication, QWidget, QGridLayout, QLabel
from PySide6.QtCore import QThread, Signal, Slot, QObject
from PySide6.QtGui import QFont # Para negrito

# A classe ROSClientWorker permanece a mesma
class ROSClientWorker(QObject):
    dados_recebidos = Signal(str)

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


# --- A Janela Principal da GUI (com as maiores mudanças) ---
class ROV_GUI(QWidget):
    def __init__(self):
        super().__init__()
        # Dicionário para guardar nossos labels de dados para fácil acesso
        self.data_labels = {}
        self.inicializar_ui()
        self.conectar_ros()

    def inicializar_ui(self):
        """
        Configura a interface com um layout de grade para todos os dados.
        """
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle("Painel de Controle do ROV - Conectando...")

        # Usando QGridLayout para organizar os widgets em linhas e colunas
        grid_layout = QGridLayout(self)
        bold_font = QFont()
        bold_font.setBold(True)

        # --- Criação dos Labels ---
        # Lista de campos de dados que vamos exibir
        # Formato: ('chave_no_json', 'Texto do Label', (linha, coluna))
        campos = [
            # Dados Simples
            ('pressure', 'Pressão (hPa):', (0, 0)),
            ('temperature', 'Temperatura (°C):', (1, 0)),
            ('battery', 'Bateria (%):', (2, 0)),
            # Aceleração Linear
            ('accel_x', 'Acel. Linear X:', (4, 0)),
            ('accel_y', 'Acel. Linear Y:', (5, 0)),
            ('accel_z', 'Acel. Linear Z:', (6, 0)),
            # Posição Local
            ('pos_x', 'Posição Local X:', (8, 0)),
            ('pos_y', 'Posição Local Y:', (9, 0)),
            ('pos_z', 'Posição Local Z:', (10, 0)),
            # Orientação (do IMU)
            ('orient_x', 'Orientação X (quat):', (4, 2)),
            ('orient_y', 'Orientação Y (quat):', (5, 2)),
            ('orient_z', 'Orientação Z (quat):', (6, 2)),
            ('orient_w', 'Orientação W (quat):', (7, 2)),
            # GPS
            ('gps_lat', 'Latitude:', (9, 2)),
            ('gps_lon', 'Longitude:', (10, 2)),
            ('gps_alt', 'Altitude:', (11, 2)),
        ]

        for chave, texto, pos in campos:
            # Cria o label estático (o título)
            title_label = QLabel(texto)
            title_label.setFont(bold_font)
            grid_layout.addWidget(title_label, pos[0], pos[1])

            # Cria o label dinâmico (o valor) e o guarda no dicionário
            value_label = QLabel("N/A")
            self.data_labels[chave] = value_label
            grid_layout.addWidget(value_label, pos[0], pos[1] + 1)
        
        # Adiciona espaçamento para não ficar tudo colado no topo
        grid_layout.setRowStretch(len(campos), 1)

        self.show()

    def conectar_ros(self):
        self.ros_thread = QThread()
        # Lembre-se de usar o IP do seu WSL aqui
        self.ros_worker = ROSClientWorker(host='172.30.55.191', port=9090) # <-- Altere o IP aqui
        self.ros_worker.moveToThread(self.ros_thread)
        self.ros_thread.started.connect(self.ros_worker.run)
        self.ros_worker.dados_recebidos.connect(self.atualizar_dados)
        self.ros_thread.start()

    @Slot(str)
    def atualizar_dados(self, json_data):
        """
        Recebe o JSON, extrai todos os dados e atualiza cada label individualmente.
        """
        try:
            dados = json.loads(json_data)
            self.setWindowTitle("Painel de Controle do ROV - Conectado")

            # --- Acessando os dados de forma segura ---
            # O .get('chave', {}) previne erros se o objeto (ex: 'imu') for nulo.
            imu_data = dados.get('imu', {})
            accel_data = imu_data.get('linear_acceleration', {})
            orient_data = imu_data.get('orientation', {})
            
            local_pos_data = dados.get('local_position', {})
            pos_data = local_pos_data.get('position', {})
            
            gps_data = dados.get('gps', {})

            # --- Atualizando os labels ---
            # Usamos o dicionário self.data_labels que criamos na inicialização
            # O f-string ':.2f' formata o número para ter 2 casas decimais.
            self.data_labels['pressure'].setText(f"{dados.get('pressure', 0):.2f}")
            self.data_labels['temperature'].setText(f"{dados.get('temperature', 0):.2f}")
            self.data_labels['battery'].setText(f"{dados.get('battery', 0):.1f}")
            
            self.data_labels['accel_x'].setText(f"{accel_data.get('x', 0):.3f}")
            self.data_labels['accel_y'].setText(f"{accel_data.get('y', 0):.3f}")
            self.data_labels['accel_z'].setText(f"{accel_data.get('z', 0):.3f}")

            self.data_labels['pos_x'].setText(f"{pos_data.get('x', 0):.2f}")
            self.data_labels['pos_y'].setText(f"{pos_data.get('y', 0):.2f}")
            self.data_labels['pos_z'].setText(f"{pos_data.get('z', 0):.2f}")
            
            self.data_labels['orient_x'].setText(f"{orient_data.get('x', 0):.3f}")
            self.data_labels['orient_y'].setText(f"{orient_data.get('y', 0):.3f}")
            self.data_labels['orient_z'].setText(f"{orient_data.get('z', 0):.3f}")
            self.data_labels['orient_w'].setText(f"{orient_data.get('w', 0):.3f}")

            self.data_labels['gps_lat'].setText(f"{gps_data.get('latitude', 0):.6f}")
            self.data_labels['gps_lon'].setText(f"{gps_data.get('longitude', 0):.6f}")
            self.data_labels['gps_alt'].setText(f"{gps_data.get('altitude', 0):.2f}")

        except (json.JSONDecodeError, TypeError) as e:
            print(f"Erro ao processar dados: {e}")

    def closeEvent(self, event):
        print("Fechando a aplicação...")
        self.ros_worker.stop()
        self.ros_thread.quit()
        self.ros_thread.wait()
        event.accept()

# Ponto de entrada do Programa
if __name__ == "__main__":
    app = QApplication(sys.argv)
    janela = ROV_GUI()
    sys.exit(app.exec())