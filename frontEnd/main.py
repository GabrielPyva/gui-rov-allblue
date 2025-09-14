# main.py

import sys
import json
import roslibpy
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
# QObject é a classe correta para workers que não são visuais
from PySide6.QtCore import QThread, Signal, Slot, QObject 

# --- Parte 1: O Worker para a Conexão ROS ---
# CORREÇÃO: A classe agora herda de QObject, não de QWidget.
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
            self.client.run() # Inicia o loop de eventos do cliente em segundo plano
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


# --- Parte 2: A Janela Principal da GUI ---
class ROV_GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.inicializar_ui()
        self.conectar_ros()

    def inicializar_ui(self):
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle("Painel de Controle do ROV - Conectando...")

        self.layout = QVBoxLayout(self)
        self.dados_label = QLabel("Aguardando dados do ROV...")
        self.layout.addWidget(self.dados_label)
        
        self.show()

    def conectar_ros(self):
        self.ros_thread = QThread()
        # **IMPORTANTE:** Lembre-se de alterar 'localhost' para o IP do seu WSL.
        self.ros_worker = ROSClientWorker(host='172.30.55.191', port=9090)
        
        self.ros_worker.moveToThread(self.ros_thread)
        
        self.ros_thread.started.connect(self.ros_worker.run)
        self.ros_worker.dados_recebidos.connect(self.atualizar_dados)
        
        self.ros_thread.start()

    @Slot(str)
    def atualizar_dados(self, json_data):
        try:
            dados = json.loads(json_data)
            
            print("Dados recebidos:", dados) # Você verá isso no terminal do Windows

            temperatura = dados.get('temperature', 'N/A')
            bateria = dados.get('battery', 'N/A')

            self.setWindowTitle("Painel de Controle do ROV - Conectado")
            self.dados_label.setText(f"Temperatura: {temperatura:.2f}°C\nBateria: {bateria:.1f}%")
        
        except (json.JSONDecodeError, TypeError):
            print("Erro ao decodificar JSON ou dados nulos recebidos.")
            self.dados_label.setText("Recebendo dados inválidos ou nulos...")

    def closeEvent(self, event):
        print("Fechando a aplicação...")
        self.ros_worker.stop()
        self.ros_thread.quit()
        self.ros_thread.wait()
        event.accept()

# --- Parte 3: Ponto de Entrada do Programa ---
if __name__ == "__main__":
    app = QApplication(sys.argv)
    janela = ROV_GUI()
    sys.exit(app.exec())