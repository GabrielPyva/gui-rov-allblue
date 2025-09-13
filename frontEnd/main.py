# main.py

import sys
from PySide6.QtWidgets import QApplication, QWidget

# 1. A classe principal da nossa aplicação
class ROV_GUI(QWidget):
    def __init__(self):
        # Chama o construtor da classe pai (QWidget)
        super().__init__()
        
        # Chama um método para configurar a UI
        self.inicializar_ui()

    def inicializar_ui(self):
        """
        Configura os elementos da interface gráfica da janela.
        """
        self.setGeometry(100, 100, 800, 600) # Posição X, Posição Y, Largura, Altura
        self.setWindowTitle("Painel de Controle do ROV") # Define o título da janela
        
        # Mostra a janela na tela
        self.show()

# 2. Ponto de entrada do programa
if __name__ == "__main__":
    # Cria a instância da aplicação. O `sys.argv` são argumentos de linha de comando.
    app = QApplication(sys.argv)
    
    # Cria a instância da nossa janela principal
    janela = ROV_GUI()
    
    # Inicia o loop de eventos da aplicação e garante uma saída limpa
    sys.exit(app.exec())