from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QBrush, QPen, QFont
from PySide6.QtCore import Qt, QRectF

class BatteryWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._battery_level = 0  # Nível inicial da bateria (0 a 100)

        # Configurações de aparência
        self.setFixedSize(100, 45)

    def set_level(self, level):
        """
        Método público para atualizar o nível da bateria.
        """
        # Garante que o nível esteja entre 0 e 100
        self._battery_level = max(0, min(100, level))
        self.update()  # Força o widget a se redesenhar chamando o paintEvent

    def paintEvent(self, event):
        """
        Este método é chamado toda vez que o widget precisa ser desenhado.
        """
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing) # Deixa as bordas mais suaves

        # Define a área de desenho
        rect = QRectF(0, 5, 80, 35) # Corpo principal da bateria
        terminal_rect = QRectF(80, 15, 5, 15) # O "polo" da bateria

        # --- Desenha o corpo externo da bateria ---
        pen = QPen(QColor("white"), 2) # Caneta branca com 2px de espessura
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush) # Sem preenchimento
        painter.drawRoundedRect(rect, 5, 5)
        painter.drawRoundedRect(terminal_rect, 2, 2)

        # --- Desenha o preenchimento interno da bateria ---
        fill_width = rect.width() * (self._battery_level / 100.0)
        fill_rect = QRectF(rect.x() + 3, rect.y() + 3, fill_width - 6, rect.height() - 6)

        # Define a cor do preenchimento com base no nível da bateria
        if self._battery_level <= 20:
            fill_color = QColor("red")
        elif self._battery_level <= 50:
            fill_color = QColor("orange")
        else:
            fill_color = QColor("lightgreen")
        
        painter.setBrush(QBrush(fill_color))
        painter.setPen(Qt.NoPen) # Não queremos borda no preenchimento
        painter.drawRoundedRect(fill_rect, 2, 2)

        # --- Desenha o texto da porcentagem ---
        font = QFont("Arial", 14)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QPen(QColor("white")))
        
        # O texto é desenhado fora do corpo da bateria.
        # Mas para o seu HUD final, você pode desenhar dentro se preferir.
        text = f"{int(self._battery_level)}%"
        # Para desenhar o texto ao lado
        # painter.drawText(QRectF(90, 5, 50, 35), Qt.AlignCenter, text)
        
        # Para desenhar o texto dentro
        painter.drawText(rect, Qt.AlignCenter, text)