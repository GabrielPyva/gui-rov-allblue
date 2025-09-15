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

class TemperatureWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._temperature = 0.0
        self.setFixedSize(120, 45) # Espaço para o ícone e o texto

    def set_temperature(self, temp):
        """Método público para atualizar a temperatura."""
        self._temperature = temp if temp is not None else 0.0
        self.update() # Força o widget a se redesenhar

    def paintEvent(self, event):
        """Desenha o ícone do termômetro e o valor."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Cores e canetas
        pen = QPen(QColor("white"), 2)
        brush_red = QBrush(QColor("red"))

        # Desenha o bulbo (círculo) do termômetro
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        bulb_rect = QRectF(5, 20, 20, 20)
        painter.drawEllipse(bulb_rect)

        # Desenha o corpo do termômetro
        body_rect = QRectF(12, 5, 6, 20)
        painter.drawRoundedRect(body_rect, 3, 3)
        
        # Preenchimento vermelho (estático, apenas para o visual)
        fill_bulb_rect = QRectF(9, 24, 12, 12)
        painter.setBrush(brush_red)
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(fill_bulb_rect)
        painter.drawRect(QRectF(13, 15, 4, 11))

        # Desenha o texto da temperatura
        font = QFont("Arial", 14)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QPen(QColor("white")))
        
        text = f"{self._temperature:.1f}°C"
        text_rect = QRectF(30, 0, 90, 45)
        painter.drawText(text_rect, Qt.AlignCenter, text)

class DepthWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._depth = 0.0
        
        # Constantes para o cálculo da profundidade
        self.RHO = 1025  # Densidade da água do mar em kg/m^3
        self.G = 9.80665  # Aceleração da gravidade em m/s^2
        self.P_ATM = 101325  # Pressão atmosférica em Pascals (1013.25 hPa)
        
        self.setFixedSize(120, 45)

    def set_pressure(self, pressure_hpa):
        """
        Recebe a pressão absoluta em hectopascais (hPa), calcula e atualiza a profundidade.
        """
        if pressure_hpa is None:
            pressure_hpa = self.P_ATM / 100 # Assume pressão atmosférica se o dado for nulo

        # 1. Converter pressão para Pascals
        pressure_pa = pressure_hpa * 100
        
        # 2. Calcular a pressão manométrica
        gauge_pressure = pressure_pa - self.P_ATM
        
        # 3. Calcular a profundidade (garante que não seja negativa)
        if gauge_pressure > 0:
            self._depth = gauge_pressure / (self.RHO * self.G)
        else:
            self._depth = 0.0
            
        self.update() # Força o redesenho

    def paintEvent(self, event):
        """Desenha o ícone de profundidade e o valor."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Desenha o ícone da seta para baixo
        pen = QPen(QColor("white"), 2)
        painter.setPen(pen)
        
        # Linha principal da seta
        painter.drawLine(15, 10, 15, 35)
        # Ponta da seta
        painter.drawLine(5, 25, 15, 35)
        painter.drawLine(25, 25, 15, 35)

        # Desenha o texto da profundidade
        font = QFont("Arial", 14)
        font.setBold(True)
        painter.setFont(font)
        
        text = f"{self._depth:.1f} m"
        text_rect = QRectF(30, 0, 90, 45)
        painter.drawText(text_rect, Qt.AlignCenter, text)