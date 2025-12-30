import sys
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLineEdit, QLabel, QGroupBox)
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
from PyQt5.QtCore import Qt, QTimer
import time

g = 9.81
scale = 300 # scale canvas

class Axis:
    def __init__(self, c=0, pg=0.0, ig=0.0, dg=0.0, tgt=0.0):
        self.c = float(c)
        self.pg = float(pg)
        self.ig = float(ig)
        self.dg = float(dg)
        self.tgt = float(tgt)
        
        self.e = 0.0
        self.pe = 0.0
        self.i = 0.0
        self.output = 0.0

    def pid(self, value, dt):
        if dt <= 0: return self.output
        self.c = value
        self.e = self.tgt - self.c
        
        pv = self.pg * self.e
        dv = self.dg * (self.e - self.pe) / dt
        
        di = self.e * dt
        tpo = (pv + dv) + self.ig * (self.i + di)
        
        # Anti-windup
        if not ((tpo > 1.0 and self.e > 0) or (tpo < -1.0 and self.e < 0)):
            self.i += di

        iv = max(-0.5, min(0.5, self.ig * self.i))
        self.output = max(-1.0, min(1.0, pv + iv + dv))
        self.pe = self.e
        return self.output

class Object():
    def __init__(self, mass=1, dimension=[50, 80], position=[0, 0]):
        self.dimension = dimension 
        self.velocity = [0, 0] 
        self.position = position # [0, 0] is now the visual center
        self.mass = mass
        self.pid = Axis(self.position[1], 0, 0, 0, 0)

    def draw(self, painter):
        w = self.dimension[0] * scale
        h = self.dimension[1] * scale
        x = int(self.position[0] * scale - w / 2)
        y = int(self.position[1] * scale - h / 2)
        
        painter.drawRect(x, y, int(w), int(h))

class App(QWidget):
    def __init__(self, width, height):
        super().__init__()
        self.win_width = width
        self.win_height = height
        
        self.object = Object(0.973, [0.1, 0.5], [0, 0])
        
        self.initUI()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.compute)
        self.timer.start(16)

    def initUI(self):
        main_layout = QHBoxLayout()
        
        self.canvas = QWidget()
        self.canvas.setMinimumSize(600, 600)
        self.canvas.setStyleSheet("background-color: white; border: 1px solid #ccc;")
        self.canvas.paintEvent = self.paintCanvas
        main_layout.addWidget(self.canvas, stretch=4)

        controls = QVBoxLayout()
        self.pg_input = self.create_input("proportional gain", "-0.588235294", controls)
        self.ig_input = self.create_input("integral gain", "-2.570958453", controls)
        self.dg_input = self.create_input("derivative gain", "-0.033647059", controls)
        self.tgt_input = self.create_input("target value", "0.0", controls)
        self.pg_input.textEdited.connect(self.update_pid_params)
        self.ig_input.textEdited.connect(self.update_pid_params)
        self.dg_input.textEdited.connect(self.update_pid_params)
        self.tgt_input.textEdited.connect(self.update_pid_params)



        self.toggleButton = QPushButton("start")
        self.toggleButton.clicked.connect(self.toggle_simulation)
        controls.addWidget(self.toggleButton)
        self.resetButton = QPushButton("reset")
        self.resetButton.clicked.connect(self.reset)
        controls.addWidget(self.resetButton)
        
        control_group = QGroupBox("PID Parameters")
        control_group.setLayout(controls)
        main_layout.addWidget(control_group, stretch=1)

        self.setLayout(main_layout)
        self.setWindowTitle('pid')
        self.setGeometry(100, 100, self.win_width, self.win_height)
        self.show()

    def update_pid_params(self):
        try:
            self.object.pid.pg = float(self.pg_input.text())
            self.object.pid.ig = float(self.ig_input.text())
            self.object.pid.dg = float(self.dg_input.text())
            self.object.pid.tgt = float(self.tgt_input.text())
        except ValueError:
            pass

    def reset(self):
        self.object.position[1] = 0
        self.object.velocity[1] = 0
        self.pt = None
        self.object.pid.i = 0
        self.canvas.update()

    def toggle_simulation(self):
        if self.toggleButton.text() == "start":
            self.toggleButton.setText("stop")
        elif self.toggleButton.text() == "stop":
            self.toggleButton.setText("start")

    def create_input(self, label_text, default_val, layout):
        layout.addWidget(QLabel(label_text))
        line_edit = QLineEdit()
        line_edit.setText(default_val)
        layout.addWidget(line_edit)
        return line_edit

    def compute(self):
        if self.toggleButton.text() == "stop":
            try:
                t = time.time()
                dt = t - self.pt

                self.object.pid.pid(self.object.position[1], dt)
                print(self.object.pid.output)
                thrust = 1.5049 * (5 + 2.5 * self.object.pid.output)
                self.object.velocity[1] += (g - thrust / self.object.mass) * dt
                self.object.position[1] += self.object.velocity[1]
                
                self.pt = t
                self.canvas.update()
            except:
                self.pt = time.time()


    def paintCanvas(self, event):
        painter = QPainter(self.canvas)
        painter.setRenderHint(QPainter.Antialiasing)
        
        cx = self.canvas.width() // 2
        cy = self.canvas.height() // 2
        painter.translate(cx, cy)
        
        painter.setPen(QPen(QColor(220, 220, 220), 1, Qt.DashLine))
        painter.drawLine(-cx, 0, cx, 0) # Horizontal axis
        painter.drawLine(0, -cy, 0, cy) # Vertical axis

        painter.setPen(QPen(Qt.red, 2))
        ty = int(self.object.pid.tgt * scale)
        painter.drawLine(-30, ty, 30, ty)

        painter.setPen(QPen(Qt.black, 1))
        painter.setBrush(QBrush(QColor(100, 150, 250, 180)))
        self.object.draw(painter)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App(1000, 800)
    sys.exit(app.exec_())