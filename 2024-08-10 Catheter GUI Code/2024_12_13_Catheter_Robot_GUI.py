import sys
import time
import csv
import serial
from serial.tools import list_ports
from math import *
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QScrollArea, QSlider, QGroupBox, 
                             QRadioButton, QComboBox)
from PyQt5.QtCore import Qt

def find_available_ports():
    connected_ports = []
    for port, desc, hwid in list_ports.comports():
        try:
            with serial.Serial(port, timeout=1) as ser:
                if ser.readable():
                    connected_ports.append(port)
        except serial.SerialException as e:
            pass
    return connected_ports

class MainWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        self.setWindowTitle('Catheter Robot Control Interface')
        # --- 原先你写了 self.setMinimumSize(1920, 1080) ---
        # 如果想让窗口可随意缩放，就注释或删除：
        # self.setMinimumSize(800, 600)  # 也可只指定一个较小值
        
        self.overall_fontsize = 16
        self.label_style = {"font-size": "16px"}
        self.title_style = {"color": "black", "font-size": "20px"}

        # 其余初始化略 ...
        self.connected_ports = find_available_ports()
        self.Connection_Flag = False
        self.Data_Received_Flag = 0
        self.Control_Mode_Flag = 0

        # --- Layout Setup ---
        MainLayout = QVBoxLayout(self)
        self.setLayout(MainLayout)

        # 1) 通讯区
        self.Comm_Layout = QHBoxLayout()
        MainLayout.addLayout(self.Comm_Layout)

        # 2) Control Mode区
        self.CtrlMode_Block = QGroupBox("Control Mode")
        MainLayout.addWidget(self.CtrlMode_Block)

        # 3) 下面加一个 ScrollArea 来放 “Catheter v1” 这个界面
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        MainLayout.addWidget(self.scroll, stretch=1)

        # 然后再创建 CatheterV1_Block
        self.Create_CtrlMode_Block()
        self.Create_CatheterV1_Block()

        # 把 self.CatheterV1_Block 作为 scroll 的子 widget
        self.scroll.setWidget(self.CatheterV1_Block)
        # 默认隐藏
        self.CatheterV1_Block.setVisible(False)

        # 其余  Create_DMC_Instrument_1_Block / ... 可省略或另行添加
        
        # --- Setup Communication area ---
        self.Create_DevicesCombobox()
        self.Create_ConnectButton()
        self.Create_LoggingButton()

        # 在 self.Comm_Layout 里加小部件
        self.Device_Label = QLabel("Device:")
        self.Device_Label.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Comm_Layout.addWidget(self.Device_Label)
        self.Comm_Layout.addWidget(self.DevicesCombobox)
        self.Comm_Layout.addWidget(self.ConnectButton)
        self.Comm_Layout.addWidget(self.LoggingButton)

        # 如果有 Timer:
        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.all)
        self.timer.start()

    def Create_CtrlMode_Block(self):
        self.CtrlMode_Block_Layout = QHBoxLayout()
        self.CtrlMode_Block.setLayout(self.CtrlMode_Block_Layout)

        self.CtrlJoint_Rbutton = QRadioButton("Direct Motor Control")
        self.CtrlTask_Rbutton  = QRadioButton("Task Space Control")
        self.CtrlCathV1_Rbutton= QRadioButton("Catheter v1 Task Control")

        self.CtrlMode_Block_Layout.addWidget(self.CtrlJoint_Rbutton)
        self.CtrlMode_Block_Layout.addWidget(self.CtrlTask_Rbutton)
        self.CtrlMode_Block_Layout.addWidget(self.CtrlCathV1_Rbutton)

        self.CtrlJoint_Rbutton.setEnabled(False)
        self.CtrlTask_Rbutton.setEnabled(False)
        self.CtrlCathV1_Rbutton.setEnabled(False)

        # 绑定槽函数
        self.CtrlJoint_Rbutton.toggled.connect(self.Direct_Motor_Ctrl_mode)
        self.CtrlTask_Rbutton.toggled.connect(self.Task_Space_Ctrl_mode)
        self.CtrlCathV1_Rbutton.toggled.connect(self.CatheterV1_Task_Ctrl_mode)

    def Create_CatheterV1_Block(self):
        self.CatheterV1_Block = QGroupBox("Catheter v1 Task Control Interface")
        self.CatheterV1_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        layout = QVBoxLayout()
        self.CatheterV1_Block.setLayout(layout)

        lbl_title = QLabel("Catheter v1 Control - Joystick + Speed Slider")
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        layout.addWidget(lbl_title)

        # ---- Joystick (上下左右+中间) ----
        self.joystick_widget = QWidget()
        self.joystick_layout = QGridLayout(self.joystick_widget)

        self.joystick_up_btn = QPushButton("↑")
        self.joystick_up_btn.setFixedSize(60,60)
        self.joystick_up_btn.pressed.connect(lambda: self.on_joystick_pressed("up"))
        self.joystick_up_btn.released.connect(self.on_joystick_released)

        self.joystick_down_btn = QPushButton("↓")
        self.joystick_down_btn.setFixedSize(60,60)
        self.joystick_down_btn.pressed.connect(lambda: self.on_joystick_pressed("down"))
        self.joystick_down_btn.released.connect(self.on_joystick_released)

        self.joystick_left_btn = QPushButton("←")
        self.joystick_left_btn.setFixedSize(60,60)
        self.joystick_left_btn.pressed.connect(lambda: self.on_joystick_pressed("left"))
        self.joystick_left_btn.released.connect(self.on_joystick_released)

        self.joystick_right_btn = QPushButton("→")
        self.joystick_right_btn.setFixedSize(60,60)
        self.joystick_right_btn.pressed.connect(lambda: self.on_joystick_pressed("right"))
        self.joystick_right_btn.released.connect(self.on_joystick_released)

        self.center_btn = QPushButton("Center\n(Switch)")
        self.center_btn.setFixedSize(70,70)
        self.center_btn.clicked.connect(self.switch_instrument)

        self.joystick_layout.addWidget(self.joystick_up_btn,    0,1)
        self.joystick_layout.addWidget(self.joystick_left_btn,  1,0)
        self.joystick_layout.addWidget(self.center_btn,         1,1)
        self.joystick_layout.addWidget(self.joystick_right_btn, 1,2)
        self.joystick_layout.addWidget(self.joystick_down_btn,  2,1)

        layout.addWidget(self.joystick_widget)

        # ---- Speed Slider (1~15) + 显示数值 ----
        slider_layout = QHBoxLayout()
        lbl_min = QLabel("1 mm/s")
        lbl_min.setStyleSheet(f"font-size: {self.overall_fontsize}px;")

        self.lbl_speed_value = QLabel("Speed: 5 mm/s")
        self.lbl_speed_value.setStyleSheet(f"font-size: {self.overall_fontsize}px;")

        lbl_max = QLabel("15 mm/s")
        lbl_max.setStyleSheet(f"font-size: {self.overall_fontsize}px;")

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1,15)
        self.speed_slider.setValue(5)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)

        slider_layout.addWidget(lbl_min)
        slider_layout.addWidget(self.speed_slider)
        slider_layout.addWidget(lbl_max)
        layout.addLayout(slider_layout)

        # 再放个 “当前导管”Label
        self.current_instrument_index = 0
        self.instrument_list = ["guidewire", "catheter", "midcatheter"]
        self.lbl_instrument = QLabel("Current Instrument: guidewire")
        self.lbl_instrument.setAlignment(Qt.AlignCenter)
        self.lbl_instrument.setStyleSheet(f"font-size: {self.overall_fontsize}px;")

        layout.addWidget(self.lbl_instrument)
        layout.addWidget(self.lbl_speed_value)  # 显示速度

        # 定时器 + 方向
        self.move_timer = QtCore.QTimer()
        self.move_timer.setInterval(100)
        self.move_timer.timeout.connect(self.update_move_direction)
        self.joystick_dir = None

    def on_speed_changed(self, value):
        self.lbl_speed_value.setText(f"Speed: {value} mm/s")

    def on_joystick_pressed(self, direction):
        self.joystick_dir = direction
        self.move_timer.start()

    def on_joystick_released(self):
        self.move_timer.stop()
        self.joystick_dir = None

    def update_move_direction(self):
        if self.joystick_dir == "up":
            print("[DEBUG] Move UP, speed =", self.speed_slider.value())
        elif self.joystick_dir == "down":
            print("[DEBUG] Move DOWN, speed =", self.speed_slider.value())
        elif self.joystick_dir == "left":
            print("[DEBUG] Move LEFT, speed =", self.speed_slider.value())
        elif self.joystick_dir == "right":
            print("[DEBUG] Move RIGHT, speed =", self.speed_slider.value())

    def switch_instrument(self):
        self.current_instrument_index = (self.current_instrument_index + 1) % len(self.instrument_list)
        name = self.instrument_list[self.current_instrument_index]
        self.lbl_instrument.setText(f"Current Instrument: {name}")
        print("[DEBUG] Switch instrument =>", name)

    # --- 其余：GUI / Connect / Logging / TimerLoop / etc. ----
    def Create_DevicesCombobox(self):
        self.DevicesCombobox = QComboBox()
        self.DevicesCombobox.addItems(self.connected_ports)

    def Create_ConnectButton(self):
        self.ConnectButton = QPushButton("Connect")
        self.ConnectButton.clicked.connect(self.on_connect_clicked)

    def Create_LoggingButton(self):
        self.LoggingButton = QPushButton("Log Data")
        self.LoggingButton.setEnabled(False)

    def on_connect_clicked(self):
        # 省略具体串口逻辑
        # 连接成功后:
        self.Connection_Flag = True
        self.CtrlJoint_Rbutton.setEnabled(True)
        self.CtrlTask_Rbutton.setEnabled(True)
        self.CtrlCathV1_Rbutton.setEnabled(True)

    def all(self):
        # 在这里做轮询
        pass

    # 以下两个可只做示例
    def Direct_Motor_Ctrl_mode(self):
        if self.CtrlJoint_Rbutton.isChecked():
            print("[DEBUG] Direct Motor Control mode selected")
            # 你若只想调 Catheter v1，就不必管下面

    def Task_Space_Ctrl_mode(self):
        if self.CtrlTask_Rbutton.isChecked():
            print("[DEBUG] Task Space Control mode selected")

    def CatheterV1_Task_Ctrl_mode(self):
        if self.CtrlCathV1_Rbutton.isChecked():
            print("[DEBUG] Catheter v1 Task Control selected")
            # 显示
            self.CatheterV1_Block.setVisible(True)
        else:
            self.CatheterV1_Block.setVisible(False)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
