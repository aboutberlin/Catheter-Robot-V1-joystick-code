import serial
from serial.tools import list_ports
import time
import datetime as dt
from math import *
import csv
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import numpy as np
import sys

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
        # self.setMinimumSize(1920, 1080)
        self.overall_fontsize = 16
        self.label_style = {"font-size": "16px"}
        self.title_style = {"color": "black", "font-size": "20px"}
        
        self.connected_ports = find_available_ports()

        self.t_0      = 0 # Initial time
        self.t_prev   = 0 # Previuos time
        self.t        = 0 # Current time
        self.t_teensy = 0 # Time in the Teensy 4.1

        self.Connection_Flag    = False
        self.LogginButton_Flag  = False
        self.Data_Received_Flag = 0
        self.first_teensy_time  = True
        self.Control_Mode_Flag = 0 # 0 = Direct Motor Control | 1 = Task Space Control
        self.Rotation_Button_Clicked = False

        # Gear parameters
        self.r_G1 = 8 # radius [8 mm] (17 teeth) This is the gear attached to the rotation motion motor (M1)
        self.r_G2 = 30 # radius [30 mm] (60 teeth) This is the gear that rotatates the whole inner mechanism
        self.r_G3 = 32 # radius [32 mm] (64 teeth) This is the gear attached to the insertion motor (M2)
        self.r_G4 = 10 # radius [10 mm] (20 teeth) This is the gear that drives the bevel gears (Bevel gears are 10 mm Diam)
        self.d_Roller = 20 # diameter [20 mm] This is the diameter of the rollers moving (forward/backward) the catheter
        # Expressions 
        # Catheter Rotation: theta_G2 = (r_G1/r_G2)*theta_M1
        # Catheter Translation: l_T = PI*d_Roller*(r_G3/r_G4)*theta_M2
        # NOTE: To decouple translation from rotation: theta_M2 = (r_G1/r_G2)*theta_M1
        #################

        self.fps = 60
        self.Plots_RefreshRate = round(1/self.fps)
        self.Teensy_freq_ble   = 60
        self.plot_time_window  = 10
        self.buffer_size  = (self.plot_time_window/2) * self.Teensy_freq_ble
        self.t_buffer = list([0] *round(self.buffer_size))
        # Instrument 1
        ## I1 Translation
        self.I1_T_Motor_pos      = 0
        self.I1_T_Motor_posref   = 0
        self.I1_T_Effector_pos    = 0
        self.I1_T_Effector_posref = 0
        self.I1_T_Motor_pos_buff      = self.t_buffer.copy()
        self.I1_T_Motor_posref_buff   = self.t_buffer.copy()
        self.I1_T_Effector_pos_buff    = self.t_buffer.copy()
        self.I1_T_Effector_posref_buff = self.t_buffer.copy()
        ## I1 Rotation
        self.I1_R_Motor_pos      = 0
        self.I1_R_Motor_posref   = 0
        self.I1_R_Effector_pos    = 0
        self.I1_R_Effector_posref = 0
        self.I1_R_Motor_pos_buff      = self.t_buffer.copy()
        self.I1_R_Motor_posref_buff   = self.t_buffer.copy()
        self.I1_R_Effector_pos_buff    = self.t_buffer.copy()
        self.I1_R_Effector_posref_buff = self.t_buffer.copy()

        # Instrument 2
        ## I2 Translation
        self.I2_T_Motor_pos      = 0
        self.I2_T_Motor_posref   = 0
        self.I2_T_Effector_pos    = 0
        self.I2_T_Effector_posref = 0
        self.I2_T_Motor_pos_buff      = self.t_buffer.copy()
        self.I2_T_Motor_posref_buff   = self.t_buffer.copy()
        self.I2_T_Effector_pos_buff    = self.t_buffer.copy()
        self.I2_T_Effector_posref_buff = self.t_buffer.copy()
        ## I2 Rotation
        self.I2_R_Motor_pos      = 0
        self.I2_R_Motor_posref   = 0
        self.I2_R_Effector_pos    = 0
        self.I2_R_Effector_posref = 0
        self.I2_R_Motor_pos_buff      = self.t_buffer.copy()
        self.I2_R_Motor_posref_buff   = self.t_buffer.copy()
        self.I2_R_Effector_pos_buff    = self.t_buffer.copy()
        self.I2_R_Effector_posref_buff = self.t_buffer.copy()
        # Instrument 3
        ## I3 Translation
        self.I3_T_Motor_pos      = 0
        self.I3_T_Motor_posref   = 0
        self.I3_T_Effector_pos    = 0
        self.I3_T_Effector_posref = 0
        self.I3_T_Motor_pos_buff      = self.t_buffer.copy()
        self.I3_T_Motor_posref_buff   = self.t_buffer.copy()
        self.I3_T_Effector_pos_buff    = self.t_buffer.copy()
        self.I3_T_Effector_posref_buff = self.t_buffer.copy()
        ## I3 Rotation
        self.I3_R_Motor_pos      = 0
        self.I3_R_Motor_posref   = 0
        self.I3_R_Effector_pos    = 0
        self.I3_R_Effector_posref = 0
        self.I3_R_Motor_pos_buff      = self.t_buffer.copy()
        self.I3_R_Motor_posref_buff   = self.t_buffer.copy()
        self.I3_R_Effector_pos_buff    = self.t_buffer.copy()
        self.I3_R_Effector_posref_buff = self.t_buffer.copy()

        self.red   = pg.mkPen(color=(255, 0, 0), width = 2)
        self.blue  = pg.mkPen(color=(0, 0, 255), width = 2)
        self.green = pg.mkPen(color=(0, 255, 0), width = 2)

        # Layout def
        MainLayout         = QVBoxLayout() # Window Layout
        self.Comm_Layout   = QHBoxLayout() # Layout for the COM pot selection and the Log data button
        self.Instruments_Layout = QHBoxLayout() # Control mode Layout


        # 3) 下面加一个 ScrollArea 来放 “Catheter v1” 这个界面
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        MainLayout.addWidget(self.scroll, stretch=1)





        # Initialize UI elements
        self.Create_DevicesCombobox()
        self.Create_ConnectButton()
        self.Create_LoggingButton()
        self.Create_CtrlMode_Block()
        self.Create_CatheterV1_Block()
        # self.Create_DMC_Instrument_1_Block()
        # self.Create_DMC_Instrument_2_Block()
        # self.Create_DMC_Instrument_3_Block()

        # Add widgets to layouts
        # Communications Layout
        self.stretch_factor = 10
        self.Device_Label = QLabel("Device:")
        self.Device_Label.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Comm_Layout.addWidget(self.Device_Label)
        self.Comm_Layout.addWidget(self.DevicesCombobox, stretch=self.stretch_factor)
        self.Comm_Layout.addWidget(self.ConnectButton, stretch=self.stretch_factor)
        self.Comm_Layout.addWidget(self.LoggingButton, stretch=self.stretch_factor)

        MainLayout.addLayout(self.Comm_Layout)
        MainLayout.addWidget(self.CtrlMode_Block)


        # self.scroll.setWidget(self.CatheterV1_Block)
        # # 默认隐藏
        # self.CatheterV1_Block.setVisible(False)


        # MainLayout.addWidget(self.scroll)
        MainLayout.addWidget(self.CatheterV1_Block)  # 仅显示v1界面

        # MainLayout.addLayout(self.Instruments_Layout)
        # self.Instruments_Layout.addWidget(self.Instrument_1_Block)
        # self.Instruments_Layout.addWidget(self.Instrument_2_Block)
        # self.Instruments_Layout.addWidget(self.Instrument_3_Block)
        self.Instruments_Layout.addWidget(self.CatheterV1_Block)
        self.setLayout(MainLayout)

    # Creation of the timer for executing the function
        self.timer = QtCore.QTimer()
        self.timer.setInterval(self.Plots_RefreshRate)
        self.timer.timeout.connect(self.all)
        self.timer.start()

    def all(self):
        self.update_instruments_state_label()
        if self.Connection_Flag:
            if self.ser.in_waiting > 0:
                self.ConnectButton.setText("Receiving Data")
                self.ConnectButton.setStyleSheet("background-color: green;"
                                                 "color: white;"
                                                 "font: Bold;"
                                                 f"font-size: {self.overall_fontsize}px;")
                self.Receive_data()
                if self.Data_Received_Flag:
                    self.CtrlJoint_Rbutton.setEnabled(True)
                    self.CtrlTask_Rbutton.setEnabled(True)
                    # self.update_plot_data()
                    self.Data_Received_Flag = 0


    def update_instruments_state_label(self):
        """
        每100ms或者按需调用, 更新 label 显示
        """
        # midcatheter insertion => I1_T
        mid_ref = self.I1_T_Motor_posref
        mid_read = self.I1_T_Motor_pos
        # 估算长度: 由 readAngle => mm ( 10deg => 5mm => 1deg => 0.5 mm )
        mid_length_mm = mid_read * 0.5  # 如果 mid_read=10 => length=5mm
        self.lbl_midcatheter_ref.setText(f"Midcatheter RefAngle: {mid_ref:.3f} deg")
        self.lbl_midcatheter_read.setText(f"Midcatheter ReadAngle: {mid_read:.3f} deg")
        self.lbl_midcatheter_len.setText(f"Midcatheter Length: {mid_length_mm:.3f} mm")

        # guidewire insertion => I1_R
        gw_in_ref = self.I1_R_Motor_posref
        gw_in_read = self.I1_R_Motor_pos
        gw_length_mm = gw_in_read * 0.5  # 如果 mid_read=10 => length=5mm
        # guidewire insertion 也是 10deg=>5mm => ratio=0.5
        # 旋转 => I2_T
        gw_rot_ref = self.I2_T_Motor_posref
        gw_rot_read = self.I2_T_Motor_pos
        self.lbl_guidewire_in_ref.setText(f"Guidewire Insert RefAngle: {gw_in_ref:.3f} deg")
        self.lbl_guidewire_in_read.setText(f"Guidewire Insert ReadAngle: {gw_in_read:.3f} deg")
        self.lbl_guidewire_len.setText(f"Guidewire Length: {gw_length_mm:.3f} mm")
        self.lbl_guidewire_rot_ref.setText(f"Guidewire Rotate RefAngle: {gw_rot_ref:.3f} deg")
        self.lbl_guidewire_rot_read.setText(f"Guidewire Rotate ReadAngle: {gw_rot_read:.3f} deg")

        # catheter insertion => I3_T
        cat_in_ref = self.I3_T_Motor_posref
        cat_in_read = self.I3_T_Motor_pos
        # 你说 +5mm => -1000deg => 1mm => -200deg => 1deg => -0.005 mm
        # cat_in_read deg => length = cat_in_read * (-0.005)
        cat_length_mm = cat_in_read * (-0.005) 
        cat_rot_ref = self.I3_R_Motor_posref
        cat_rot_read = self.I3_R_Motor_pos
        self.lbl_catheter_in_ref.setText(f"Catheter Insert RefAngle: {cat_in_ref:.3f} deg")
        self.lbl_catheter_in_read.setText(f"Catheter Insert ReadAngle: {cat_in_read:.3f} deg")
        self.lbl_catheter_len.setText(f"Catheter Length: {cat_length_mm:.3f} mm")
        self.lbl_catheter_rot_ref.setText(f"Catheter Rotate RefAngle: {cat_rot_ref:.3f} deg")
        self.lbl_catheter_rot_read.setText(f"Catheter Rotate ReadAngle: {cat_rot_read:.3f} deg")


    # Function to connect to the selected device
    def Connect_Button_Clicked(self):
        ### Defining the size of the received packages ###
        self.RXD_datapackage_length = 32 # Receive data package size
        self.TXD_datapackage_length = 20 # Transmit data package size
        self.data_length        = self.RXD_datapackage_length - 3
        self.decoded_data       = [0]*self.data_length
        self.BAUD_RATE          = 115200
        self.SER_PORT           = self.DevicesCombobox.currentText()

        self.ser = serial.Serial(port = self.SER_PORT, baudrate = self.BAUD_RATE)
        self.ser.timeout = 0 # set read timeout
        if self.SER_PORT:
            print(f"Connecting to {self.SER_PORT}...")
            print(self.ser)
            if self.ser.is_open:
                print(f'{self.SER_PORT} port opened')
                self.ConnectButton.setText("Connecting...")
                self.ConnectButton.setStyleSheet("background-color: green;"
                                                 "color: white;"
                                                 "font: Bold;"
                                                 f"font-size: {self.overall_fontsize}px;")
                self.LoggingButton.setEnabled(True)
                self.LoggingButton.setStyleSheet("background-color: rgb(0, 0, 150);"
                                                 "color: white;"
                                                 "font: Bold;"
                                                 f"font-size: {self.overall_fontsize}px;")
                self.Connection_Flag = True
                # 原先只有前两个
                self.CtrlJoint_Rbutton.setEnabled(True)
                self.CtrlTask_Rbutton.setEnabled(True)
                # 新增：启用catheter v1模式
                self.CtrlCathV1_Rbutton.setEnabled(True)

    def Receive_data(self):
        if self.ser.in_waiting >= self.RXD_datapackage_length:
            if self.ser.read(1) == b'\xA5':  # 165 in uint8
                if self.ser.read(1) == b'\x5A':  # 90 in uint8
                    self.expected_length = self.ser.read(1)
                    if self.expected_length == bytes([self.RXD_datapackage_length]):
                        # 开始真正读取剩余的 32-3=29 字节
                        raw_data = self.ser.read(self.RXD_datapackage_length - 3)  # 29字节

                        # [0..3] => time (int32)
                        t_teensy_4B = int.from_bytes(raw_data[0:4], byteorder='little', signed=True)
                        # [4..7] => M1_pos (int32)
                        M1_pos_4B   = int.from_bytes(raw_data[4:8],  byteorder='little', signed=True)
                        # [8..11] => M2_pos
                        M2_pos_4B   = int.from_bytes(raw_data[8:12], byteorder='little', signed=True)
                        M3_pos_4B   = int.from_bytes(raw_data[12:16], byteorder='little', signed=True)
                        M4_pos_4B = 0
                        # **解码方向和模式**
                        direction_encoded = raw_data[16]
                        mode_encoded = raw_data[17]                        
                        M5_pos_4B   = int.from_bytes(raw_data[20:24], byteorder='little', signed=True)
                        M6_pos_4B   = int.from_bytes(raw_data[24:28], byteorder='little', signed=True)
                        # ...
                        # 依次解析 M3, M4, M5, M6
                        # 最后 1 字节 [28] 就是 123（校验）

                        # 再做 /10.0 得到真正的“度”
                        self.t_teensy       = float(t_teensy_4B)/100.0
                        self.I1_T_Motor_pos = float(M1_pos_4B)/100.0
                        self.I1_R_Motor_pos = float(M2_pos_4B)/100.0
                        self.I2_T_Motor_pos = float(M3_pos_4B)/100.0
                        self.I2_R_Motor_pos = float(M4_pos_4B)/100.0
                        self.I3_T_Motor_pos = float(M5_pos_4B)/100.0
                        self.I3_R_Motor_pos = float(M6_pos_4B)/100.0

                        direction_mapping = {0: "Center", 1: "Up", 2: "Down", 3: "Left", 4: "Right"}
                        self.joystick_direction = direction_mapping.get(direction_encoded, "Unknown")

                        # **模式解码**
                        self.joystick_mode = mode_encoded  # 直接使用

                    # **在这里定义 mode_to_instrument**
                        mode_to_instrument = {
                            1: "guidewire",
                            2: "midcatheter",
                            3: "catheter"
                        }
                        # print(f"[DEBUG] Joystick Direction: {self.joystick_direction}, Mode: {self.joystick_mode}")
                        self.lbl_joystick_status.setText(f"Joystick: {self.joystick_direction} | Mode: {self.joystick_mode}")

                        # 更新 GUI 上的当前 instrument
                        self.lbl_instrument.setText(f"Current Instrument: {mode_to_instrument.get(self.joystick_mode, 'guidewire')}")
    # def Receive_data(self):
    #     if self.ser.in_waiting >= self.RXD_datapackage_length:
    #         if self.ser.read(1) == b'\xA5':  # 165 in uint8
    #             if self.ser.read(1) == b'\x5A':  # 90 in uint8
    #                 self.expected_length = self.ser.read(1)
    #                 if self.expected_length == bytes([self.RXD_datapackage_length]):
    #                     if self.ser.in_waiting >= self.data_length:
    #                         coded_data = self.ser.read(self.data_length)
    #                         # self.decoded_data = [0] * (13)
    #                         decode_i = 0
    #                         for i in range(1, self.data_length, 2):
    #                             var = coded_data[i-1] + coded_data[i] * 256
    #                             var = (var - 65536) / 10.0 if var > 32767 else var / 10.0
    #                             self.decoded_data[decode_i] = var
    #                             decode_i += 1
                        
    #                         self.t_teensy       = self.decoded_data[0]
    #                         self.I1_T_Motor_pos = self.decoded_data[1]
    #                         self.I1_R_Motor_pos = self.decoded_data[2]
    #                         self.I2_T_Motor_pos = self.decoded_data[3]
    #                         self.I2_R_Motor_pos = self.decoded_data[4]
    #                         self.I3_T_Motor_pos = self.decoded_data[5]
    #                         self.I3_R_Motor_pos = self.decoded_data[6]
    #                         self.Data_Received_Flag = coded_data[25]
                            
    #                         if self.first_teensy_time:
    #                             self.t_0_teensy = self.t_teensy
    #                             self.first_teensy_time = False

    def Transmit_data(self):
        packet_length = 32  # or 32, 看你如何设计

        # 构造一个 array
        self.data_package = bytearray(packet_length)

        # 1) 包头
        self.data_package[0] = 165
        self.data_package[1] = 90
        self.data_package[2] = packet_length

        # 2) 假如你不想发time，就只发 6个int32
        # python里: I1_M1_cmd = int( self.I1_T_Motor_posref * 10 )
        I1_M1_cmd = int(self.I1_T_Motor_posref * 100)
        I1_M2_cmd = int(self.I1_R_Motor_posref * 100)
        I2_M1_cmd = int(self.I2_T_Motor_posref * 100)
        I2_M2_cmd = int(self.I2_R_Motor_posref * 100)
        I3_M1_cmd = int(self.I3_T_Motor_posref * 100)
        I3_M2_cmd = int(self.I3_R_Motor_posref * 100)
        # ...
        # 同理其他 5 个

        # 3) 把 6 个 int32 依次放到 data_package
        # [3..6], [7..10], [11..14], ...
        self.data_package[3:7] = I1_M1_cmd.to_bytes(4, 'little', signed=True)
        self.data_package[7:11] = I1_M2_cmd.to_bytes(4, 'little', signed=True)
        self.data_package[11:15] = I2_M1_cmd.to_bytes(4, 'little', signed=True)
        self.data_package[15:19] = I2_M2_cmd.to_bytes(4, 'little', signed=True)
        self.data_package[19:23] = I3_M1_cmd.to_bytes(4, 'little', signed=True)
        self.data_package[23:27] = I3_M2_cmd.to_bytes(4, 'little', signed=True)
        # ...
        # data_package[...]= ...
        # ...
        if self.ser.is_open:
            self.ser.write(self.data_package)

    def update_plot_data(self):
        if self.Connection_Flag == True:

            self.t = time.time() - self.t_0

            if self.t_prev != self.t:
                # This must be improved
                if self.Rotation_Button_Clicked:
                    self.I1_T_Effector_pos = self.I1_T_Effector_pos
                    self.I1_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I1_R_Motor_pos)
                    self.I2_T_Effector_pos = self.I2_T_Effector_pos
                    self.I2_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I2_R_Motor_pos)
                    self.I3_T_Effector_pos = self.I3_T_Effector_pos
                    self.I3_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I3_R_Motor_pos)
                else:
                    self.I1_T_Effector_pos = self.Translation_Transform_M2_to_Catheter(self.I1_T_Motor_pos)
                    self.I1_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I1_R_Motor_pos)
                    self.I2_T_Effector_pos = self.Translation_Transform_M2_to_Catheter(self.I2_T_Motor_pos)
                    self.I2_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I2_R_Motor_pos)
                    self.I3_T_Effector_pos = self.Translation_Transform_M2_to_Catheter(self.I3_T_Motor_pos)
                    self.I3_R_Effector_pos = self.Rotation_Transform_M1_to_Catheter(self.I3_R_Motor_pos)
                # 
                self.current_time = self.t_teensy - self.t_0_teensy
                self.buffers = [self.t_buffer,
                                self.I1_T_Motor_pos_buff, self.I1_T_Motor_posref_buff,
                                self.I1_R_Motor_pos_buff, self.I1_R_Motor_posref_buff,
                                self.I2_T_Motor_pos_buff, self.I2_T_Motor_posref_buff,
                                self.I2_R_Motor_pos_buff, self.I2_R_Motor_posref_buff,
                                self.I3_T_Motor_pos_buff, self.I3_T_Motor_posref_buff,
                                self.I3_R_Motor_pos_buff, self.I3_R_Motor_posref_buff,
                                self.I1_T_Effector_pos_buff, self.I1_T_Effector_posref_buff,
                                self.I1_R_Effector_pos_buff, self.I1_R_Effector_posref_buff,
                                self.I2_T_Effector_pos_buff, self.I2_T_Effector_posref_buff,
                                self.I2_R_Effector_pos_buff, self.I2_R_Effector_posref_buff,
                                self.I3_T_Effector_pos_buff, self.I3_T_Effector_posref_buff,
                                self.I3_R_Effector_pos_buff, self.I3_R_Effector_posref_buff]
                self.values  = [self.current_time,
                                self.I1_T_Motor_pos, self.I1_T_Motor_posref/100,
                                self.I1_R_Motor_pos, self.I1_R_Motor_posref/100,
                                self.I2_T_Motor_pos, self.I2_T_Motor_posref/100,
                                self.I2_R_Motor_pos, self.I2_R_Motor_posref/100,
                                self.I3_T_Motor_pos, self.I3_T_Motor_posref/100,
                                self.I3_R_Motor_pos, self.I3_R_Motor_posref/100,
                                self.I1_T_Effector_pos, self.I1_T_Effector_posref/100,
                                self.I1_R_Effector_pos, self.I1_R_Effector_posref/100,
                                self.I2_T_Effector_pos, self.I2_T_Effector_posref/100,
                                self.I2_R_Effector_pos, self.I2_R_Effector_posref/100,
                                self.I3_T_Effector_pos, self.I3_T_Effector_posref/100,
                                self.I3_R_Effector_pos, self.I3_R_Effector_posref/100]
                self.update_buffers(self.buffers, self.values)

                # Instrument 1
                ## Translation
                self.I1_T_Motor_pos_line.setData(self.t_buffer, self.I1_T_Motor_pos_buff)
                self.I1_T_Effector_pos_line.setData(self.t_buffer, self.I1_T_Effector_pos_buff)
                self.I1_T_Block_Label.setText(str(self.I1_T_Motor_pos))
                ## Rotation
                self.I1_R_Motor_pos_line.setData(self.t_buffer, self.I1_R_Motor_pos_buff)
                self.I1_R_Effector_pos_line.setData(self.t_buffer, self.I1_R_Effector_pos_buff)                
                self.I1_R_Block_Label.setText(str(self.I1_R_Motor_pos))

                # Instrument 2
                ## Translation
                self.I2_T_Motor_pos_line.setData(self.t_buffer, self.I2_T_Motor_pos_buff)
                self.I2_T_Effector_pos_line.setData(self.t_buffer, self.I2_T_Effector_pos_buff)
                self.I2_T_Block_Label.setText(str(self.I2_T_Motor_pos))
                ## Rotation
                self.I2_R_Motor_pos_line.setData(self.t_buffer, self.I2_R_Motor_pos_buff)
                self.I2_R_Effector_pos_line.setData(self.t_buffer, self.I2_R_Effector_pos_buff)
                self.I2_R_Block_Label.setText(str(self.I2_R_Motor_pos))

                # Instrument 3
                ## Translation
                self.I3_T_Motor_pos_line.setData(self.t_buffer, [val * 1000 for val in self.I3_T_Motor_pos_buff])
                self.I3_T_Effector_pos_line.setData(self.t_buffer, [val * 1000 for val in self.I3_T_Effector_pos_buff])
                self.I3_T_Block_Label.setText(str(self.I3_T_Motor_pos * 1000))
                ## Rotation
                self.I3_R_Motor_pos_line.setData(self.t_buffer, self.I3_R_Motor_pos_buff)
                self.I3_R_Effector_pos_line.setData(self.t_buffer, self.I3_R_Effector_pos_buff)
                self.I3_R_Block_Label.setText(str(self.I3_R_Motor_pos))

                if not self.Control_Mode_Flag:
                    self.I1_T_cmd_line.setData(self.t_buffer, self.I1_T_Motor_posref_buff)
                    self.I1_R_cmd_line.setData(self.t_buffer, self.I1_R_Motor_posref_buff)
                    self.I2_T_cmd_line.setData(self.t_buffer, self.I2_T_Motor_posref_buff)
                    self.I2_R_cmd_line.setData(self.t_buffer, self.I2_R_Motor_posref_buff)
                    self.I3_T_cmd_line.setData(self.t_buffer, self.I3_T_Motor_posref_buff)
                    self.I3_R_cmd_line.setData(self.t_buffer, self.I3_R_Motor_posref_buff)
                else:
                    self.I1_T_cmd_line.setData(self.t_buffer, self.I1_T_Effector_posref_buff)
                    self.I1_R_cmd_line.setData(self.t_buffer, self.I1_R_Effector_posref_buff)
                    self.I2_T_cmd_line.setData(self.t_buffer, self.I2_T_Effector_posref_buff)
                    self.I2_R_cmd_line.setData(self.t_buffer, self.I2_R_Effector_posref_buff)
                    self.I3_T_cmd_line.setData(self.t_buffer, self.I3_T_Effector_posref_buff)
                    self.I3_R_cmd_line.setData(self.t_buffer, self.I3_R_Effector_posref_buff)

                if self.LogginButton_Flag == True:
                    self.LoggedData = {
                        "time": self.t,                        
                        "M1_Pos_cmd": self.M1_pos_cmd
                    }

                    with open(self.csv_file_name, mode="a", newline="") as self.file:
                        self.writer = csv.DictWriter(self.file, fieldnames = self.DataHeaders)
                        self.writer.writerow(self.LoggedData)
            self.t_prev = self.t
        else:
            print("NOT Connected")

    def update_buffers(self, buffers, values):
        for buffer, value in zip(buffers, values):
            buffer[:] = buffer[1:]  # This modifies the original buffer
            buffer.append(value)

    def LogginButton_Clicked(self):

        self.LogginButton_Flag = True

        self.t_0 = time.time()
        self.LoggingButton.setText("Logging data")
        self.LoggingButton.setStyleSheet("background-color : blue")
        self.csv_file_name = "GUI_Logger_" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
        self.DataHeaders   = ["time", "M1_Pos", "M2_Pos", "M3_Pos", "M4_Pos", "M5_Pos"]

        with open(self.csv_file_name, mode="w", newline="") as self.file:
            self.writer = csv.DictWriter(self.file, fieldnames = self.DataHeaders)
            self.writer.writeheader()

    def Create_DevicesCombobox(self):
        self.DevicesCombobox = QComboBox()
        self.DevicesCombobox.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.DevicesCombobox.addItems(self.connected_ports)
    
    def Create_ConnectButton(self):
        self.ConnectButton = QPushButton("Connect")
        self.ConnectButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.ConnectButton.clicked.connect(self.Connect_Button_Clicked)

    def Create_LoggingButton(self):
        self.LoggingButton = QPushButton("Log Data")
        self.LoggingButton.setEnabled(False)
        self.LoggingButton.setStyleSheet("background-color: rgb(150, 150, 150);"
                                    "color: white;"
                                    "font: Bold;"
                                    f"font-size: {self.overall_fontsize}px;")
        self.LoggingButton.clicked.connect(self.LogginButton_Clicked)

    def Create_CtrlMode_Block(self):
        ## Level of Assistance Block
        self.CtrlMode_Block  = QGroupBox("Control Mode")
        self.CtrlMode_Block.setAlignment(Qt.AlignCenter)
        self.CtrlMode_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.CtrlMode_Block_Layout = QHBoxLayout()
        self.CtrlMode_Block.setLayout(self.CtrlMode_Block_Layout)
        self.CtrlMode_Block.setEnabled(True)
        # Control Modes
        self.CtrlJoint_Rbutton = QRadioButton("Direct Motor Control")
        self.CtrlTask_Rbutton  = QRadioButton("Task Space Control")
        self.CtrlCathV1_Rbutton = QRadioButton("Catheter v1 Task Control")



    # 把三个RadioButton都加入布局

        self.CtrlMode_Block_Layout.addWidget(self.CtrlJoint_Rbutton)
        self.CtrlMode_Block_Layout.addWidget(self.CtrlTask_Rbutton)
        self.CtrlMode_Block_Layout.addWidget(self.CtrlCathV1_Rbutton)

    # 默认都先设为Disabled，后续等串口连接后启用
        # 3) 连接槽函数

        self.CtrlJoint_Rbutton.setEnabled(False)
        self.CtrlJoint_Rbutton.toggled.connect(self.Direct_Motor_Ctrl_mode)
        self.CtrlTask_Rbutton.setEnabled(False)
        self.CtrlTask_Rbutton.toggled.connect(self.Task_Space_Ctrl_mode)
        self.CtrlCathV1_Rbutton.setEnabled(False)
        self.CtrlCathV1_Rbutton.toggled.connect(self.CatheterV1_Task_Ctrl_mode)



    def Create_CatheterV1_Block(self):
        """
        创建一个新的界面块(GroupBox)，
        供 'Catheter v1 Task Control' 模式下使用
        """
        self.CatheterV1_Block = QGroupBox("Catheter v1 Task Control Interface")
        self.CatheterV1_Block.setAlignment(Qt.AlignCenter)
        self.CatheterV1_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.CatheterV1_Block_Layout = QVBoxLayout()
        self.CatheterV1_Block.setLayout(self.CatheterV1_Block_Layout)

        # ========== 1) 顶部提示或说明 ==========
        self.lblCatheterHint = QLabel("Catheter v1 Control - Joystick + Speed Slider")
        self.lblCatheterHint.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.lblCatheterHint.setAlignment(Qt.AlignCenter)
        self.CatheterV1_Block_Layout.addWidget(self.lblCatheterHint)

        # 添加一个“物理摇杆启用”按钮
        self.joystick_physical_toggle_btn = QPushButton("Enable Physical Joystick")
        self.joystick_physical_toggle_btn.setCheckable(True)  # 使其成为切换按钮（Toggle）
        self.joystick_physical_toggle_btn.setStyleSheet("font-size: 14px; padding: 5px;")
        self.joystick_physical_toggle_btn.clicked.connect(self.toggle_physical_joystick)

        # 将按钮添加到布局
        self.CatheterV1_Block_Layout.addWidget(self.joystick_physical_toggle_btn)




        # ========== 2) 中间的“遥杆”界面（上下左右 + 中间按钮）==========
        self.joystick_widget = QWidget()
        self.joystick_layout = QGridLayout()
        self.joystick_widget.setLayout(self.joystick_layout)

        # 上按钮
        self.joystick_up_btn = QPushButton("↑")
        self.joystick_up_btn.setFixedSize(60,60)
        # 长按示例：pressed / released + QTimer
        self.joystick_up_btn.pressed.connect(self.CatheterV1_up_pressed)
        self.joystick_up_btn.released.connect(self.CatheterV1_up_released)

        # 下按钮
        self.joystick_down_btn = QPushButton("↓")
        self.joystick_down_btn.setFixedSize(60,60)
        self.joystick_down_btn.pressed.connect(self.CatheterV1_down_pressed)
        self.joystick_down_btn.released.connect(self.CatheterV1_down_released)

        # 左按钮
        self.joystick_left_btn = QPushButton("←")
        self.joystick_left_btn.setFixedSize(60,60)
        self.joystick_left_btn.pressed.connect(self.CatheterV1_left_pressed)
        self.joystick_left_btn.released.connect(self.CatheterV1_left_released)

        # 右按钮
        self.joystick_right_btn = QPushButton("→")
        self.joystick_right_btn.setFixedSize(60,60)
        self.joystick_right_btn.pressed.connect(self.CatheterV1_right_pressed)
        self.joystick_right_btn.released.connect(self.CatheterV1_right_released)

        # 中心按钮 - 切换instrument
        self.joystick_center_btn = QPushButton("Center\n(Switch)")
        self.joystick_center_btn.setFixedSize(70,70)
        self.joystick_center_btn.clicked.connect(self.CatheterV1_center_clicked)

        # 布局：上排留空、中排左中右、下排留空
        # 也可以更简洁直接排
        self.joystick_layout.addWidget(self.joystick_up_btn,    0, 1, alignment=Qt.AlignCenter)
        self.joystick_layout.addWidget(self.joystick_left_btn,  1, 0, alignment=Qt.AlignCenter)
        self.joystick_layout.addWidget(self.joystick_center_btn,1, 1, alignment=Qt.AlignCenter)
        self.joystick_layout.addWidget(self.joystick_right_btn, 1, 2, alignment=Qt.AlignCenter)
        self.joystick_layout.addWidget(self.joystick_down_btn,  2, 1, alignment=Qt.AlignCenter)

        self.CatheterV1_Block_Layout.addWidget(self.joystick_widget)

        # ========== 3) 速度滑条 (1 mm/s ~ 100 mm/s) ==========
        slider_layout = QHBoxLayout()
        lbl_min = QLabel("1 mm/s")
        lbl_min.setStyleSheet(f"font-size: {self.overall_fontsize}px;")

        self.lbl_speed_value = QLabel("Speed: 5 mm/s")
        self.lbl_speed_value.setStyleSheet(f"font-size: {self.overall_fontsize}px;")

        lbl_max = QLabel("100 mm/s")
        lbl_max.setStyleSheet(f"font-size: {self.overall_fontsize}px;")

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1,1000)
        self.speed_slider.setValue(5)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)

        slider_layout.addWidget(lbl_min)
        slider_layout.addWidget(self.speed_slider)
        slider_layout.addWidget(lbl_max)
        self.CatheterV1_Block_Layout.addLayout(slider_layout)



        self.Create_InstrumentsState_Block()
        self.CatheterV1_Block_Layout.addWidget(self.InstrumentsState_Block)




        # ========== 4) 显示当前Instrument标签 ==========
        self.current_instrument_index = 0
        self.instruments_list = ["guidewire", "catheter", "midcatheter"]

        self.lbl_instrument = QLabel("Current Instrument: guidewire")
        self.lbl_instrument.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.lbl_instrument.setAlignment(Qt.AlignCenter)
        self.CatheterV1_Block_Layout.addWidget(self.lbl_instrument)
        self.CatheterV1_Block_Layout.addWidget(self.lbl_speed_value)
        # ========== 5) 定时器(可选，用于长按自动操作) ==========
        self.joystick_timer = QtCore.QTimer()
        self.joystick_timer.setInterval(100)  # 每 100ms 触发一次
        self.joystick_timer.timeout.connect(self.CatheterV1_timer_update)

        # ========== 6) Joystick 方向和模式状态显示 ==========
        self.lbl_joystick_status = QLabel("Joystick: Center | Mode: 1")
        self.lbl_joystick_status.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.lbl_joystick_status.setAlignment(Qt.AlignCenter)
        self.CatheterV1_Block_Layout.addWidget(self.lbl_joystick_status)

        # 默认先隐藏(或setEnabled(False))，只有选中Catheter v1后再显示
        self.CatheterV1_Block.setVisible(False)

    def toggle_physical_joystick(self):
        if self.joystick_physical_toggle_btn.isChecked():
            self.joystick_timer.start()
            self.joystick_physical_toggle_btn.setText("Disable Physical Joystick")
            print("[DEBUG] Physical Joystick Enabled: ON")
        else:
            self.joystick_timer.stop()
            self.joystick_physical_toggle_btn.setText("Enable Physical Joystick")
            print("[DEBUG] Physical Joystick Enabled: OFF")


    def on_speed_changed(self, value):
        self.lbl_speed_value.setText(f"Speed: {value} mm/s")

    def CatheterV1_up_pressed(self):
        print("[DEBUG] Up Pressed")
        self.joystick_current_direction = "up"
        self.joystick_timer.start()   # 开始周期性触发

    def CatheterV1_up_released(self):
        print("[DEBUG] Up Released")
        self.joystick_timer.stop()

    def CatheterV1_down_pressed(self):
        print("[DEBUG] Down Pressed")
        self.joystick_current_direction = "down"
        self.joystick_timer.start()

    def CatheterV1_down_released(self):
        print("[DEBUG] Down Released")
        self.joystick_timer.stop()

    def CatheterV1_left_pressed(self):
        print("[DEBUG] Left Pressed")
        self.joystick_current_direction = "left"
        self.joystick_timer.start()

    def CatheterV1_left_released(self):
        print("[DEBUG] Left Released")
        self.joystick_timer.stop()

    def CatheterV1_right_pressed(self):
        print("[DEBUG] Right Pressed")
        self.joystick_current_direction = "right"
        self.joystick_timer.start()

    def CatheterV1_right_released(self):
        print("[DEBUG] Right Released")
        self.joystick_timer.stop()

    def CatheterV1_timer_update(self):
        """
            定时器触发，用于控制导管移动。
            1. **如果 Joystick 处于非 Center 状态，则完全忽略 GUI 方向按钮，直接按 Joystick 控制**
            2. **如果 Joystick 处于 Center，则执行 GUI 方向按钮的逻辑**
        """
        direction = self.joystick_direction  # 物理摇杆方向
        gui_direction = self.joystick_direction  # GUI按钮方向
        instrument = self.instruments_list[self.current_instrument_index]  
        speed_mm_s = self.speed_slider.value()  
        dt = 0.1  

    # **根据 mode 修改 instrument**
        mode_to_instrument = {
            1: "guidewire",
            2: "midcatheter",
            3: "catheter"
        }
        instrument = mode_to_instrument.get(self.joystick_mode, "guidewire")  # 默认 guidewire

        # **Joystick 优先级更高**
        if direction != "Center":
            active_direction = direction  # **Joystick 控制**
        elif gui_direction:
            active_direction = gui_direction  # **GUI 控制**
        else:
            return  # **没有任何输入，直接返回**
        distance_mm = speed_mm_s * dt  

        if active_direction == "Up":
            self.move_instrument_linear(instrument, +distance_mm)
        elif active_direction == "Down":
            self.move_instrument_linear(instrument, -distance_mm)
        elif active_direction == "Left":
            self.move_instrument_rotate(instrument, -distance_mm)
        elif active_direction == "Right":
            self.move_instrument_rotate(instrument, +distance_mm)

    def move_instrument_linear(self, instrument, distance_mm):
        """
        参数:
        instrument: 'guidewire', 'midcatheter', 'catheter'
        distance_mm: 本次要移动的线性距离, 正(+mm)代表前进, 负(-mm)代表后退

        根据你的电机映射, 计算要增加的 motor_posref(角度), 然后 self.Transmit_data()
        """
        print("[DEBUG] Current instrument:", instrument, "distance_mm:", distance_mm)
        # 计算 “delta_angle”
        if instrument == "guidewire":
            # 你说 guidewire前进后退 用 I1_R (10度=>5mm)
            # ratio = 2 deg/mm
            delta_deg = distance_mm * 2.0
            # 更新 self.I1_R_Motor_posref
            self.I1_R_Motor_posref += delta_deg

        elif instrument == "midcatheter":
            # midcatheter前进后退 用 I1_T, ratio=2 deg/mm
            delta_deg = distance_mm * 2.0
            self.I1_T_Motor_posref += delta_deg
            self.I1_R_Motor_posref -= delta_deg

        elif instrument == "catheter":
            # catheter前进后退 用 I3_T, ratio=-200 deg/mm
            # +distance_mm => angle要 -= X
            delta_deg = distance_mm * (-20.0) # 1mm => -200deg
            self.I3_T_Motor_posref += delta_deg

        # 更新后, 发送
        self.Transmit_data()

    def move_instrument_rotate(self, instrument, distance_mm):
        """
        假设: 我们仍然用距离(mm)来表示"转多少", 但这是你自定义的映射,
        例如 1mm => 1°(real) => motorAngle= ratio*1°.
        guidewire ratio=2, catheter ratio=360/280=1.2857
        """
        if instrument == "guidewire":
            # guidewire rotation => I2_T motor
            # ratio=2 => motorAngle = 2 * realAngle
            # 先定义: distance_mm => realAngle=? 假设 1 mm => 1 deg-
            realAngle = distance_mm/50  # 这里等价
            motorAngleDelta = 2.0 * realAngle*50
            self.I2_T_Motor_posref += motorAngleDelta

        elif instrument == "catheter":
            # catheter rotation => I3_R motor
            # ratio = 360/280 ~ 1.2857
            ratio = 360.0/280.0
            realAngle = distance_mm/50  # 仍然 1mm => 1deg 的假设
            motorAngleDelta = ratio * realAngle*50
            self.I3_R_Motor_posref += motorAngleDelta

        elif instrument == "midcatheter":
            # 如果 midcatheter 不支持旋转, 就不做
            print("[DEBUG] midcatheter has no rotation, do nothing")
            return

        # 发送
        self.Transmit_data()

    def Create_InstrumentsState_Block(self):
        """
        用来显示每个电机参考角度/实际角度，以及估算长度等
        """
        self.InstrumentsState_Block = QGroupBox("Instruments State")
        layout = QVBoxLayout()

        # 逐个加 label
        # midcatheter
        self.lbl_midcatheter_ref = QLabel("Midcatheter RefAngle: 0.0 deg")
        self.lbl_midcatheter_read = QLabel("Midcatheter ReadAngle: 0.0 deg")
        self.lbl_midcatheter_len = QLabel("Midcatheter Length: 0.0 mm")

        layout.addWidget(self.lbl_midcatheter_ref)
        layout.addWidget(self.lbl_midcatheter_read)
        layout.addWidget(self.lbl_midcatheter_len)
        layout.addSpacing(10)

        # guidewire
        self.lbl_guidewire_in_ref = QLabel("Guidewire Insert RefAngle: 0.0 deg")
        self.lbl_guidewire_in_read = QLabel("Guidewire Insert ReadAngle: 0.0 deg")
        self.lbl_guidewire_len = QLabel("Midcatheter Length: 0.0 mm")
        self.lbl_guidewire_rot_ref = QLabel("Guidewire Rotate RefAngle: 0.0 deg")
        self.lbl_guidewire_rot_read = QLabel("Guidewire Rotate ReadAngle: 0.0 deg")
        layout.addWidget(self.lbl_guidewire_in_ref)
        layout.addWidget(self.lbl_guidewire_in_read)
        layout.addWidget(self.lbl_guidewire_len)
        layout.addWidget(self.lbl_guidewire_rot_ref)
        layout.addWidget(self.lbl_guidewire_rot_read)
        layout.addSpacing(10)

        # catheter
        self.lbl_catheter_in_ref = QLabel("Catheter Insert RefAngle: 0.0 deg")
        self.lbl_catheter_in_read = QLabel("Catheter Insert ReadAngle: 0.0 deg")
        self.lbl_catheter_len = QLabel("Catheter Length: 0.0 mm")
        self.lbl_catheter_rot_ref = QLabel("Catheter Rotate RefAngle: 0.0 deg")
        self.lbl_catheter_rot_read = QLabel("Catheter Rotate ReadAngle: 0.0 deg")
        layout.addWidget(self.lbl_catheter_in_ref)
        layout.addWidget(self.lbl_catheter_in_read)
        layout.addWidget(self.lbl_catheter_len)
        layout.addWidget(self.lbl_catheter_rot_ref)
        layout.addWidget(self.lbl_catheter_rot_read)

        self.InstrumentsState_Block.setLayout(layout)


    # def CatheterV1_timer_update(self):
    #     """
    #     在这里根据 self.joystick_current_direction 决定做什么动作
    #     每隔 100ms 调一次 => 10次/秒
    #     """
    #     if self.joystick_current_direction == "up":
    #         self.CatheterV1_move_up()
    #     elif self.joystick_current_direction == "down":
    #         self.CatheterV1_move_down()
    #     elif self.joystick_current_direction == "left":
    #         self.CatheterV1_move_left()
    #     elif self.joystick_current_direction == "right":
    #         self.CatheterV1_move_right()

    # def CatheterV1_move_up(self):
    #     current_instrument = self.instruments_list[self.current_instrument_index]  
    #     speed_mm_s = self.speed_slider.value()
    #     print(f"[DEBUG] move_up, Instrument={current_instrument}, Speed={speed_mm_s} mm/s")

    #     # 根据 instrument 执行不同的“旧函数”
    #     if current_instrument == "guidewire":
    #         # guidewire 的“前进后退”对应旧的 I1_R_Up/Down
    #         self.I1_R_UpBtn_Clicked()
    #     elif current_instrument == "midcatheter":
    #         # midcatheter 的“前进后退”对应旧的 I1_T_Up/Down
    #         self.I1_T_UpBtn_Clicked()
    #     elif current_instrument == "catheter":
    #         # catheter 的“前进后退”对应旧的 I3_T_Up/Down
    #         self.I3_T_UpBtn_Clicked()
    #     # 如果你想把 speed 传进去，也可给旧函数加个形参(比如 I1_R_UpBtn_Clicked(speed_mm_s))
    #     # 在旧函数内做角度换算

    # def CatheterV1_move_down(self):
    #     current_instrument = self.instruments_list[self.current_instrument_index]  
    #     speed_mm_s = self.speed_slider.value()
    #     print(f"[DEBUG] move_down, Instrument={current_instrument}, Speed={speed_mm_s} mm/s")
        
    #     if current_instrument == "guidewire":
    #         self.I1_R_DownBtn_Clicked()
    #     elif current_instrument == "midcatheter":
    #         self.I1_T_DownBtn_Clicked()
    #     elif current_instrument == "catheter":
    #         self.I3_T_DownBtn_Clicked()

    # def CatheterV1_move_left(self):
    #     current_instrument = self.instruments_list[self.current_instrument_index]
    #     speed_mm_s = self.speed_slider.value()
    #     print(f"[DEBUG] move_left, Instrument={current_instrument}, Speed={speed_mm_s} mm/s")

    #     # guidewire 的“旋转”对应 旧的 I2_T_Up/Down
    #     # catheter 的“旋转”对应 旧的 I3_R_Up/Down
    #     # midcatheter 如果没有旋转, 就可以什么都不做
    #     if current_instrument == "guidewire":
    #         # 假设 “左” 对应 I2_T_DownBtn_Clicked()，看你需求
    #         self.I2_T_DownBtn_Clicked()
    #     elif current_instrument == "catheter":
    #         self.I3_R_DownBtn_Clicked()
    #     elif current_instrument == "midcatheter":
    #         pass  # midcatheter 不旋转, 不操作

    # def CatheterV1_move_right(self):
    #     current_instrument = self.instruments_list[self.current_instrument_index]
    #     speed_mm_s = self.speed_slider.value()
    #     print(f"[DEBUG] move_right, Instrument={current_instrument}, Speed={speed_mm_s} mm/s")

    #     if current_instrument == "guidewire":
    #         # “右” 对应 I2_T_UpBtn_Clicked()?
    #         self.I2_T_UpBtn_Clicked()
    #     elif current_instrument == "catheter":
    #         self.I3_R_UpBtn_Clicked()
    #     elif current_instrument == "midcatheter":
    #         pass

    def CatheterV1_center_clicked(self):
        """
        切换 guidewire / catheter / midcatheter
        """
        self.current_instrument_index = (self.current_instrument_index + 1) % len(self.instruments_list)
        current_instrument_name = self.instruments_list[self.current_instrument_index]
        self.lbl_instrument.setText(f"Current Instrument: {current_instrument_name}")
        print(f"[DEBUG] Switch to instrument: {current_instrument_name}")

    def CatheterV1_speed_changed(self, value):
        """
        当滑条变化时回调
        """
        print(f"[DEBUG] Speed slider changed => {value} mm/s")
        self.lbl_speed_value.setText(f"Speed: {value} mm/s")

        # 你可以把这个数值保存在某个全局/类成员，然后在 move_xxx() 时用


    def Create_DMC_Instrument_1_Block(self):
        ## Instrument 1 Block
        self.Instrument_1_Block  = QGroupBox("Instrument 1")
        self.Instrument_1_Block.setAlignment(Qt.AlignCenter)
        self.Instrument_1_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Instrument_1_Block_Layout = QVBoxLayout()
        self.Instrument_1_Block.setLayout(self.Instrument_1_Block_Layout)
        self.Instrument_1_Block.setEnabled(False)
        # I1 Translation Block
        self.I1_T_Block  = QGroupBox("Insertion / Withdrawal")
        self.I1_T_Block.setAlignment(Qt.AlignCenter)
        self.I1_T_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I1_T_Block_Layout = QGridLayout()
        self.I1_T_Block.setLayout(self.I1_T_Block_Layout)
        self.I1_T_Block.setEnabled(True)
        self.I1_T_sclebtn10 = QRadioButton("x 10")
        self.I1_T_sclebtn1  = QRadioButton("x 1")
        self.I1_T_sclebtn01 = QRadioButton("x 0.1")
        self.I1_T_Radio_Buttons = [self.I1_T_sclebtn01, self.I1_T_sclebtn1, self.I1_T_sclebtn10]
        self.I1_T_sclebtn1.setChecked(True)
        self.I1_T_upbtn     = QPushButton("^")
        self.I1_T_downbtn   = QPushButton("v")
        self.I1_T_Block_Layout.addWidget(self.I1_T_sclebtn10,1,0)
        self.I1_T_Block_Layout.addWidget(self.I1_T_sclebtn1,2,0)
        self.I1_T_Block_Layout.addWidget(self.I1_T_sclebtn01,3,0)
        self.I1_T_Block_Layout.addWidget(self.I1_T_upbtn,1,1)
        self.I1_T_Block_Label = QLabel(f"deg")
        self.I1_T_Block_Layout.addWidget(self.I1_T_Block_Label,2,1,alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I1_T_Block_Layout.addWidget(self.I1_T_downbtn,3,1)
        self.I1_T_upbtn.clicked.connect(self.I1_T_UpBtn_Clicked)
        self.I1_T_downbtn.clicked.connect(self.I1_T_DownBtn_Clicked)
        # I1 Translation Plot
        self.I1_Translation_Motor_Plot = pg.PlotWidget()
        self.I1_T_Block_Layout.addWidget(self.I1_Translation_Motor_Plot,0,2,5,5)
        self.I1_Translation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I1_Translation_Motor_Plot.setLabel('left', "[deg]", **self.label_style)
        self.I1_Translation_Motor_Plot.setLabel('bottom', "Time [s]", **self.label_style)
        self.I1_Translation_Motor_Plot.addLegend()
        self.I1_Translation_Motor_Plot.setBackground('w')
        self.I1_Translation_Motor_Plot.showGrid(x=True, y=True)
        self.I1_T_cmd_line          = self.I1_Translation_Motor_Plot.plot(self.t_buffer, self.I1_T_Motor_posref_buff, name = "Command", pen = self.blue)
        self.I1_T_Motor_pos_line    = self.I1_Translation_Motor_Plot.plot(self.t_buffer, self.I1_T_Motor_pos_buff, name = "Motor", pen = self.red)
        self.I1_T_Effector_pos_line = self.I1_Translation_Motor_Plot.plot(self.t_buffer, self.I1_T_Effector_pos_buff, name = "Catheter", pen = self.green)
        # I1 Rotation Block
        self.I1_R_Block  = QGroupBox("Rotation")
        self.I1_R_Block.setAlignment(Qt.AlignCenter)
        self.I1_R_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I1_R_Block_Layout = QGridLayout()
        self.I1_R_Block.setLayout(self.I1_R_Block_Layout)
        self.I1_R_Block.setEnabled(True)
        self.I1_R_sclebtn10 = QRadioButton("x 10")
        self.I1_R_sclebtn1  = QRadioButton("x 1")
        self.I1_R_sclebtn01 = QRadioButton("x 0.1")
        self.I1_R_Radio_Buttons = [self.I1_R_sclebtn01, self.I1_R_sclebtn1, self.I1_R_sclebtn10]
        self.I1_R_sclebtn1.setChecked(True)
        self.I1_R_upbtn     = QPushButton(">")
        self.I1_R_downbtn   = QPushButton("<")
        self.I1_R_Block_Layout.addWidget(self.I1_R_sclebtn10,1,0)
        self.I1_R_Block_Layout.addWidget(self.I1_R_sclebtn1,2,0)
        self.I1_R_Block_Layout.addWidget(self.I1_R_sclebtn01,3,0)
        self.I1_R_Block_Layout.addWidget(self.I1_R_upbtn,1,1)
        self.I1_R_Block_Label = QLabel(f"deg")
        self.I1_R_Block_Layout.addWidget(self.I1_R_Block_Label,2,1,alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I1_R_Block_Layout.addWidget(self.I1_R_downbtn,3,1)
        self.I1_R_upbtn.clicked.connect(self.I1_R_UpBtn_Clicked)
        self.I1_R_downbtn.clicked.connect(self.I1_R_DownBtn_Clicked)
        # I1 Rotation Plot
        self.I1_Rotation_Motor_Plot = pg.PlotWidget()
        self.I1_R_Block_Layout.addWidget(self.I1_Rotation_Motor_Plot,0,2,5,5)
        self.I1_Rotation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I1_Rotation_Motor_Plot.setLabel('left', "[deg]", **self.label_style)
        self.I1_Rotation_Motor_Plot.setLabel('bottom', "Time [s]", **self.label_style)
        self.I1_Rotation_Motor_Plot.addLegend()
        self.I1_Rotation_Motor_Plot.setBackground('w')
        self.I1_Rotation_Motor_Plot.showGrid(x=True, y=True)
        self.I1_R_cmd_line          = self.I1_Rotation_Motor_Plot.plot(self.t_buffer, self.I1_R_Motor_posref_buff, name = "Command", pen = self.blue)
        self.I1_R_Motor_pos_line    = self.I1_Rotation_Motor_Plot.plot(self.t_buffer, self.I1_R_Motor_pos_buff, name = "Motor", pen = self.red)
        self.I1_R_Effector_pos_line = self.I1_Rotation_Motor_Plot.plot(self.t_buffer, self.I1_R_Effector_pos_buff, name = "Catheter", pen = self.green)
        
        # Adding the control blocks into the instrument block
        self.Instrument_1_Block_Layout.addWidget(self.I1_T_Block)
        self.Instrument_1_Block_Layout.addWidget(self.I1_R_Block)

    def Create_DMC_Instrument_2_Block(self):
        ## Instrument 2 Block
        self.Instrument_2_Block  = QGroupBox("Instrument 2")
        self.Instrument_2_Block.setAlignment(Qt.AlignCenter)
        self.Instrument_2_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Instrument_2_Block_Layout = QVBoxLayout()
        self.Instrument_2_Block.setLayout(self.Instrument_2_Block_Layout)
        self.Instrument_2_Block.setEnabled(False)
        # I2 Translation Block
        self.I2_T_Block  = QGroupBox("Insertion / Withdrawal")
        self.I2_T_Block.setAlignment(Qt.AlignCenter)
        self.I2_T_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I2_T_Block_Layout = QGridLayout()
        self.I2_T_Block.setLayout(self.I2_T_Block_Layout)
        self.I2_T_Block.setEnabled(True)
        self.I2_T_sclebtn10 = QRadioButton("x 10")
        self.I2_T_sclebtn1  = QRadioButton("x 1")
        self.I2_T_sclebtn01 = QRadioButton("x 0.1")
        self.I2_T_Radio_Buttons = [self.I2_T_sclebtn01, self.I2_T_sclebtn1, self.I2_T_sclebtn10]
        self.I2_T_sclebtn1.setChecked(True)
        self.I2_T_upbtn     = QPushButton("^")
        self.I2_T_downbtn   = QPushButton("v")
        self.I2_T_Block_Layout.addWidget(self.I2_T_sclebtn10,1,0)
        self.I2_T_Block_Layout.addWidget(self.I2_T_sclebtn1,2,0)
        self.I2_T_Block_Layout.addWidget(self.I2_T_sclebtn01,3,0)
        self.I2_T_Block_Layout.addWidget(self.I2_T_upbtn,1,1)
        self.I2_T_Block_Label = QLabel(f"deg")
        self.I2_T_Block_Layout.addWidget(self.I2_T_Block_Label,2,1,alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I2_T_Block_Layout.addWidget(self.I2_T_downbtn,3,1)
        self.I2_T_upbtn.clicked.connect(self.I2_T_UpBtn_Clicked)
        self.I2_T_downbtn.clicked.connect(self.I2_T_DownBtn_Clicked)
        # I2 Translation Plot
        self.I2_Translation_Motor_Plot = pg.PlotWidget()
        self.I2_T_Block_Layout.addWidget(self.I2_Translation_Motor_Plot,0,2,5,5)
        self.I2_Translation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I2_Translation_Motor_Plot.setLabel('left', "[deg]", **self.label_style)
        self.I2_Translation_Motor_Plot.setLabel('bottom', "Time [s]", **self.label_style)
        self.I2_Translation_Motor_Plot.addLegend()
        self.I2_Translation_Motor_Plot.setBackground('w')
        self.I2_Translation_Motor_Plot.showGrid(x=True, y=True)
        self.I2_T_cmd_line          = self.I2_Translation_Motor_Plot.plot(self.t_buffer, self.I2_T_Motor_posref_buff, name = "Command", pen = self.blue)
        self.I2_T_Motor_pos_line    = self.I2_Translation_Motor_Plot.plot(self.t_buffer, self.I2_T_Motor_pos_buff, name = "Motor", pen = self.red)
        self.I2_T_Effector_pos_line = self.I2_Translation_Motor_Plot.plot(self.t_buffer, self.I2_T_Effector_pos_buff, name = "Catheter", pen = self.green)
        # I2 Rotation Block
        self.I2_R_Block  = QGroupBox("Rotation")
        self.I2_R_Block.setAlignment(Qt.AlignCenter)
        self.I2_R_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I2_R_Block_Layout = QGridLayout()
        self.I2_R_Block.setLayout(self.I2_R_Block_Layout)
        self.I2_R_Block.setEnabled(True)
        self.I2_R_sclebtn10 = QRadioButton("x 10")
        self.I2_R_sclebtn1  = QRadioButton("x 1")
        self.I2_R_sclebtn01 = QRadioButton("x 0.1")
        self.I2_R_Radio_Buttons = [self.I2_R_sclebtn01, self.I2_R_sclebtn1, self.I2_R_sclebtn10]
        self.I2_R_sclebtn1.setChecked(True)
        self.I2_R_upbtn     = QPushButton(">")
        self.I2_R_downbtn   = QPushButton("<")
        self.I2_R_Block_Layout.addWidget(self.I2_R_sclebtn10,1,0)
        self.I2_R_Block_Layout.addWidget(self.I2_R_sclebtn1,2,0)
        self.I2_R_Block_Layout.addWidget(self.I2_R_sclebtn01,3,0)
        self.I2_R_Block_Layout.addWidget(self.I2_R_upbtn,1,1)
        self.I2_R_Block_Label = QLabel(f"deg")
        self.I2_R_Block_Layout.addWidget(self.I2_R_Block_Label,2,1,alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I2_R_Block_Layout.addWidget(self.I2_R_downbtn,3,1)
        self.I2_R_upbtn.clicked.connect(self.I2_R_UpBtn_Clicked)
        self.I2_R_downbtn.clicked.connect(self.I2_R_DownBtn_Clicked)
        # I2 Rotation Plot
        self.I2_Rotation_Motor_Plot = pg.PlotWidget()
        self.I2_R_Block_Layout.addWidget(self.I2_Rotation_Motor_Plot,0,2,5,5)
        self.I2_Rotation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I2_Rotation_Motor_Plot.setLabel('left', "[deg]", **self.label_style)
        self.I2_Rotation_Motor_Plot.setLabel('bottom', "Time [s]", **self.label_style)
        self.I2_Rotation_Motor_Plot.addLegend()
        self.I2_Rotation_Motor_Plot.setBackground('w')
        self.I2_Rotation_Motor_Plot.showGrid(x=True, y=True)
        self.I2_R_cmd_line          = self.I2_Rotation_Motor_Plot.plot(self.t_buffer, self.I2_R_Motor_posref_buff, name = "Command", pen = self.blue)
        self.I2_R_Motor_pos_line    = self.I2_Rotation_Motor_Plot.plot(self.t_buffer, self.I2_R_Motor_pos_buff, name = "Motor", pen = self.red)
        self.I2_R_Effector_pos_line = self.I2_Rotation_Motor_Plot.plot(self.t_buffer, self.I2_R_Effector_pos_buff, name = "Catheter", pen = self.green)
        
        # Adding the control blocks into the instrument block
        self.Instrument_2_Block_Layout.addWidget(self.I2_T_Block)
        self.Instrument_2_Block_Layout.addWidget(self.I2_R_Block)

    def Create_DMC_Instrument_3_Block(self):
        ## Instrument 3 Block
        self.Instrument_3_Block  = QGroupBox("Instrument 3")
        self.Instrument_3_Block.setAlignment(Qt.AlignCenter)
        self.Instrument_3_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.Instrument_3_Block_Layout = QVBoxLayout()
        self.Instrument_3_Block.setLayout(self.Instrument_3_Block_Layout)
        self.Instrument_3_Block.setEnabled(False)
        # I3 Translation Block
        self.I3_T_Block  = QGroupBox("Insertion / Withdrawal")
        self.I3_T_Block.setAlignment(Qt.AlignCenter)
        self.I3_T_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I3_T_Block_Layout = QGridLayout()
        self.I3_T_Block.setLayout(self.I3_T_Block_Layout)
        self.I3_T_Block.setEnabled(True)
        self.I3_T_sclebtn10 = QRadioButton("x 10")
        self.I3_T_sclebtn1  = QRadioButton("x 1")
        self.I3_T_sclebtn01 = QRadioButton("x 0.1")
        self.I3_T_Radio_Buttons = [self.I3_T_sclebtn01, self.I3_T_sclebtn1, self.I3_T_sclebtn10]
        self.I3_T_sclebtn1.setChecked(True)
        self.I3_T_upbtn     = QPushButton("^")
        self.I3_T_downbtn   = QPushButton("v")
        self.I3_T_Block_Layout.addWidget(self.I3_T_sclebtn10,1,0)
        self.I3_T_Block_Layout.addWidget(self.I3_T_sclebtn1,2,0)
        self.I3_T_Block_Layout.addWidget(self.I3_T_sclebtn01,3,0)
        self.I3_T_Block_Layout.addWidget(self.I3_T_upbtn,1,1)
        self.I3_T_Block_Label = QLabel(f"deg")
        self.I3_T_Block_Layout.addWidget(self.I3_T_Block_Label,2,1,alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I3_T_Block_Layout.addWidget(self.I3_T_downbtn,3,1)
        self.I3_T_upbtn.clicked.connect(self.I3_T_UpBtn_Clicked)
        self.I3_T_downbtn.clicked.connect(self.I3_T_DownBtn_Clicked)
        # I3 Translation Plot
        self.I3_Translation_Motor_Plot = pg.PlotWidget()
        self.I3_T_Block_Layout.addWidget(self.I3_Translation_Motor_Plot,0,3,5,5)
        self.I3_Translation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I3_Translation_Motor_Plot.setLabel('left', "[deg]", **self.label_style)
        self.I3_Translation_Motor_Plot.setLabel('bottom', "Time [s]", **self.label_style)
        self.I3_Translation_Motor_Plot.addLegend()
        self.I3_Translation_Motor_Plot.setBackground('w')
        self.I3_Translation_Motor_Plot.showGrid(x=True, y=True)
        self.I3_T_cmd_line          = self.I3_Translation_Motor_Plot.plot(self.t_buffer, self.I3_T_Motor_posref_buff, name = "Command", pen = self.blue)
        self.I3_T_Motor_pos_line    = self.I3_Translation_Motor_Plot.plot(self.t_buffer, self.I3_T_Motor_pos_buff, name = "Motor", pen = self.red)
        self.I3_T_Effector_pos_line = self.I3_Translation_Motor_Plot.plot(self.t_buffer, self.I3_T_Effector_pos_buff, name = "Catheter", pen = self.green)
        # I3 Rotation Block
        self.I3_R_Block  = QGroupBox("Rotation")
        self.I3_R_Block.setAlignment(Qt.AlignCenter)
        self.I3_R_Block.setStyleSheet(f"font-size: {self.overall_fontsize}px;")
        self.I3_R_Block_Layout = QGridLayout()
        self.I3_R_Block.setLayout(self.I3_R_Block_Layout)
        self.I3_R_Block.setEnabled(True)
        self.I3_R_sclebtn10 = QRadioButton("x 10")
        self.I3_R_sclebtn1  = QRadioButton("x 1")
        self.I3_R_sclebtn01 = QRadioButton("x 0.1")
        self.I3_R_Radio_Buttons = [self.I3_R_sclebtn01, self.I3_R_sclebtn1, self.I3_R_sclebtn10]
        self.I3_R_sclebtn1.setChecked(True)
        self.I3_R_upbtn     = QPushButton(">")
        self.I3_R_downbtn   = QPushButton("<")
        self.I3_R_Block_Layout.addWidget(self.I3_R_sclebtn10,1,0)
        self.I3_R_Block_Layout.addWidget(self.I3_R_sclebtn1,2,0)
        self.I3_R_Block_Layout.addWidget(self.I3_R_sclebtn01,3,0)
        self.I3_R_Block_Layout.addWidget(self.I3_R_upbtn,1,1)
        self.I3_R_Block_Label = QLabel(f"deg")
        self.I3_R_Block_Layout.addWidget(self.I3_R_Block_Label,2,1,alignment=Qt.AlignmentFlag.AlignHCenter)
        self.I3_R_Block_Layout.addWidget(self.I3_R_downbtn,3,1)
        self.I3_R_upbtn.clicked.connect(self.I3_R_UpBtn_Clicked)
        self.I3_R_downbtn.clicked.connect(self.I3_R_DownBtn_Clicked)
        # I3 Rotation Plot
        self.I3_Rotation_Motor_Plot = pg.PlotWidget()
        self.I3_R_Block_Layout.addWidget(self.I3_Rotation_Motor_Plot,0,3,5,5)
        self.I3_Rotation_Motor_Plot.setTitle("Angular Position", **self.title_style)
        self.I3_Rotation_Motor_Plot.setLabel('left', "[deg]", **self.label_style)
        self.I3_Rotation_Motor_Plot.setLabel('bottom', "Time [s]", **self.label_style)
        self.I3_Rotation_Motor_Plot.addLegend()
        self.I3_Rotation_Motor_Plot.setBackground('w')
        self.I3_Rotation_Motor_Plot.showGrid(x=True, y=True)
        self.I3_R_cmd_line          = self.I3_Rotation_Motor_Plot.plot(self.t_buffer, self.I3_R_Motor_posref_buff, name = "Command", pen = self.blue)
        self.I3_R_Motor_pos_line    = self.I3_Rotation_Motor_Plot.plot(self.t_buffer, self.I3_R_Motor_pos_buff, name = "Motor", pen = self.red)
        self.I3_R_Effector_pos_line = self.I3_Rotation_Motor_Plot.plot(self.t_buffer, self.I3_R_Effector_pos_buff, name = "Catheter", pen = self.green)
        
        # Adding the control blocks into the instrument block
        self.Instrument_3_Block_Layout.addWidget(self.I3_T_Block)
        self.Instrument_3_Block_Layout.addWidget(self.I3_R_Block)
    
    # Instrument 1 Buttons
    def I1_T_UpBtn_Clicked(self):
        self.I1_T_Motor_posref += 100
        # self.Rotation_Button_Clicked = FalseS
        # if not self.Control_Mode_Flag:
        #     self.I1_T_Motor_posref = self.Compute_Increment(self.I1_T_Motor_posref, self.I1_T_Radio_Buttons)
        # else:
        #     self.I1_T_Effector_posref = self.Compute_Increment(self.I1_T_Effector_posref, self.I1_T_Radio_Buttons)
        #     self.I1_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I1_T_Effector_posref)
        self.Transmit_data()

    def I1_T_DownBtn_Clicked(self):
        self.I1_T_Motor_posref -= 100
        # self.Rotation_Button_Clicked = False
        # if not self.Control_Mode_Flag:
        #     self.I1_T_Motor_posref = self.Compute_Decrement(self.I1_T_Motor_posref, self.I1_T_Radio_Buttons)
        # else:
        #     self.I1_T_Effector_posref = self.Compute_Decrement(self.I1_T_Effector_posref, self.I1_T_Radio_Buttons)
        #     self.I1_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I1_T_Effector_posref)
        self.Transmit_data()

    def I1_R_UpBtn_Clicked(self):
        self.CW_Rotation = True
        self.CCW_Rotation = False
        self.Rotation_Button_Clicked = True
        if not self.Control_Mode_Flag:
            self.I1_R_Motor_posref = self.Compute_Increment(self.I1_R_Motor_posref, self.I1_R_Radio_Buttons)
        else:
            self.I1_R_Effector_posref = self.Compute_Increment(self.I1_R_Effector_posref, self.I1_R_Radio_Buttons)
            self.I1_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I1_R_Effector_posref)
        #self.I1_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I1_R_Motor_posref, self.I1_T_Motor_posref, self.I1_R_Radio_Buttons) # This is to compensate the coupling
        self.Transmit_data()

    def I1_R_DownBtn_Clicked(self):
        self.CW_Rotation = False
        self.CCW_Rotation = True
        self.Rotation_Button_Clicked = True
        if not self.Control_Mode_Flag:
            self.I1_R_Motor_posref = self.Compute_Decrement(self.I1_R_Motor_posref, self.I1_R_Radio_Buttons)
        else:
            self.I1_R_Effector_posref = self.Compute_Decrement(self.I1_R_Effector_posref, self.I1_R_Radio_Buttons)
            self.I1_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I1_R_Effector_posref)
        #self.I1_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I1_R_Motor_posref, self.I1_T_Motor_posref, self.I1_R_Radio_Buttons) # This is to compensate the coupling
        self.Transmit_data()
    
    # Instrument 2 Buttons
    def I2_T_UpBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if not self.Control_Mode_Flag:
            self.I2_T_Motor_posref = self.Compute_Increment(self.I2_T_Motor_posref, self.I2_T_Radio_Buttons)
        else:
            self.I2_T_Effector_posref = self.Compute_Increment(self.I2_T_Effector_posref, self.I2_T_Radio_Buttons)
            self.I2_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I2_T_Effector_posref)
        self.Transmit_data()

    def I2_T_DownBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if not self.Control_Mode_Flag:
            self.I2_T_Motor_posref = self.Compute_Decrement(self.I2_T_Motor_posref, self.I2_T_Radio_Buttons)
        else:
            self.I2_T_Effector_posref = self.Compute_Decrement(self.I2_T_Effector_posref, self.I2_T_Radio_Buttons)
            self.I2_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I2_T_Effector_posref)
        self.Transmit_data()

    def I2_R_UpBtn_Clicked(self):
        self.CW_Rotation = True
        self.CCW_Rotation = False
        self.Rotation_Button_Clicked = True
        if not self.Control_Mode_Flag:
            self.I2_R_Motor_posref = self.Compute_Increment(self.I2_R_Motor_posref, self.I2_R_Radio_Buttons)
        else:
            self.I2_R_Effector_posref = self.Compute_Increment(self.I2_R_Effector_posref, self.I2_R_Radio_Buttons)
            self.I2_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I2_R_Effector_posref)
        #self.I2_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I2_R_Motor_posref, self.I2_T_Motor_posref, self.I2_R_Radio_Buttons) # This is to compensate the coupling
        self.Transmit_data()

    def I2_R_DownBtn_Clicked(self):
        self.CW_Rotation = False
        self.CCW_Rotation = True
        self.Rotation_Button_Clicked = True
        if not self.Control_Mode_Flag:
            self.I2_R_Motor_posref = self.Compute_Decrement(self.I2_R_Motor_posref, self.I2_R_Radio_Buttons)
        else:
            self.I2_R_Effector_posref = self.Compute_Decrement(self.I2_R_Effector_posref, self.I2_R_Radio_Buttons)
            self.I2_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I2_R_Effector_posref)
        #self.I2_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I2_R_Motor_posref, self.I2_T_Motor_posref, self.I2_R_Radio_Buttons) # This is to compensate the coupling
        self.Transmit_data()

    # Instrument 3 Buttons
    def I3_T_UpBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if not self.Control_Mode_Flag:
            self.I3_T_Motor_posref = self.Compute_Increment(self.I3_T_Motor_posref, self.I3_T_Radio_Buttons)
        else:
            self.I3_T_Effector_posref = self.Compute_Increment(self.I3_T_Effector_posref, self.I3_T_Radio_Buttons)
            self.I3_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I3_T_Effector_posref)
        self.Transmit_data()

    def I3_T_DownBtn_Clicked(self):
        self.Rotation_Button_Clicked = False
        if not self.Control_Mode_Flag:
            self.I3_T_Motor_posref = self.Compute_Decrement(self.I3_T_Motor_posref, self.I3_T_Radio_Buttons)
        else:
            self.I3_T_Effector_posref = self.Compute_Decrement(self.I3_T_Effector_posref, self.I3_T_Radio_Buttons)
            self.I3_T_Motor_posref    = self.Translation_Transform_Catheter_to_M2(self.I3_T_Effector_posref)
        self.Transmit_data()

    def I3_R_UpBtn_Clicked(self):
        self.CW_Rotation = True
        self.CCW_Rotation = False
        self.Rotation_Button_Clicked = True
        if not self.Control_Mode_Flag:
            self.I3_R_Motor_posref = self.Compute_Increment(self.I3_R_Motor_posref, self.I3_R_Radio_Buttons)
        else:
            self.I3_R_Effector_posref = self.Compute_Increment(self.I3_R_Effector_posref, self.I3_R_Radio_Buttons)
            self.I3_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I3_R_Effector_posref)
        #self.I3_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I3_R_Motor_posref, self.I3_T_Motor_posref, self.I3_R_Radio_Buttons) # This is to compensate the coupling
        self.Transmit_data()

    def I3_R_DownBtn_Clicked(self):
        self.CW_Rotation = False
        self.CCW_Rotation = True
        self.Rotation_Button_Clicked = True
        if not self.Control_Mode_Flag:
            self.I3_R_Motor_posref = self.Compute_Decrement(self.I3_R_Motor_posref, self.I3_R_Radio_Buttons)
        else:
            self.I3_R_Effector_posref = self.Compute_Decrement(self.I3_R_Effector_posref, self.I3_R_Radio_Buttons)
            self.I3_R_Motor_posref = self.Rotation_Transform_Catheter_to_M1(self.I3_R_Effector_posref)
        #self.I3_T_Motor_posref = self.Decouple_Rotation_Translation_M1_to_M2(self.I3_R_Motor_posref, self.I3_T_Motor_posref, self.I3_R_Radio_Buttons) # This is to compensate the coupling
        self.Transmit_data()

    # Transformations
    def Rotation_Transform_M1_to_Catheter(self, theta_M1): # Given Motor 1 angular position (responsible for rotation) computes the anglular position & displacement of the catheter
        theta_Catheter = (self.r_G1/self.r_G2)*theta_M1
        return theta_Catheter

    def Rotation_Transform_Catheter_to_M1(self, theta_Catheter): # Given Motor 1 angular position (responsible for rotation) computes the anglular position & displacement of the catheter
        theta_M1 = theta_Catheter/(self.r_G1/self.r_G2)
        return theta_M1
    
    def Translation_Transform_M2_to_Catheter(self, theta_M2): # Given Motor 2 angular position (responsible for rotation) computes the linear position & displacement of the catheter
        Catheter_Translation = pi*self.d_Roller*(self.r_G3/self.r_G4)*self.deg2rad(theta_M2)
        return Catheter_Translation
    
    def Translation_Transform_Catheter_to_M2(self, Catheter_Translation): # Given Motor 2 angular position (responsible for rotation) computes the linear position & displacement of the catheter
        # Catheter_Translation = pi*self.d_Roller*(self.r_G3/self.r_G4)*theta_M2
        theta_M2 = self.rad2deg(Catheter_Translation/(pi*self.d_Roller*(self.r_G3/self.r_G4)))
        return theta_M2
    
    def Decouple_Rotation_Translation_M1_to_M2(self, theta_M1, theta_M2, radio_buttons):
        if self.CW_Rotation:
            # cmd_M2 = theta_M2 - (self.r_G1/self.r_G2)*(theta_M1 - self.Compute_Decrement(theta_M1, radio_buttons))
            cmd_M2 = theta_M2 - self.Rotation_Transform_M1_to_Catheter((theta_M1 - self.Compute_Decrement(theta_M1, radio_buttons)))
        else:
            cmd_M2 = theta_M2 - self.Rotation_Transform_M1_to_Catheter((theta_M1 - self.Compute_Increment(theta_M1, radio_buttons)))
        return cmd_M2
        

    def Compute_Increment(self, current_cmd, radio_buttons):
        if radio_buttons[0].isChecked():
            Displacement_cmd = current_cmd + 0.1*10
            return Displacement_cmd
        elif radio_buttons[2].isChecked():
            Displacement_cmd = current_cmd + 10*10
            return Displacement_cmd
        else:
            Displacement_cmd = current_cmd + 1*10
            return Displacement_cmd

    def Compute_Decrement(self, current_cmd, radio_buttons):
        if radio_buttons[0].isChecked():
            Displacement_cmd = current_cmd - 0.1*10
            return Displacement_cmd
        elif radio_buttons[2].isChecked():
            Displacement_cmd = current_cmd - 10*10
            return Displacement_cmd
        else:
            Displacement_cmd = current_cmd - 1*10
            return Displacement_cmd


    def Direct_Motor_Ctrl_mode(self):
        self.Instrument_1_Block.setEnabled(True)
        self.Instrument_2_Block.setEnabled(True)
        self.Instrument_3_Block.setEnabled(True)
        self.Control_Mode_Flag = 0

    def Task_Space_Ctrl_mode(self):
        self.Instrument_1_Block.setEnabled(True)
        self.Instrument_2_Block.setEnabled(True)
        self.Instrument_3_Block.setEnabled(True)
        self.Control_Mode_Flag  = 1

    def CatheterV1_Task_Ctrl_mode(self):
        """
        当选中 'catheter v1 task control' 时执行:
        - 隐藏/禁用旧的 Blocks
        - 显示 CatheterV1_Block
        """
        if self.CtrlCathV1_Rbutton.isChecked():
            # 隐藏旧的
            # self.Instrument_1_Block.setVisible(False)
            # self.Instrument_2_Block.setVisible(False)
            # self.Instrument_3_Block.setVisible(False)
            
            # 显示新的
            self.CatheterV1_Block.setVisible(True)

        else:
            # 如果反选了，就把 v1 界面隐藏
            self.CatheterV1_Block.setVisible(False)


    def update_buffers(self, buffers, values):
        for buffer, value in zip(buffers, values):
            buffer[:] = buffer[1:]  # This modifies the original buffer
            buffer.append(value)
 

    def saturation(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def deg2rad(self,deg):
        return deg*pi/180
    
    def rad2deg(self,deg):
        return deg*180/pi


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    Window = MainWindow()
    Window.show()
    sys.exit(app.exec_())