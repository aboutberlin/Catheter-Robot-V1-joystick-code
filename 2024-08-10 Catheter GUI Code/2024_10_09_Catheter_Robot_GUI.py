import serial
from serial.tools import list_ports
# from Slicer_Automate_Pipeline import automate_workflow
import struct
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
import json
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

fps = 120
Plots_RefreshRate = int(1000/fps) 
Teensy_freq_ble   = 120 
plot_time_window  = 10 
buffer_size  = plot_time_window * Teensy_freq_ble 

t_buffer = list([0] *buffer_size)
M1_Pos_buffer     = t_buffer.copy()
M2_Pos_buffer     = t_buffer.copy()
M1_Pos_cmd_buffer = t_buffer.copy()
M2_Pos_cmd_buffer = t_buffer.copy()
M3_Pos_buffer     = t_buffer.copy()
M3_Pos_cmd_buffer = t_buffer.copy()
M4_Pos_buffer     = t_buffer.copy()
M4_Pos_cmd_buffer = t_buffer.copy()
M5_Pos_buffer     = t_buffer.copy()
M5_Pos_cmd_buffer = t_buffer.copy()

M1_Pos = 0
M2_Pos = 0
M1_pos_cmd = 0
M2_pos_cmd = 0

M3_Pos = 0
M4_Pos = 0
M5_Pos = 0
M3_pos_cmd = 0
M4_pos_cmd = 0
M5_pos_cmd = 0

Pos_LA_1 =0
Pos_LA_2 = 0

LA1_Pos = 0
LA2_Pos = 0

LA1_SpinBox = 0
LA2_SpinBox = 0

pos_cmd     = 0
sw_bias     = 0
sw_amp      = 0
sw_freq     = 0
step_amp    = 0
step_t_high = 0
step_t_low  = 0

t_0    = 0 # Initial time
t_prev = 0 # Previuos time
t      = 0 # Current time
t_teensy = 0 # Time in the Teensy 4.1

Connection_Flag    = False
LogginButton_Flag  = False
Data_Received_Flag = False
first_teensy_time  = True

M_Selected           = 0
CtrlMode_Selected    = 0
Signal_Mode_Selected = 0
M1_gID = 0
M2_gID = 0
M3_gID = 0
M4_gID = 0
M5_gID = 0

M1_Selected = 0
M2_Selected = 0
M3_Selected = 0
M4_Selected = 0
M5_Selected = 0

red  = pg.mkPen(color=(255, 0, 0), width = 2)
blue = pg.mkPen(color=(0, 0, 255), width = 2)


class MainWindow(QWidget):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        global t_buffer, M1_Pos_buffer, M2_Pos_buffer, M3_Pos_buffer, M4_Pos_buffer, M5_Pos_buffer,\
            M1_Pos_cmd_buffer, M2_Pos_cmd_buffer, M3_Pos_cmd_buffer, M4_Pos_cmd_buffer, M5_Pos_cmd_buffer,\
            t_0, t_prev,\
            ConnectButton, LoggingButton, SerialComboBox,\
            Connection_Flag, connected_ports,\
            CtrlPos_Rbutton, CtrlVel_Rbutton, CtrlTor_Rbutton, CtrlImp_Rbutton,\
            Cmd_text, Control_Mode, Motors, Tabs,\
            Tab_M1, Tab_M2, Tab_M3, Tab_M4, Tab_M5,\
            M1_Constant_cmd_block, M1_Sinwave_cmd_block, M1_Step_cmd_block, M1_Slider,\
            M2_Constant_cmd_block, M2_Sinwave_cmd_block, M2_Step_cmd_block, M2_Slider,\
            M3_Constant_cmd_block, M3_Sinwave_cmd_block, M3_Step_cmd_block, M3_Slider,\
            M4_Constant_cmd_block, M4_Sinwave_cmd_block, M4_Step_cmd_block, M4_Slider,\
            M5_Constant_cmd_block, M5_Sinwave_cmd_block, M5_Step_cmd_block, M5_Slider,\
            LA1_Slider, LA1_SpinBox, \
            LA2_Slider, LA2_SpinBox, \
            sw_bias, sw_amp, sw_freq,\
            Motor1_Checkbox, Motor2_Checkbox, Motor3_Checkbox, Motor4_Checkbox, Motor5_Checkbox,\
            M1_sw_bias, M1_sw_amp, M1_sw_freq, M1_step_stepsize, M1_step_period, M1_step_dutycycle,\
            M2_sw_bias, M2_sw_amp, M2_sw_freq, M2_step_stepsize, M2_step_period, M2_step_dutycycle,\
            M3_sw_bias, M3_sw_amp, M3_sw_freq, M3_step_stepsize, M3_step_period, M3_step_dutycycle,\
            M4_sw_bias, M4_sw_amp, M4_sw_freq, M4_step_stepsize, M4_step_period, M4_step_dutycycle,\
            M5_sw_bias, M5_sw_amp, M5_sw_freq, M5_step_stepsize, M5_step_period, M5_step_dutycycle
                
        self.setWindowTitle('Catheter Robot GUI')
        
        connected_ports = find_available_ports()

        # Layout def
        MainLayout      = QHBoxLayout() # Window Layout
        Ctrls_Layout    = QVBoxLayout() # Layout for the controls on the left side of the window
        Comm_Layout     = QHBoxLayout() # Layout for the COM pot selection and the Log data button
        Select_Layout   = QVBoxLayout() # Main Layout for the selection of motor and control mode
        Motors_Layout   = QHBoxLayout() # Motor selection Layout
        CtrlMode_Layout = QVBoxLayout() # Control mode Layout
        # LA_Cmd_Layout  =  QHBoxLayout()
        Cmd_Layout      = QHBoxLayout() # Layout for sending commands
        Plot_Layout     = QVBoxLayout() # Layout for the Plots (Position, Velocity, and Torque)

        #Main Layout
        MainLayout.addLayout(Ctrls_Layout, stretch=3)
        MainLayout.addLayout(Plot_Layout, stretch=2)

        # Sublayouts
        Ctrls_Layout.addLayout(Comm_Layout, stretch=1)
        Ctrls_Layout.addLayout(Select_Layout, stretch=1)
        Ctrls_Layout.addLayout(Cmd_Layout, stretch=6)

        # Set the Main window layout
        self.setLayout(MainLayout)

        # Ctrls_Layout
        ConnectButton  = QPushButton("Connect")
        SerialComboBox = QComboBox()
        LoggingButton  = QPushButton("Data Logging")

        Comm_Layout.addWidget(QLabel("ComPort:"))
        Comm_Layout.addWidget(SerialComboBox)
        SerialComboBox.addItems(connected_ports)
        Comm_Layout.addWidget(ConnectButton)
        Comm_Layout.addWidget(LoggingButton)

        # Select_Layout objects
        Control_Mode = QGroupBox("Control Mode")
        Motors       = QGroupBox("Motor Control Panel")
        # Control Modes
        CtrlPos_Rbutton = QRadioButton("Position Control")
        CtrlVel_Rbutton = QRadioButton("Velocity Control")
        CtrlTor_Rbutton = QRadioButton("Torque Control")
        CtrlImp_Rbutton = QRadioButton("Impedance Control")
        Run_Pipeline_Button = QPushButton("Run Pipeline")
        Run_Pipeline_Button.clicked.connect(lambda: run_pipeline())

        # Control Modes
        Control_Mode.setLayout(CtrlMode_Layout)
        Select_Layout.addWidget(Control_Mode)
        CtrlMode_Layout.addWidget(CtrlPos_Rbutton)
        CtrlMode_Layout.addWidget(CtrlVel_Rbutton)
        CtrlMode_Layout.addWidget(CtrlTor_Rbutton)
        CtrlMode_Layout.addWidget(CtrlImp_Rbutton)
        CtrlMode_Layout.addWidget(Run_Pipeline_Button)

        # Motors
        Tabs   = QTabWidget()
        Tab_M1 = QWidget() 
        Tab_M2 = QWidget() 
        Tab_M3 = QWidget() 
        Tab_M4 = QWidget() 
        Tab_M5 = QWidget() 
        
        Motors.setLayout(Motors_Layout)
        Cmd_Layout.addWidget(Motors)
        
        Motors_Layout.addWidget(Tab_M1)
        Motors_Layout.addWidget(Tab_M2)
        Motors_Layout.addWidget(Tab_M3)
        Motors_Layout.addWidget(Tab_M4)
        Motors_Layout.addWidget(Tab_M5)

        #########################################################################
        ############################ Motor 1 Tab ################################
        #########################################################################
       
        M1_Tab_Layout = QVBoxLayout()
        Tab_M1.setLayout(M1_Tab_Layout) 

        Motor1_Checkbox = QCheckBox("Guidewire Rotation")
        M1_Tab_Layout.addWidget(Motor1_Checkbox)

        M1_Constant_cmd_block = QGroupBox("Guidewire Rotation Constant Command")
        M1_Constant_cmd_block_Layout = QGridLayout()
        M1_Constant_cmd_block.setLayout(M1_Constant_cmd_block_Layout)
        M1_Constant_cmd_block.setCheckable(True)
        M1_Constant_cmd_block.setChecked(False)
        M1_Tab_Layout.addWidget(M1_Constant_cmd_block)

        M1_Slider = QSlider(Qt.Horizontal)        
        M1_Constant_cmd_block_Layout.addWidget(M1_Slider,2,1,2,3)
        M1_Slider.setMinimum(0)
        M1_Slider.setMaximum(180)
        M1_Slider.setValue(0)
        M1_Slider.setTickPosition(QSlider.TicksBelow)
        M1_Slider.setTickInterval(1)
        M1_Slider.valueChanged.connect(M1_Slider_ValueChange)

        M1_SpinBox = QDoubleSpinBox()
        M1_SpinBox.setMinimum(0)
        M1_SpinBox.setMaximum(180)
        M1_SpinBox.setValue(0)
        M1_SpinBox.setSingleStep(1)
        M1_Constant_cmd_block_Layout.addWidget(M1_SpinBox, 2, 4)

        M1_Slider.valueChanged.connect(lambda value: M1_SpinBox.setValue(int(value)))
        M1_SpinBox.valueChanged.connect(lambda value: M1_Slider.setValue(int(value)))
              
        # Sinwave command block
        M1_Sinwave_cmd_block = QGroupBox("Guidewire Rotation Sinwave")
        M1_Sinwave_cmd_block_Layout = QGridLayout()
        M1_Sinwave_cmd_block.setLayout(M1_Sinwave_cmd_block_Layout)
        M1_Sinwave_cmd_block.setCheckable(True)
        M1_Sinwave_cmd_block.setChecked(False)
        M1_Tab_Layout.addWidget(M1_Sinwave_cmd_block)
        # Sinwave parameters selectors
        M1_sw_bias = QDoubleSpinBox()
        M1_sw_amp  = QDoubleSpinBox()
        M1_sw_freq = QDoubleSpinBox()
        M1_sw_bias.setValue(0)
        M1_sw_amp.setValue(0)
        M1_sw_freq.setValue(0)
        M1_sw_bias.valueChanged.connect(M1_Sinwave)
        M1_sw_amp.valueChanged.connect(M1_Sinwave)
        M1_sw_freq.valueChanged.connect(M1_Sinwave)
        M1_sw_bias_lb = QLabel("Bias (deg)")
        M1_sw_amp_lb  = QLabel("Amplitude (deg)")
        M1_sw_freq_lb = QLabel("Frequency (Hz)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_bias_lb, sw_label_row, 1)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_amp_lb, sw_label_row, 2)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_freq_lb, sw_label_row, 3)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_bias, sw_ctrls_row, 1)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_amp, sw_ctrls_row, 2)
        M1_Sinwave_cmd_block_Layout.addWidget(M1_sw_freq, sw_ctrls_row, 3)

        # Step command block
        M1_Step_cmd_block        = QGroupBox("Guidewire Rotation Step")
        M1_Step_cmd_block_Layout = QGridLayout()
        M1_Step_cmd_block.setLayout(M1_Step_cmd_block_Layout)
        M1_Step_cmd_block.setCheckable(True)
        M1_Step_cmd_block.setChecked(False)
        M1_Tab_Layout.addWidget(M1_Step_cmd_block)
        # Step parameters selectors
        M1_step_stepsize  = QDoubleSpinBox()
        M1_step_period    = QDoubleSpinBox()
        M1_step_dutycycle = QDoubleSpinBox()
        M1_step_stepsize.setValue(0)
        M1_step_period.setValue(0.01)
        M1_step_dutycycle.setValue(0)
        M1_step_stepsize.valueChanged.connect(M1_Step)
        M1_step_period.valueChanged.connect(M1_Step)
        M1_step_dutycycle.valueChanged.connect(M1_Step)
        M1_step_stepsize_lb  = QLabel("Step Size (deg)")
        M1_step_period_lb    = QLabel("Period (s)")
        M1_step_dutycycle_lb = QLabel("Duty Cycle (%)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M1_Step_cmd_block_Layout.addWidget(M1_step_stepsize_lb, sw_label_row, 1)
        M1_Step_cmd_block_Layout.addWidget(M1_step_period_lb, sw_label_row, 2)
        M1_Step_cmd_block_Layout.addWidget(M1_step_dutycycle_lb, sw_label_row, 3)
        M1_Step_cmd_block_Layout.addWidget(M1_step_stepsize, sw_ctrls_row, 1)
        M1_Step_cmd_block_Layout.addWidget(M1_step_period, sw_ctrls_row, 2)
        M1_Step_cmd_block_Layout.addWidget(M1_step_dutycycle, sw_ctrls_row, 3)

        #########################################################################
        ############################ Motor 2 Tab ################################
        #########################################################################
        
        M2_Tab_Layout = QVBoxLayout()
        Tab_M2.setLayout(M2_Tab_Layout)

        Motor2_Checkbox = QCheckBox("Guidewire Linear")
        M2_Tab_Layout.addWidget(Motor2_Checkbox)

        M2_Constant_cmd_block = QGroupBox("Guidewire Linear Constant Command")
        M2_Constant_cmd_block_Layout = QGridLayout()
        M2_Constant_cmd_block.setLayout(M2_Constant_cmd_block_Layout)
        M2_Constant_cmd_block.setCheckable(True)
        M2_Constant_cmd_block.setChecked(False)
        M2_Tab_Layout.addWidget(M2_Constant_cmd_block)

        M2_Slider = QSlider(Qt.Horizontal)        
        M2_Constant_cmd_block_Layout.addWidget(M2_Slider,2,1,2,3)
        M2_Slider.setMinimum(0)
        M2_Slider.setMaximum(180)
        M2_Slider.setValue(0)
        M2_Slider.setTickPosition(QSlider.TicksBelow)
        M2_Slider.setTickInterval(1)
        M2_Slider.valueChanged.connect(M2_Slider_ValueChange)

        M2_SpinBox = QDoubleSpinBox()
        M2_SpinBox.setMinimum(0)
        M2_SpinBox.setMaximum(180)
        M2_SpinBox.setValue(0)
        M2_SpinBox.setSingleStep(1)
        M2_Constant_cmd_block_Layout.addWidget(M2_SpinBox, 2, 4)

        M2_Slider.valueChanged.connect(lambda value: M2_SpinBox.setValue(int(value)))
        M2_SpinBox.valueChanged.connect(lambda value: M2_Slider.setValue(int(value)))

        # Sinwave command block
        M2_Sinwave_cmd_block = QGroupBox("Guidewire Linear Sinwave")
        M2_Sinwave_cmd_block_Layout = QGridLayout()
        M2_Sinwave_cmd_block.setLayout(M2_Sinwave_cmd_block_Layout)
        M2_Sinwave_cmd_block.setCheckable(True)
        M2_Sinwave_cmd_block.setChecked(False)
        M2_Tab_Layout.addWidget(M2_Sinwave_cmd_block)
        # Sinwave parameters selectors
        M2_sw_bias = QDoubleSpinBox()
        M2_sw_amp  = QDoubleSpinBox()
        M2_sw_freq = QDoubleSpinBox()
        M2_sw_bias.setValue(0)
        M2_sw_amp.setValue(0)
        M2_sw_freq.setValue(0)
        M2_sw_bias.valueChanged.connect(M2_Sinwave)
        M2_sw_amp.valueChanged.connect(M2_Sinwave)
        M2_sw_freq.valueChanged.connect(M2_Sinwave)
        M2_sw_bias_lb = QLabel("Bias (deg)")
        M2_sw_amp_lb  = QLabel("Amplitude (deg)")
        M2_sw_freq_lb = QLabel("Frequency (Hz)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_bias_lb, sw_label_row, 1)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_amp_lb, sw_label_row, 2)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_freq_lb, sw_label_row, 3)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_bias, sw_ctrls_row, 1)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_amp, sw_ctrls_row, 2)
        M2_Sinwave_cmd_block_Layout.addWidget(M2_sw_freq, sw_ctrls_row, 3)

        # Step command block
        M2_Step_cmd_block        = QGroupBox("Guidewire Linear Step")
        M2_Step_cmd_block_Layout = QGridLayout()
        M2_Step_cmd_block.setLayout(M2_Step_cmd_block_Layout)
        M2_Step_cmd_block.setCheckable(True)
        M2_Step_cmd_block.setChecked(False)
        M2_Tab_Layout.addWidget(M2_Step_cmd_block)
        # Step parameters selectors
        M2_step_stepsize  = QDoubleSpinBox()
        M2_step_period    = QDoubleSpinBox()
        M2_step_dutycycle = QDoubleSpinBox()
        M2_step_stepsize.setValue(0)
        M2_step_period.setValue(0.01)
        M2_step_dutycycle.setValue(0)
        M2_step_stepsize.valueChanged.connect(M2_Step)
        M2_step_period.valueChanged.connect(M2_Step)
        M2_step_dutycycle.valueChanged.connect(M2_Step)
        M2_step_stepsize_lb  = QLabel("Step Size (deg)")
        M2_step_period_lb    = QLabel("Period (s)")
        M2_step_dutycycle_lb = QLabel("Duty Cycle (%)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M2_Step_cmd_block_Layout.addWidget(M2_step_stepsize_lb, sw_label_row, 1)
        M2_Step_cmd_block_Layout.addWidget(M2_step_period_lb, sw_label_row, 2)
        M2_Step_cmd_block_Layout.addWidget(M2_step_dutycycle_lb, sw_label_row, 3)
        M2_Step_cmd_block_Layout.addWidget(M2_step_stepsize, sw_ctrls_row, 1)
        M2_Step_cmd_block_Layout.addWidget(M2_step_period, sw_ctrls_row, 2)
        M2_Step_cmd_block_Layout.addWidget(M2_step_dutycycle, sw_ctrls_row, 3)
        
        #########################################################################
        ############################ Motor 3 Tab ################################
        #########################################################################
        
        M3_Tab_Layout = QVBoxLayout()
        Tab_M3.setLayout(M3_Tab_Layout)

        Motor3_Checkbox = QCheckBox("Microcatheter Linear")
        M3_Tab_Layout.addWidget(Motor3_Checkbox)

        M3_Constant_cmd_block = QGroupBox("Microcatheter Linear Constant Command")
        M3_Constant_cmd_block_Layout = QGridLayout()
        M3_Constant_cmd_block.setLayout(M3_Constant_cmd_block_Layout)
        M3_Constant_cmd_block.setCheckable(True)
        M3_Constant_cmd_block.setChecked(False)
        M3_Tab_Layout.addWidget(M3_Constant_cmd_block)

        M3_Slider = QSlider(Qt.Horizontal)        
        M3_Constant_cmd_block_Layout.addWidget(M3_Slider,2,1,2,3)
        M3_Slider.setMinimum(0)
        M3_Slider.setMaximum(180)
        M3_Slider.setValue(0)
        M3_Slider.setTickPosition(QSlider.TicksBelow)
        M3_Slider.setTickInterval(1)
        M3_Slider.valueChanged.connect(M3_Slider_ValueChange)

        M3_SpinBox = QSpinBox()       
        M3_SpinBox.setMinimum(0)
        M3_SpinBox.setMaximum(180)
        M3_SpinBox.setValue(0)
        M3_SpinBox.setSingleStep(1)
        M3_Constant_cmd_block_Layout.addWidget(M3_SpinBox, 2, 4)

        M3_Slider.valueChanged.connect(lambda value: M3_SpinBox.setValue(int(value)))
        M3_SpinBox.valueChanged.connect(lambda value: M3_Slider.setValue(int(value)))

        # Sinwave command block
        M3_Sinwave_cmd_block = QGroupBox("Microcatheter Linear Sinwave")
        M3_Sinwave_cmd_block_Layout = QGridLayout()
        M3_Sinwave_cmd_block.setLayout(M3_Sinwave_cmd_block_Layout)
        M3_Sinwave_cmd_block.setCheckable(True)
        M3_Sinwave_cmd_block.setChecked(False)
        M3_Tab_Layout.addWidget(M3_Sinwave_cmd_block)
        # Sinwave parameters selectors
        M3_sw_bias = QDoubleSpinBox()
        M3_sw_amp  = QDoubleSpinBox()
        M3_sw_freq = QDoubleSpinBox()
        M3_sw_bias.setValue(0)
        M3_sw_amp.setValue(0)
        M3_sw_freq.setValue(0)
        M3_sw_bias.valueChanged.connect(M3_Sinwave)
        M3_sw_amp.valueChanged.connect(M3_Sinwave)
        M3_sw_freq.valueChanged.connect(M3_Sinwave)
        M3_sw_bias_lb = QLabel("Bias (deg)")
        M3_sw_amp_lb  = QLabel("Amplitude (deg)")
        M3_sw_freq_lb = QLabel("Frequency (Hz)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M3_Sinwave_cmd_block_Layout.addWidget(M3_sw_bias_lb, sw_label_row, 1)
        M3_Sinwave_cmd_block_Layout.addWidget(M3_sw_amp_lb, sw_label_row, 2)
        M3_Sinwave_cmd_block_Layout.addWidget(M3_sw_freq_lb, sw_label_row, 3)
        M3_Sinwave_cmd_block_Layout.addWidget(M3_sw_bias, sw_ctrls_row, 1)
        M3_Sinwave_cmd_block_Layout.addWidget(M3_sw_amp, sw_ctrls_row, 2)
        M3_Sinwave_cmd_block_Layout.addWidget(M3_sw_freq, sw_ctrls_row, 3)

        # Step command block
        M3_Step_cmd_block        = QGroupBox("Microcatheter Linear Step")
        M3_Step_cmd_block_Layout = QGridLayout()
        M3_Step_cmd_block.setLayout(M3_Step_cmd_block_Layout)
        M3_Step_cmd_block.setCheckable(True)
        M3_Step_cmd_block.setChecked(False)
        M3_Tab_Layout.addWidget(M3_Step_cmd_block)
        # Step parameters selectors
        M3_step_stepsize  = QDoubleSpinBox()
        M3_step_period    = QDoubleSpinBox()
        M3_step_dutycycle = QDoubleSpinBox()
        M3_step_stepsize.setValue(0)
        M3_step_period.setValue(0.01)
        M3_step_dutycycle.setValue(0)
        M3_step_stepsize.valueChanged.connect(M3_Step)
        M3_step_period.valueChanged.connect(M3_Step)
        M3_step_dutycycle.valueChanged.connect(M3_Step)
        M3_step_stepsize_lb  = QLabel("Step Size (deg)")
        M3_step_period_lb    = QLabel("Period (s)")
        M3_step_dutycycle_lb = QLabel("Duty Cycle (%)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M3_Step_cmd_block_Layout.addWidget(M3_step_stepsize_lb, sw_label_row, 1)
        M3_Step_cmd_block_Layout.addWidget(M3_step_period_lb, sw_label_row, 2)
        M3_Step_cmd_block_Layout.addWidget(M3_step_dutycycle_lb, sw_label_row, 3)
        M3_Step_cmd_block_Layout.addWidget(M3_step_stepsize, sw_ctrls_row, 1)
        M3_Step_cmd_block_Layout.addWidget(M3_step_period, sw_ctrls_row, 2)
        M3_Step_cmd_block_Layout.addWidget(M3_step_dutycycle, sw_ctrls_row, 3)

        #########################################################################
        ############################ Motor 4 Tab ################################
        #########################################################################
        
        M4_Tab_Layout = QVBoxLayout()
        Tab_M4.setLayout(M4_Tab_Layout)
        
        Motor4_Checkbox = QCheckBox("Guide Catheter Linear")
        M4_Tab_Layout.addWidget(Motor4_Checkbox)
        
        M4_Constant_cmd_block = QGroupBox("Guide Catheter Linear Constant Command")
        M4_Constant_cmd_block_Layout = QGridLayout()
        M4_Constant_cmd_block.setLayout(M4_Constant_cmd_block_Layout)
        M4_Constant_cmd_block.setCheckable(True)
        M4_Constant_cmd_block.setChecked(False)
        M4_Tab_Layout.addWidget(M4_Constant_cmd_block)

        M4_Slider = QSlider(Qt.Horizontal)        
        M4_Constant_cmd_block_Layout.addWidget(M4_Slider,2,1,2,3)
        M4_Slider.setMinimum(0)
        M4_Slider.setMaximum(180)
        M4_Slider.setValue(0)
        M4_Slider.setTickPosition(QSlider.TicksBelow)
        M4_Slider.setTickInterval(1)
        M4_Slider.valueChanged.connect(M4_Slider_ValueChange)

        M4_SpinBox = QSpinBox()       
        M4_SpinBox.setMinimum(0)
        M4_SpinBox.setMaximum(180)
        M4_SpinBox.setValue(0)
        M4_SpinBox.setSingleStep(1)
        M4_Constant_cmd_block_Layout.addWidget(M4_SpinBox, 2, 4)
   
        M4_Slider.valueChanged.connect(lambda value: M4_SpinBox.setValue(int(value)))
        M4_SpinBox.valueChanged.connect(lambda value: M4_Slider.setValue(int(value)))

        # Sinwave command block
        M4_Sinwave_cmd_block = QGroupBox("Guide Catheter Linear Sinwave")
        M4_Sinwave_cmd_block_Layout = QGridLayout()
        M4_Sinwave_cmd_block.setLayout(M4_Sinwave_cmd_block_Layout)
        M4_Sinwave_cmd_block.setCheckable(True)
        M4_Sinwave_cmd_block.setChecked(False)
        M4_Tab_Layout.addWidget(M4_Sinwave_cmd_block)
        # Sinwave parameters selectors
        M4_sw_bias = QDoubleSpinBox()
        M4_sw_amp  = QDoubleSpinBox()
        M4_sw_freq = QDoubleSpinBox()
        M4_sw_bias.setValue(0)
        M4_sw_amp.setValue(0)
        M4_sw_freq.setValue(0)
        M4_sw_bias.valueChanged.connect(M4_Sinwave)
        M4_sw_amp.valueChanged.connect(M4_Sinwave)
        M4_sw_freq.valueChanged.connect(M4_Sinwave)
        M4_sw_bias_lb = QLabel("Bias (deg)")
        M4_sw_amp_lb  = QLabel("Amplitude (deg)")
        M4_sw_freq_lb = QLabel("Frequency (Hz)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M4_Sinwave_cmd_block_Layout.addWidget(M4_sw_bias_lb, sw_label_row, 1)
        M4_Sinwave_cmd_block_Layout.addWidget(M4_sw_amp_lb, sw_label_row, 2)
        M4_Sinwave_cmd_block_Layout.addWidget(M4_sw_freq_lb, sw_label_row, 3)
        M4_Sinwave_cmd_block_Layout.addWidget(M4_sw_bias, sw_ctrls_row, 1)
        M4_Sinwave_cmd_block_Layout.addWidget(M4_sw_amp, sw_ctrls_row, 2)
        M4_Sinwave_cmd_block_Layout.addWidget(M4_sw_freq, sw_ctrls_row, 3)

        # Step command block
        M4_Step_cmd_block        = QGroupBox("Guide Catheter Linear Step")
        M4_Step_cmd_block_Layout = QGridLayout()
        M4_Step_cmd_block.setLayout(M4_Step_cmd_block_Layout)
        M4_Step_cmd_block.setCheckable(True)
        M4_Step_cmd_block.setChecked(False)
        M4_Tab_Layout.addWidget(M4_Step_cmd_block)
        # Step parameters selectors
        M4_step_stepsize  = QDoubleSpinBox()
        M4_step_period    = QDoubleSpinBox()
        M4_step_dutycycle = QDoubleSpinBox()
        M4_step_stepsize.setValue(0)
        M4_step_period.setValue(0.01)
        M4_step_dutycycle.setValue(0)
        M4_step_stepsize.valueChanged.connect(M4_Step)
        M4_step_period.valueChanged.connect(M4_Step)
        M4_step_dutycycle.valueChanged.connect(M4_Step)
        M4_step_stepsize_lb  = QLabel("Step Size (deg)")
        M4_step_period_lb    = QLabel("Period (s)")
        M4_step_dutycycle_lb = QLabel("Duty Cycle (%)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M4_Step_cmd_block_Layout.addWidget(M4_step_stepsize_lb, sw_label_row, 1)
        M4_Step_cmd_block_Layout.addWidget(M4_step_period_lb, sw_label_row, 2)
        M4_Step_cmd_block_Layout.addWidget(M4_step_dutycycle_lb, sw_label_row, 3)
        M4_Step_cmd_block_Layout.addWidget(M4_step_stepsize, sw_ctrls_row, 1)
        M4_Step_cmd_block_Layout.addWidget(M4_step_period, sw_ctrls_row, 2)
        M4_Step_cmd_block_Layout.addWidget(M4_step_dutycycle, sw_ctrls_row, 3)

        #########################################################################
        ############################ Motor 5 Tab ################################
        #########################################################################
        
        M5_Tab_Layout = QVBoxLayout()
        Tab_M5.setLayout(M5_Tab_Layout)

        Motor5_Checkbox = QCheckBox("Guide Catheter Rotation")
        M5_Tab_Layout.addWidget(Motor5_Checkbox)

        M5_Constant_cmd_block = QGroupBox("Guide Catheter Rotation Constant Command")
        M5_Constant_cmd_block_Layout = QGridLayout()
        M5_Constant_cmd_block.setLayout(M5_Constant_cmd_block_Layout)
        M5_Constant_cmd_block.setCheckable(True)
        M5_Constant_cmd_block.setChecked(False)
        M5_Tab_Layout.addWidget(M5_Constant_cmd_block)

        M5_Slider = QSlider(Qt.Horizontal)        
        M5_Constant_cmd_block_Layout.addWidget(M5_Slider,2,1,2,3)
        M5_Slider.setMinimum(0)
        M5_Slider.setMaximum(180)
        M5_Slider.setValue(0)
        M5_Slider.setTickPosition(QSlider.TicksBelow)
        M5_Slider.setTickInterval(1)
        M5_Slider.valueChanged.connect(M5_Slider_ValueChange)
        M5_SpinBox = QSpinBox()        
        M5_SpinBox.setMinimum(0)
        M5_SpinBox.setMaximum(180)
        M5_SpinBox.setValue(0)
        M5_SpinBox.setSingleStep(1)
        M5_Constant_cmd_block_Layout.addWidget(M5_SpinBox, 2, 4)

        M5_Slider.valueChanged.connect(lambda value: M5_SpinBox.setValue(int(value)))
        M5_SpinBox.valueChanged.connect(lambda value: M5_Slider.setValue(int(value)))

        # Sinwave command block
        M5_Sinwave_cmd_block = QGroupBox("Guide Catheter Rotation Sinwave")
        M5_Sinwave_cmd_block_Layout = QGridLayout()
        M5_Sinwave_cmd_block.setLayout(M5_Sinwave_cmd_block_Layout)
        M5_Sinwave_cmd_block.setCheckable(True)
        M5_Sinwave_cmd_block.setChecked(False)
        M5_Tab_Layout.addWidget(M5_Sinwave_cmd_block)
        # Sinwave parameters selectors
        M5_sw_bias = QDoubleSpinBox()
        M5_sw_amp  = QDoubleSpinBox()
        M5_sw_freq = QDoubleSpinBox()
        M5_sw_bias.setValue(0)
        M5_sw_amp.setValue(0)
        M5_sw_freq.setValue(0)
        M5_sw_bias.valueChanged.connect(M5_Sinwave)
        M5_sw_amp.valueChanged.connect(M5_Sinwave)
        M5_sw_freq.valueChanged.connect(M5_Sinwave)
        M5_sw_bias_lb = QLabel("Bias (deg)")
        M5_sw_amp_lb  = QLabel("Amplitude (deg)")
        M5_sw_freq_lb = QLabel("Frequency (Hz)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M5_Sinwave_cmd_block_Layout.addWidget(M5_sw_bias_lb, sw_label_row, 1)
        M5_Sinwave_cmd_block_Layout.addWidget(M5_sw_amp_lb, sw_label_row, 2)
        M5_Sinwave_cmd_block_Layout.addWidget(M5_sw_freq_lb, sw_label_row, 3)
        M5_Sinwave_cmd_block_Layout.addWidget(M5_sw_bias, sw_ctrls_row, 1)
        M5_Sinwave_cmd_block_Layout.addWidget(M5_sw_amp, sw_ctrls_row, 2)
        M5_Sinwave_cmd_block_Layout.addWidget(M5_sw_freq, sw_ctrls_row, 3)

        # Step command block
        M5_Step_cmd_block        = QGroupBox("Guide Catheter Rotation Step")
        M5_Step_cmd_block_Layout = QGridLayout()
        M5_Step_cmd_block.setLayout(M5_Step_cmd_block_Layout)
        M5_Step_cmd_block.setCheckable(True)
        M5_Step_cmd_block.setChecked(False)
        M5_Tab_Layout.addWidget(M5_Step_cmd_block)
        # Step parameters selectors
        M5_step_stepsize  = QDoubleSpinBox()
        M5_step_period    = QDoubleSpinBox()
        M5_step_dutycycle = QDoubleSpinBox()
        M5_step_stepsize.setValue(0)
        M5_step_period.setValue(0.01)
        M5_step_dutycycle.setValue(0)
        M5_step_stepsize.valueChanged.connect(M5_Step)
        M5_step_period.valueChanged.connect(M5_Step)
        M5_step_dutycycle.valueChanged.connect(M5_Step)
        M5_step_stepsize_lb  = QLabel("Step Size (deg)")
        M5_step_period_lb    = QLabel("Period (s)")
        M5_step_dutycycle_lb = QLabel("Duty Cycle (%)")
        sw_label_row = 1
        sw_ctrls_row = sw_label_row + 1
        M5_Step_cmd_block_Layout.addWidget(M5_step_stepsize_lb, sw_label_row, 1)
        M5_Step_cmd_block_Layout.addWidget(M5_step_period_lb, sw_label_row, 2)
        M5_Step_cmd_block_Layout.addWidget(M5_step_dutycycle_lb, sw_label_row, 3)
        M5_Step_cmd_block_Layout.addWidget(M5_step_stepsize, sw_ctrls_row, 1)
        M5_Step_cmd_block_Layout.addWidget(M5_step_period, sw_ctrls_row, 2)
        M5_Step_cmd_block_Layout.addWidget(M5_step_dutycycle, sw_ctrls_row, 3)

        #################################################################################
        ####################### Linear Actuator 1 #######################################
        #################################################################################

        LA1_Label = QLabel("Linear Actuator 1")
        CtrlMode_Layout.addWidget(LA1_Label)

        LA1_Checkbox = QCheckBox("Enable Linear Actuator 1")
        CtrlMode_Layout.addWidget(LA1_Checkbox)

        LA1_Slider = QSlider(Qt.Horizontal)
        LA1_Slider.setMinimum(11)
        LA1_Slider.setMaximum(91)
        LA1_Slider.setValue(11)
        LA1_Slider.setTickPosition(QSlider.TicksBelow)
        LA1_Slider.setTickInterval(10)
        LA1_Slider.valueChanged.connect(LA1_Slider_ValueChange)

        LA1_SpinBox = QSpinBox()
        LA1_SpinBox.setMinimum(11)
        LA1_SpinBox.setMaximum(91)
        LA1_SpinBox.setValue(11)
        LA1_SpinBox.setSingleStep(10)

        LA1_Slider.valueChanged.connect(lambda value: LA1_SpinBox.setValue(int(value)))
        LA1_SpinBox.valueChanged.connect(lambda value: LA1_Slider.setValue(int(value)))
        
        CtrlMode_Layout.addWidget(LA1_Slider)
        CtrlMode_Layout.addWidget(LA1_SpinBox)    

        #################################################################################
        ####################### Linear Actuator 2 #######################################
        #################################################################################
        
        LA2_Label  = QLabel("Linear Actuator 2")
        LA2_Checkbox = QCheckBox("Enable Linear Actuator 2")
        LA2_Slider = QSlider(Qt.Horizontal)
        LA2_Slider.setMinimum(11)
        LA2_Slider.setMaximum(91)
        LA2_Slider.setValue(11)
        LA2_Slider.setTickPosition(QSlider.TicksBelow)
        LA2_Slider.setTickInterval(10)
        LA2_Slider.valueChanged.connect(LA2_Slider_ValueChange)
        LA2_SpinBox = QSpinBox()
        LA2_SpinBox.setMinimum(11)
        LA2_SpinBox.setMaximum(91)
        LA2_SpinBox.setSingleStep(10)
        

        LA2_Slider.valueChanged.connect(lambda value: LA2_SpinBox.setValue(int(value)))
        LA2_SpinBox.valueChanged.connect(lambda value: LA2_Slider.setValue(int(value)))

        CtrlMode_Layout.addWidget(LA2_Label)
        CtrlMode_Layout.addWidget(LA2_Checkbox)
        CtrlMode_Layout.addWidget(LA2_Slider)
        CtrlMode_Layout.addWidget(LA2_SpinBox)     

        M1_Pos_Plot = pg.PlotWidget()
        M2_Pos_Plot = pg.PlotWidget()
        M3_Pos_Plot = pg.PlotWidget()
        M4_Pos_Plot = pg.PlotWidget()
        M5_Pos_Plot = pg.PlotWidget()

        Plot_Layout.addWidget(M1_Pos_Plot)
        Plot_Layout.addWidget(M2_Pos_Plot)
        Plot_Layout.addWidget(M3_Pos_Plot)
        Plot_Layout.addWidget(M4_Pos_Plot)
        Plot_Layout.addWidget(M5_Pos_Plot)

        ConnectButton.clicked.connect(Connect_Clicked)
        LoggingButton.clicked.connect(LogginButton_Clicked)

        label_style = {"font-size": "16px"}
        title_style = {"color": "black", "font-size": "20px"}
        
        M1_Pos_buffer = M1_Pos_buffer* 1
        M1_Pos_cmd_buffer = M1_Pos_cmd_buffer* 1

        M1_Pos_Plot.setTitle("Guidewire Angular Position", **title_style)
        M1_Pos_Plot.setLabel('left', "[deg]", **label_style)
        M1_Pos_Plot.setLabel('bottom', "Time [s]", **label_style)
        M1_Pos_Plot.addLegend()
        M1_Pos_Plot.setBackground('w')
        M1_Pos_Plot.showGrid(x=True, y=True)
        self.M1_pos_line     = M1_Pos_Plot.plot(t_buffer, M1_Pos_buffer, name = "Actual", pen = red)
        self.M1_pos_cmd_line = M1_Pos_Plot.plot(t_buffer, M1_Pos_cmd_buffer, name = "Command", pen = blue)
        
        M2_Pos_buffer = M2_Pos_buffer
        M2_Pos_cmd_buffer = M2_Pos_cmd_buffer
        
        M2_Pos_Plot.setTitle("Guidewire Linear Position", **title_style)
        M2_Pos_Plot.setLabel('left', "[deg]", **label_style)
        M2_Pos_Plot.setLabel('bottom', "Time [s]", **label_style)
        M2_Pos_Plot.addLegend()
        M2_Pos_Plot.setBackground('w')
        M2_Pos_Plot.showGrid(x=True, y=True)
        self.M2_pos_line     = M2_Pos_Plot.plot(t_buffer, M2_Pos_buffer, name = "Actual", pen = red)
        self.M2_pos_cmd_line = M2_Pos_Plot.plot(t_buffer, M2_Pos_cmd_buffer, name = "Command", pen = blue)
        
        M3_Pos_buffer = M3_Pos_buffer
        M3_Pos_cmd_buffer = M3_Pos_cmd_buffer
        
        M3_Pos_Plot.setTitle("Microcatheter Linear Position", **title_style)
        M3_Pos_Plot.setLabel('left', "[deg]", **label_style)
        M3_Pos_Plot.setLabel('bottom', "Time [s]", **label_style)
        M3_Pos_Plot.addLegend()
        M3_Pos_Plot.setBackground('w')
        M3_Pos_Plot.showGrid(x=True, y=True)
        self.M3_pos_line     = M3_Pos_Plot.plot(t_buffer, M3_Pos_buffer, name="Actual", pen=red)
        self.M3_pos_cmd_line = M3_Pos_Plot.plot(t_buffer, M3_Pos_cmd_buffer, name="Command", pen=blue)

        M4_Pos_buffer = M4_Pos_buffer
        M4_Pos_cmd_buffer = M4_Pos_cmd_buffer
        
        M4_Pos_Plot.setTitle("Guide Catheter Linear Position", **title_style)
        M4_Pos_Plot.setLabel('left', "[deg]", **label_style)
        M4_Pos_Plot.setLabel('bottom', "Time [s]", **label_style)
        M4_Pos_Plot.addLegend()
        M4_Pos_Plot.setBackground('w')
        M4_Pos_Plot.showGrid(x=True, y=True)
        self.M4_pos_line     = M4_Pos_Plot.plot(t_buffer, M4_Pos_buffer, name="Actual", pen=red)
        self.M4_pos_cmd_line = M4_Pos_Plot.plot(t_buffer, M4_Pos_cmd_buffer, name="Command", pen=blue)

        M5_Pos_buffer = M5_Pos_buffer* 1
        M5_Pos_cmd_buffer = M5_Pos_cmd_buffer* 1
        
        M5_Pos_Plot.setTitle("Guide Catheter Angular Position", **title_style)
        M5_Pos_Plot.setLabel('left', "[deg]", **label_style)
        M5_Pos_Plot.setLabel('bottom', "Time [s]", **label_style)
        M5_Pos_Plot.addLegend()
        M5_Pos_Plot.setBackground('w')
        M5_Pos_Plot.showGrid(x=True, y=True)
        self.M5_pos_line     = M5_Pos_Plot.plot(t_buffer, M5_Pos_buffer, name="Actual", pen=red)
        self.M5_pos_cmd_line = M5_Pos_Plot.plot(t_buffer, M5_Pos_cmd_buffer, name="Command", pen=blue)

        # Creation of the timer for executing the function
        self.timer = QtCore.QTimer()
        self.timer.setInterval(Plots_RefreshRate)
        self.timer.timeout.connect(self.all) 
        self.timer.start()

    def all(self):
        global Connection_Flag, Data_Received_Flag, M_Selected, CtrlMode_Selected,\
        M1_Selected, M2_Selected, M3_Selected, M4_Selected, M5_Selected

        General_State_Check()
        
        if Connection_Flag:
            if ser.in_waiting > 0:
                ConnectButton.setText("Receiving")
                ConnectButton.setStyleSheet("background-color : green")
                Recieve_data()
                if Data_Received_Flag:
                    self.update_plot_data()
                    Data_Received_Flag = False
                   
    def update_plot_data(self):
        global t_buffer, M1_Pos_buffer, M2_Pos_buffer, M3_Pos_buffer, M4_Pos_buffer, M5_Pos_buffer, \
            M1_Pos_cmd_buffer, M2_Pos_cmd_buffer, M3_Pos_cmd_buffer, M4_Pos_cmd_buffer, M5_Pos_cmd_buffer, \
            M1_Pos, M2_Pos, M3_Pos, M4_Pos, M5_Pos, \
            M1_pos_cmd, M2_pos_cmd, M3_pos_cmd, M4_pos_cmd, M5_pos_cmd, \
            t_0, t_prev, t_teensy, t_0_teensy, \
            Connection_Flag, LogginButton_Flag, \
            csv_file_name, DataHeaders, pos_cmd

        if Connection_Flag == True:

            t = time.time() - t_0

            if t_prev != t:

                delta_t = t_teensy - t_0_teensy
                buffers = [t_buffer, M1_Pos_buffer, M1_Pos_cmd_buffer, M2_Pos_buffer, M2_Pos_cmd_buffer, 
                       M3_Pos_buffer, M3_Pos_cmd_buffer, M4_Pos_buffer, M4_Pos_cmd_buffer, 
                       M5_Pos_buffer, M5_Pos_cmd_buffer]
                values  = [delta_t, M1_Pos, M1_pos_cmd, M2_Pos, M2_pos_cmd, 
                       M3_Pos, M3_pos_cmd, M4_Pos, M4_pos_cmd, 
                       M5_Pos, M5_pos_cmd]
                update_buffers(buffers, values)

                self.M1_pos_cmd_line.setData(t_buffer, M1_Pos_cmd_buffer)
                self.M1_pos_line.setData(t_buffer, M1_Pos_buffer)

                self.M2_pos_cmd_line.setData(t_buffer, M2_Pos_cmd_buffer)
                self.M2_pos_line.setData(t_buffer, M2_Pos_buffer)
                
                self.M3_pos_cmd_line.setData(t_buffer, M3_Pos_cmd_buffer)
                self.M3_pos_line.setData(t_buffer, M3_Pos_buffer)

                self.M4_pos_cmd_line.setData(t_buffer, M4_Pos_cmd_buffer)
                self.M4_pos_line.setData(t_buffer, M4_Pos_buffer)

                self.M5_pos_cmd_line.setData(t_buffer, M5_Pos_cmd_buffer)
                self.M5_pos_line.setData(t_buffer, M5_Pos_buffer)

                if LogginButton_Flag == True:
                    LoggedData = {
                        "time": t,                        
                        "M1_Pos_cmd": M1_pos_cmd,
                        "M1_Pos": M1_Pos,
                        "M2_Pos_cmd": M2_pos_cmd,
                        "M2_Pos": M2_Pos,
                        "M3_Pos_cmd": M3_pos_cmd,
                        "M3_Pos": M3_Pos,
                        "M4_Pos_cmd": M4_pos_cmd,
                        "M4_Pos": M4_Pos,
                        "M5_Pos_cmd": M5_pos_cmd,
                        "M5_Pos": M5_Pos
                    }

                    with open(csv_file_name, mode="a", newline="") as file:
                        writer = csv.DictWriter(file, fieldnames = DataHeaders)
                        writer.writerow(LoggedData)
            t_prev = t
        else:
            print("NOT Connected")

def M1_Slider_ValueChange():
        global M1_Slider, M1_pos_cmd, pos_cmd

        if M1_pos_cmd < -360 or M1_pos_cmd > 360:
            showdialog()
            M1_pos_cmd = saturation(M1_pos_cmd, -360, 360)

        M1_pos_cmd = int(M1_Slider.value())
        
        Transmit_data()


def M2_Slider_ValueChange():
        global M2_Slider, M2_pos_cmd, pos_cmd

        if M2_pos_cmd < -360 or M2_pos_cmd > 360:
            showdialog()
            M2_pos_cmd = saturation(M2_pos_cmd, -360, 360)

        M2_pos_cmd = int(M2_Slider.value())
        Transmit_data()

def M3_Slider_ValueChange():
    global M3_Slider, M3_pos_cmd, pos_cmd

    if M3_pos_cmd < -360 or M3_pos_cmd > 360:
        showdialog()
        M3_pos_cmd = saturation(M3_pos_cmd, -360, 360)

    M3_pos_cmd = int(M3_Slider.value())
    
    Transmit_data()


def M4_Slider_ValueChange():
    global M4_Slider, M4_pos_cmd, pos_cmd

    if M4_pos_cmd < -360 or M4_pos_cmd > 360:
        showdialog()
        M4_pos_cmd = saturation(M4_pos_cmd, -360, 360)

    M4_pos_cmd = int(M4_Slider.value())
    
    Transmit_data()


def M5_Slider_ValueChange():
    global M5_Slider, M5_pos_cmd, pos_cmd

    if M5_pos_cmd < -360 or M5_pos_cmd > 360:
        showdialog()
        M5_pos_cmd = saturation(M5_pos_cmd, -360, 360)

    M5_pos_cmd = int(M5_Slider.value())
    
    Transmit_data()
    
def LA1_Slider_ValueChange():
    global LA1_Slider, Pos_LA_1
    
    if Pos_LA_1 < 11 or Pos_LA_1 > 91:
        Pos_LA_1 = saturation(Pos_LA_1, 11, 91)
    
    Pos_LA_1 = int(LA1_Slider.value())
    
    Transmit_data()

def LA2_Slider_ValueChange():
    global LA2_Slider, Pos_LA_2
    
    if Pos_LA_2 < 11 or Pos_LA_2 > 91:
        Pos_LA_2 = saturation(Pos_LA_2, 11, 91)
    
    Pos_LA_2 = int(LA2_Slider.value())
    
    Transmit_data()

def M1_Sinwave():
    global M1_sw_bias, M1_sw_amp, M1_sw_freq,\
            sw_bias, sw_amp, sw_freq
    
    sw_bias = int(M1_sw_bias.value())
    sw_amp  = int(M1_sw_amp.value())
    sw_freq = int(M1_sw_freq.value() * 100)

    Transmit_data()


def M2_Sinwave():
    global M2_sw_bias, M2_sw_amp, M2_sw_freq,\
            sw_bias, sw_amp, sw_freq
    
    sw_bias = int(M2_sw_bias.value())
    sw_amp  = int(M2_sw_amp.value())
    sw_freq = int(M2_sw_freq.value() * 100)

    Transmit_data()


def M3_Sinwave():
    global M3_sw_bias, M3_sw_amp, M3_sw_freq,\
            sw_bias, sw_amp, sw_freq
    
    sw_bias = int(M3_sw_bias.value())
    sw_amp  = int(M3_sw_amp.value())
    sw_freq = int(M3_sw_freq.value() * 100)

    Transmit_data()


def M4_Sinwave():
    global M4_sw_bias, M4_sw_amp, M4_sw_freq,\
            sw_bias, sw_amp, sw_freq
    
    sw_bias = int(M4_sw_bias.value())
    sw_amp  = int(M4_sw_amp.value())
    sw_freq = int(M4_sw_freq.value() * 100)

    Transmit_data()


def M5_Sinwave():
    global M5_sw_bias, M5_sw_amp, M5_sw_freq,\
            sw_bias, sw_amp, sw_freq
    
    sw_bias = int(M5_sw_bias.value())
    sw_amp  = int(M5_sw_amp.value())
    sw_freq = int(M5_sw_freq.value() * 100)

    Transmit_data()


def M1_Step():
    global M1_step_stepsize, M1_step_period, M1_step_dutycycle,\
            step_amp, step_t_high, step_t_low
    
    step_amp    = int(M1_step_stepsize.value())
    step_t_high = int(M1_step_period.value())
    step_t_low  = int(M1_step_dutycycle.value())

    Transmit_data()


def M2_Step():
    global M2_step_stepsize, M2_step_period, M2_step_dutycycle,\
            step_amp, step_t_high, step_t_low
    
    step_amp    = int(M2_step_stepsize.value())
    step_t_high = int(M2_step_period.value())
    step_t_low  = int(M2_step_dutycycle.value())

    Transmit_data()


def M3_Step():
    global M3_step_stepsize, M3_step_period, M3_step_dutycycle,\
            step_amp, step_t_high, step_t_low
    
    step_amp    = int(M3_step_stepsize.value())
    step_t_high = int(M3_step_period.value())
    step_t_low  = int(M3_step_dutycycle.value())

    Transmit_data()


def M4_Step():
    global M4_step_stepsize, M4_step_period, M4_step_dutycycle,\
            step_amp, step_t_high, step_t_low
    
    step_amp    = int(M4_step_stepsize.value())
    step_t_high = int(M4_step_period.value())
    step_t_low  = int(M4_step_dutycycle.value())

    Transmit_data()


def M5_Step():
    global M5_step_stepsize, M5_step_period, M5_step_dutycycle,\
            step_amp, step_t_high, step_t_low
    
    step_amp    = int(M5_step_stepsize.value())
    step_t_high = int(M5_step_period.value())
    step_t_low  = int(M5_step_dutycycle.value())

    Transmit_data()
        


def Connect_Clicked():
        global ConnectButton, ser, Connection_Flag, t_0, ble_datalength, data_length, decoded_data, rs232_datalength

        ble_datalength   = 32 
        rs232_datalength = 20 
        data_length      = ble_datalength - 3
        decoded_data     = [0]*data_length

        #### Setting up the serial communication port and baudrate ###
        serial_port = SerialComboBox.currentText()
        baud_rate   = 115200

        ### Stablish the serial connection ###
        ser = serial.Serial(port=serial_port, baudrate=baud_rate)
        ser.timeout = 0

        if ser.is_open:
            print(f"Connected to port: {serial_port} at baud rate: {baud_rate}")
            ConnectButton.setText("Bluetooth activated")
            ConnectButton.setStyleSheet("background-color : yellow")
            Connection_Flag = True

            t_0 = time.time()
            
        else:
            print('Failed to open serial port')

        while not ser.is_open:
            print('Serial port closed')

def Recieve_data():
        global  ser, ble_datalength, data_length, decoded_data, Data_Received_Flag,\
                t_teensy, M1_Pos, M2_Pos, M3_Pos, M4_Pos, M5_Pos,\
                LA1_Pos, LA2_Pos, \
                t_0_teensy, first_teensy_time
        
        if ser.in_waiting >= ble_datalength:
            if ser.read(1) == b'\xA5':  # 165 in uint8
                if ser.read(1) == b'\x5A':  # 90 in uint8
                    expected_length = ser.read(1) 
                    if expected_length == bytes([ble_datalength]):
                        if ser.in_waiting >= data_length:  
                            coded_data = ser.read(data_length)
                            #decoded_data = [0] * (data_length // 2)  
                            decode_i = 0
                            for i in range(1, data_length, 2):
                                var = coded_data[i-1] + coded_data[i] * 256
                                var = (var - 65536) / 100.0 if var > 32767 else var / 100.0
                                decoded_data[decode_i] = var
                                decode_i += 1
                        
                            t_teensy = decoded_data[0]
                            M1_Pos   = decoded_data[1] 
                            M2_Pos   = decoded_data[2]
                            M3_Pos   = decoded_data[3]
                            M4_Pos   = decoded_data[4]
                            M5_Pos   = decoded_data[5]
                            LA1_Pos  = decoded_data[6]
                            LA2_Pos  = decoded_data[7]
                            print(str(LA1_Pos) + " | " + str(LA2_Pos) + " | " + str(M1_Pos))
                            Data_Received_Flag = True
                            
                            if first_teensy_time:
                                t_0_teensy = t_teensy
                                first_teensy_time = False


def Transmit_data():
    global rs232_datalength, data_package, ser, pos_cmd, CtrlMode_Selected, M_Selected, Signal_Mode_Selected,\
        pos_cmd, sw_bias, sw_amp, sw_freq, step_amp, step_t_high, step_t_low,\
        M1_gID, M2_gID, M3_gID, M4_gID, M5_gID,\
        M1_Selected, M2_Selected, M3_Selected, M4_Selected, M5_Selected,\
        Pos_LA_1, Pos_LA_2
    
    data_package = bytearray([165, 90, rs232_datalength, M1_gID, M2_gID, M3_gID, M4_gID, M5_gID, M1_pos_cmd, M2_pos_cmd, M3_pos_cmd, M4_pos_cmd, M5_pos_cmd, sw_bias, sw_amp, sw_freq, Pos_LA_1, Pos_LA_2]) 
                            

    if ser.is_open:
        ser.write(data_package)
        # print(str(M1_pos_cmd) + " | " + str(M2_pos_cmd) + " | " + str(M3_pos_cmd) + " | " + str(M4_pos_cmd) + " | " + str(M5_pos_cmd) + "|" + str(Pos_LA_1) + "|" + str(Pos_LA_2))

def LogginButton_Clicked():
    global LogginButton_Flag, LoggingButton, csv_file_name, DataHeaders, t_0

    LogginButton_Flag = True

    t_0 = time.time()
    LoggingButton.setText("Logging data")
    LoggingButton.setStyleSheet("background-color : blue")
    csv_file_name = "GUI_Logger_" + time.strftime("%Y-%m-%d_%H-%M-%S") + ".csv"
    DataHeaders   = ["time", "M1_Pos", "M2_Pos", "M3_Pos", "M4_Pos", "M5_Pos"]

    with open(csv_file_name, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames = DataHeaders)
        writer.writeheader()

def General_State_Check():
    global CtrlPos_Rbutton, CtrlVel_Rbutton, CtrlTor_Rbutton, CtrlImp_Rbutton,\
        M_Selected, CtrlMode_Selected, Signal_Mode_Selected, \
        M1_Selected, M2_Selected, M3_Selected, M4_Selected, M5_Selected,\
        Control_Mode, Motors, Tabs,\
        M1_Constant_cmd_block, M1_Sinwave_cmd_block, M1_Step_cmd_block,\
        M1_gID, M2_gID, M3_gID, M4_gID, M5_gID,\
        Tab_M1, Tab_M2, Tab_M3, Tab_M4, Tab_M5,\
        Motor1_Checkbox, Motor2_Checkbox, Motor3_Checkbox, Motor4_Checkbox, Motor5_Checkbox,\
        M1_Selected, M2_Selected, M3_Selected, M4_Selected, M5_Selected,\
        M3_Constant_cmd_block, M3_Sinwave_cmd_block, M3_Step_cmd_block,\
        M4_Constant_cmd_block, M4_Sinwave_cmd_block, M4_Step_cmd_block,\
        M5_Constant_cmd_block, M5_Sinwave_cmd_block, M5_Step_cmd_block,\
        M1_Selected, M2_Selected, M3_Selected, M4_Selected, M5_Selected
    
    if (CtrlPos_Rbutton.isChecked() == True) or (CtrlVel_Rbutton.isChecked() == True) or (CtrlTor_Rbutton.isChecked() == True) or (CtrlImp_Rbutton.isChecked()):
        Motors.setEnabled(True)
    else:
        Motors.setEnabled(False)

# Function to identify and assign gIDs for all selected motors
    # Motor 1
    if Motor1_Checkbox.isChecked():
        if M1_Constant_cmd_block.isChecked():
            M1_gID = 111
        elif M1_Sinwave_cmd_block.isChecked():
            M1_gID = 121
        elif M1_Step_cmd_block.isChecked():
            M1_gID = 131
        
    # Motor 2
    if Motor2_Checkbox.isChecked():
        if M2_Constant_cmd_block.isChecked():
            M2_gID = 112
        elif M2_Sinwave_cmd_block.isChecked():
            M2_gID = 122
        elif M2_Step_cmd_block.isChecked():
            M2_gID = 132
        
    # Motor 3
    if Motor3_Checkbox.isChecked():
        if M3_Constant_cmd_block.isChecked():
            M3_gID = 113
        elif M3_Sinwave_cmd_block.isChecked():
            M3_gID = 123
        elif M3_Step_cmd_block.isChecked():
            M3_gID = 133
        
    # Motor 4
    if Motor4_Checkbox.isChecked():
        if M4_Constant_cmd_block.isChecked():
            M4_gID = 114
        elif M4_Sinwave_cmd_block.isChecked():
            M4_gID = 124
        elif M4_Step_cmd_block.isChecked():
            M4_gID = 134
        
    # Motor 5
    if Motor5_Checkbox.isChecked():
        if M5_Constant_cmd_block.isChecked():
            M5_gID = 115
        elif M5_Sinwave_cmd_block.isChecked():
            M5_gID = 125
        elif M5_Step_cmd_block.isChecked():
            M5_gID = 135
    
    # print(str(M1_gID) + " | " + str(M2_gID) + " | " + str(M3_gID) + " | " + str(M4_gID) + " | " + str(M5_gID))
    

def showdialog():
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Warning)
    msg.setWindowTitle("Warning")
    msg.setText("Value out of bounds")
    msg.setInformativeText("Please introduce a numerical value between 0 and 2.")
    msg.setDetailedText("Every value minor to 0 will be taken as 0 and every value greater that 2 will be taken as 2")
    msg.setStandardButtons(QMessageBox.Ok)
    msg.exec_()


def update_buffers(buffers, values):
    for buffer, value in zip(buffers, values):
        buffer[:] = buffer[1:]  # This modifies the original buffer
        buffer.append(value)
 
def saturation(value, min_value, max_value):
    return max(min_value, min(value, max_value))

# Define the function to run the pipeline
def run_pipeline():
    try:
        automate_workflow()  # Run the function from the other script
        print("Pipeline executed successfully.")
    except Exception as e:
        print(f"An error occurred while running the pipeline: {e}")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    Window = MainWindow()
    Window.show()
    sys.exit(app.exec_())