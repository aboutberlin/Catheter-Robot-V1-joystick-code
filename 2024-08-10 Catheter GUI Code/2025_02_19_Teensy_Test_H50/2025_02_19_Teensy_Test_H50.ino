// 19/02/2025
// Catheter Robot Wireless Controller

#include "H50_motor_lib.h"
#include <FlexCAN_T4.h>
#include <SD.h>
#include <cmath> // Include cmath for fmod
#include <stdbool.h>

//const int chipSelect = BUILTIN_SDCARD;  // Teensy 4.1 uses the built-in SD card slot
//File dataFile; // Declare dataFile as a global variable

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;

double test_angle = 0;

uint32_t ID_offset = 0x140;
uint32_t motor_ID0 = 6; // Dummy motor
uint32_t motor_ID1 = 1; // Motor Can Bus ID 1 for guidewire rotation
uint32_t motor_ID2 = 2; // Motor Can Bus ID 2 for guidewire insertion
uint32_t motor_ID3 = 3; // Motor Can Bus ID 3 
uint32_t motor_ID4 = 4; // Motor Can Bus ID 4 
uint32_t motor_ID5 = 5; // Motor Can Bus ID 5 
uint32_t motor_ID6 = 6; // Motor Can Bus ID 6

int CAN_ID = 3;  // CAN port from Teensy
double Gear_ratio = 1;

H50_motor_lib m0(motor_ID0, CAN_ID, Gear_ratio);
H50_motor_lib m1(motor_ID1, CAN_ID, Gear_ratio);
H50_motor_lib m2(motor_ID2, CAN_ID, Gear_ratio);
H50_motor_lib m3(motor_ID3, CAN_ID, Gear_ratio);
H50_motor_lib m4(motor_ID4, CAN_ID, Gear_ratio);
H50_motor_lib m5(motor_ID5, CAN_ID, Gear_ratio);
H50_motor_lib m6(motor_ID6, CAN_ID, Gear_ratio);

bool coupled_flag = false;

double freq_ctrl = 100;
double freq_ble  = 60;  
unsigned long t_0 = 0;
unsigned long t   = 0;
unsigned long prev_t_ctrl = 0;
unsigned long prev_t_ble  = 0;                                      
unsigned long T_ctrl_micros = (unsigned long)(1000000.0 / freq_ctrl);
unsigned long T_ble_micros  = (unsigned long)(1000000.0 / freq_ble);

double timeInterval = (double)(1000000.0 / freq_ctrl);
double currentTime = 0;
double previousTime = 0; 
double normalizedTime = 0;


//Data sent via bluetooth
char datalength_ble    = 32;
char data_ble[60]      = {0};
char data_rs232_rx[60] = {0};

int M1_pos = 0;
int M2_pos = 0;
int M3_pos = 0;
int M4_pos = 0;
int M5_pos = 0;
int M6_pos = 0;

int t_teensy = 0;

float pos_cmd_M1 = 0;
float pos_cmd_M2 = 0;
float pos_cmd_M3 = 0;
float pos_cmd_M4 = 0;
float pos_cmd_M5 = 0;
float pos_cmd_M6 = 0;

double m1_pos_cmd = 0.0;
double m2_pos_cmd = 0.0;
double m3_pos_cmd = 0.0;
double m4_pos_cmd = 0.0;
double m5_pos_cmd = 0.0;
double m6_pos_cmd = 0.0;

double m1_pos_cmd_prev = 0.0;
double m2_pos_cmd_prev = 0.0;
double m3_pos_cmd_prev = 0.0;
double m4_pos_cmd_prev = 0.0;
double m5_pos_cmd_prev = 0.0;
double m6_pos_cmd_prev = 0.0;

double delay_control = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.println("- - - - - - - - - - - - - - - - - -");
  Serial.println("Initial Setup...");
  delay(100);
  Serial.begin(115200); //used for communication with computer.
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN();
  delay(500);

  // m0.init_motor();
  m1.init_motor();
  // m2.init_motor();
  // m3.init_motor();
  // m4.init_motor();
  // m5.init_motor();
  // m6.init_motor();

  delay(1000);
  // reset_motor_angle();
  // test_reset_motor_angle();
  delay(1000);


  // m0.read_PID_gains(); 
  // m1.read_PID_gains();
  // m2.read_PID_gains();
  // m3.read_PID_gains();
  // m4.read_PID_gains();
  // m5.read_PID_gains();
  // m6.read_PID_gains();

  // Testing the assingment of the PID parameters
  // Serial.println(m1.cur_kp);
  // Serial.println(m1.cur_ki);
  // Serial.println(m1.spd_kp);
  // Serial.println(m1.spd_ki);
  // Serial.println(m1.pos_kp);
  // Serial.println(m1.pos_ki);
  // Serial.println(m1.pos_kd);

  Serial.println("Motor Setup complete!");
  Serial.println("* * * * * * * * * * * * * * * * * *");
  Serial.print("Controller executed at ");
  Serial.print(String(freq_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(freq_ble));
  Serial.println(" Hz");
  Serial.println("- - - - - - - - - - - - - - - - - -");
  delay(5000);
  t_0 = micros();
}

void loop() {
  
  t = micros() - t_0;
  currentTime = micros();

  if (t - prev_t_ctrl > T_ctrl_micros) {

    if (t - prev_t_ble > T_ble_micros) {
      Receive_ble_Data();
      Transmit_ble_Data();
      prev_t_ble = t;
    }

    // m1.read_multi_turn_angle();
    // Serial.println(m1.motorAngle_new);

    m1_pos_cmd = 0 + 45 * sin((2 * PI * 0.25) * (t / 1000000.0));
    m1.send_position_command_2(m1_pos_cmd, 1000);
    
    test_read_motor_data();
    Serial.print(m1_pos_cmd);
    Serial.print(" | ");
    Serial.println(m1.motorAngle_new);

    

    // cmd_assignment();
    // send_cmd_to_motors();
    // read_motor_data();

    prev_t_ctrl = t;
  }
}

void test_read_motor_data(){
  // m1.receive_CAN_data();
  // Wait(delay_control);
  // m1.read_motor_status_3();
  // m1.receive_CAN_data();
  // Wait(delay_control);
  m1.read_multi_turn_angle();
  m1.receive_CAN_data();
  Wait(delay_control);
}

void test_reset_motor_angle(){
    m1.read_multi_turn_angle();
    delay(10);
    m1.receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    delay(10);
}


void initial_CAN() {
  Serial.println("CAN Communication Setup...");
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(500);
  Serial.println("CAN Communication Setup DONE");
  delay(500);
}


void reset_motor_angle() {
  for (int i = 0; i < 20; i++) {
    m0.read_multi_turn_angle();
    delay(10);
    m0.receive_CAN_data();
    m0.motorAngle_offset = m0.motorAngle_raw;
    delay(10);

    m1.read_multi_turn_angle();
    delay(10);
    m1.receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    delay(10);

    m2.read_multi_turn_angle();
    delay(10);
    m2.receive_CAN_data();
    m2.motorAngle_offset = m2.motorAngle_raw;
    delay(10);
 
    m3.read_multi_turn_angle();
    delay(10);
    m3.receive_CAN_data();
    m3.motorAngle_offset = m3.motorAngle_raw;
    delay(10);
    
    m4.read_multi_turn_angle();
    delay(10);
    m4.receive_CAN_data();
    m4.motorAngle_offset = m4.motorAngle_raw;
    delay(10);

    m5.read_multi_turn_angle();
    delay(10);
    m5.receive_CAN_data();
    m5.motorAngle_offset = m5.motorAngle_raw;
    delay(10);

    m6.read_multi_turn_angle();
    delay(10);
    m6.receive_CAN_data();
    m6.motorAngle_offset = m6.motorAngle_raw;
    delay(10);
  }
}

void Wait(double delay_Control) {
  double time_Start = micros();
  double time_Delta = delay_Control;
  double time_Control = 0;

  do {
    time_Control = micros() - time_Start;
  } while (time_Control < time_Delta);
}

void tunning() {
  m0.write_PID_RAM();
  Wait(delay_control);
  m0.write_PID_RAM();
  Wait(delay_control);
  m0.write_PID_ROM();
  Wait(delay_control);
  m0.write_PID_ROM();
  Wait(delay_control);

  m1.write_PID_RAM();
  Wait(delay_control);
  m1.write_PID_RAM();
  Wait(delay_control);
  m1.write_PID_ROM();
  Wait(delay_control);
  m1.write_PID_ROM();
  Wait(delay_control);

  m2.write_PID_RAM();
  Wait(delay_control);
  m2.write_PID_RAM();
  Wait(delay_control);
  m2.write_PID_ROM();
  Wait(delay_control);
  m2.write_PID_ROM();
  Wait(delay_control);

  m3.write_PID_RAM();
  Wait(delay_control);
  m3.write_PID_RAM();
  Wait(delay_control);
  m3.write_PID_ROM();
  Wait(delay_control);
  m3.write_PID_ROM();
  Wait(delay_control);

  m4.write_PID_RAM();
  Wait(delay_control);
  m4.write_PID_RAM();
  Wait(delay_control);
  m4.write_PID_ROM();
  Wait(delay_control);
  m4.write_PID_ROM();
  Wait(delay_control);

  m5.write_PID_RAM();
  Wait(delay_control);
  m5.write_PID_RAM();
  Wait(delay_control);
  m5.write_PID_ROM();
  Wait(delay_control);
  m5.write_PID_ROM();
  Wait(delay_control);

  m6.write_PID_RAM();
  Wait(delay_control);
  m6.write_PID_RAM();
  Wait(delay_control);
  m6.write_PID_ROM();
  Wait(delay_control);
  m6.write_PID_ROM();
  Wait(delay_control);
}

void Receive_ble_Data(){
  if (Serial5.available() >= 20) { // Read the incoming byte:

    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165 && data_rs232_rx[1] == 90 && data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

      pos_cmd_M1 = (float)((int16_t)((data_rs232_rx[4] << 8) | data_rs232_rx[3])) / 10.0;
      pos_cmd_M2 = (float)((int16_t)((data_rs232_rx[6] << 8) | data_rs232_rx[5])) / 10.0;
      pos_cmd_M3 = (float)((int16_t)((data_rs232_rx[8] << 8) | data_rs232_rx[7])) / 10.0;
      pos_cmd_M4 = (float)((int16_t)((data_rs232_rx[10] << 8) | data_rs232_rx[9])) / 10.0;
      pos_cmd_M5 = (float)((int16_t)((data_rs232_rx[12] << 8) | data_rs232_rx[11])) / 10.0;
      pos_cmd_M6 = (float)((int16_t)((data_rs232_rx[14] << 8) | data_rs232_rx[13])) / 10.0;
    }
  }
}

void Transmit_ble_Data(){

  t_teensy = (t / 1000000.0)*10.0; // t is in microseconds
  M1_pos = (int) (m2.motorAngle * 10.0);
  M2_pos = (int) (m3.motorAngle * 10.0);
  M3_pos = (int) (m4.motorAngle * 10.0);
  M4_pos = (int) (m5.motorAngle * 10.0);
  M5_pos = (int) (m6.motorAngle * 10.0);
  M6_pos = (int) (m1.motorAngle * 10.0);

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8;
  data_ble[5]  = M1_pos;
  data_ble[6]  = M1_pos >> 8;
  data_ble[7]  = M2_pos;
  data_ble[8]  = M2_pos >> 8;
  data_ble[9]  = M3_pos;
  data_ble[10] = M3_pos >> 8;
  data_ble[11] = M4_pos;
  data_ble[12] = M4_pos >> 8;
  data_ble[13] = M5_pos;
  data_ble[14] = M5_pos >> 8;
  data_ble[15] = M6_pos;
  data_ble[16] = M6_pos >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0 >> 8;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 123;

  Serial5.write(data_ble, datalength_ble);
  // test_angle = (double) (m1.motorAngle_int32) / 100.0; // This get the global angular position (no boundaries between -359 to 359)
}


void M1_cdm_assignment() {
  m1_pos_cmd = pos_cmd_M1;
}

void M2_cdm_assignment() {
  m2_pos_cmd = pos_cmd_M2;
}

void M3_cdm_assignment() {
  m3_pos_cmd = pos_cmd_M3;
}

void M4_cdm_assignment() {
  m4_pos_cmd = pos_cmd_M4;
}

void M5_cdm_assignment() {
  m5_pos_cmd = pos_cmd_M5;
}

void M6_cdm_assignment() {
  m6_pos_cmd = pos_cmd_M6;
}


void test_cmds(){
  m1_pos_cmd = 0 + 45 * sin((2 * PI * 0.25) * (t / 1000000.0));
  m2_pos_cmd = 0 + 45 * sin((2 * PI * 0.5) * (t / 1000000.0));
  m3_pos_cmd = 0 + 90 * sin((2 * PI * 0.25) * (t / 1000000.0));
  m4_pos_cmd = 0 + 90 * sin((2 * PI * 0.5) * (t / 1000000.0));
  m5_pos_cmd = 0 + 135 * sin((2 * PI * 0.25) * (t / 1000000.0));
  m6_pos_cmd = 0 + 135 * sin((2 * PI * 0.5) * (t / 1000000.0));
}

void cmd_assignment() {
  M1_cdm_assignment();
  M2_cdm_assignment();
  M3_cdm_assignment();
  M4_cdm_assignment();
  M5_cdm_assignment();
  M6_cdm_assignment();
  // test_cmds();
}

void send_cmd_to_motors(){
  m1.send_position_command_2(m1_pos_cmd, 10000);
  m2.send_position_command_2(m2_pos_cmd, 10000);
  m3.send_position_command_2(m3_pos_cmd, 10000);
  m4.send_position_command_2(m4_pos_cmd, 10000);
  m5.send_position_command_2(m5_pos_cmd, 10000);
  m6.send_position_command_2(m6_pos_cmd, 10000);
}

void read_motor_data(){
  // For mID0
  m0.receive_CAN_data();
  Wait(delay_control);
  m0.read_motor_status_3();
  m0.receive_CAN_data();
  Wait(delay_control);
  m0.read_multi_turn_angle();
  m0.receive_CAN_data();
  Wait(delay_control);

  // For mID1
  m1.receive_CAN_data();
  Wait(delay_control);
  m1.read_motor_status_3();
  m1.receive_CAN_data();
  Wait(delay_control);
  m1.read_multi_turn_angle();
  m1.receive_CAN_data();
  Wait(delay_control);

  // For mID2
  m2.receive_CAN_data();
  Wait(delay_control);    
  m2.read_motor_status_3();
  m2.receive_CAN_data();
  Wait(delay_control);    
  m2.read_multi_turn_angle();
  m2.receive_CAN_data();
  Wait(delay_control);
  
  // For mID3
  m3.receive_CAN_data();
  Wait(delay_control);    
  m3.read_motor_status_3();
  m3.receive_CAN_data();
  Wait(delay_control);    
  m3.read_multi_turn_angle();
  m3.receive_CAN_data();
  Wait(delay_control);

  // For mID4
  m4.receive_CAN_data();
  Wait(delay_control);    
  m4.read_motor_status_3();
  m4.receive_CAN_data();
  Wait(delay_control);    
  m4.read_multi_turn_angle();
  m4.receive_CAN_data();
  Wait(delay_control);

  //For mID5
  m5.receive_CAN_data();
  Wait(delay_control);    
  m5.read_motor_status_3();
  m5.receive_CAN_data();
  Wait(delay_control);    
  m5.read_multi_turn_angle();
  m5.receive_CAN_data();
  Wait(delay_control);

  // For mID6
  m6.receive_CAN_data();
  Wait(delay_control);    
  m6.read_motor_status_3();
  m6.receive_CAN_data();
  Wait(delay_control);    
  m6.read_multi_turn_angle();
  m6.receive_CAN_data();
  Wait(delay_control);
}