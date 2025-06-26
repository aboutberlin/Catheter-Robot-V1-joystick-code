// 2024/09/10
// Catheter Robot Wireless Controller

#include "Gemini_Teensy41.h"
#include <FlexCAN_T4.h>
#include <SD.h>
#include <cmath> // Include cmath for fmod

//const int chipSelect = BUILTIN_SDCARD;  // Teensy 4.1 uses the built-in SD card slot
//File dataFile; // Declare dataFile as a global variable

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;

uint32_t ID_offset = 0x140;
uint32_t motor_ID1 = 1; // Motor Can Bus ID 1 for guidewire rotation
uint32_t motor_ID2 = 2;// Motor Can Bus ID 2 for guidewire insertion
uint32_t motor_ID3 = 4; // Motor Can Bus ID 3 for microcatheter (motor #3)
uint32_t motor_ID4 = 5; // Motor Can Bus ID 4 for guidcatheter insertion;
uint32_t motor_ID5 = 6; // Motor Can Bus ID 5 for guidecatheter rotation
uint32_t motor_ID6 = 0; // Motor Can Bus ID 5 for guidecatheter rotation

int CAN_ID = 3;  // CAN port from Teensy
double Gear_ratio = 1;

Gemini_Teensy41 m1(motor_ID1, CAN_ID, Gear_ratio);
Gemini_Teensy41 m2(motor_ID2, CAN_ID, Gear_ratio);
Gemini_Teensy41 m3(motor_ID3, CAN_ID, Gear_ratio);
Gemini_Teensy41 m4(motor_ID4, CAN_ID, Gear_ratio);
Gemini_Teensy41 m5(motor_ID5, CAN_ID, Gear_ratio);
Gemini_Teensy41 m6(motor_ID6, CAN_ID, Gear_ratio);

double freq_ctrl = 100;
double freq_ble  = 30;  
unsigned long t_0 = 0;
unsigned long t   = 0;
unsigned long prev_t_ctrl = 0;
unsigned long prev_t_ble  = 0;                                      
unsigned long T_ctrl_micros = (unsigned long)(1000000 / freq_ctrl);
unsigned long T_ble_micros  = (unsigned long)(1000000 / freq_ble);

double timeInterval = (double)(1000000 / freq_ctrl);
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

int t_teensy = 0;

int M_Selected           = 0;
int M1_Selected          = 0;
int M2_Selected          = 0;
int M3_Selected          = 0;
int M4_Selected          = 0;
int M5_Selected          = 0;
int CtrlMode_Selected    = 0;
int Signal_Mode_Selected = 0;

float pos_cmd        = 0;
float pos_cmd_M1 = 0;
float pos_cmd_M2 = 0;
float pos_cmd_M3 = 0;
float pos_cmd_M4 = 0;
float pos_cmd_M5 = 0;
float sw_bias        = 0;
float sw_amp         = 0;
float sw_freq        = 0;
float step_size      = 0;
float step_period    = 0;
float step_dutycycle = 0;

int M1_g_ID = 0;
int M2_g_ID = 0;
int M3_g_ID = 0;
int M4_g_ID = 0;
int M5_g_ID = 0;

double m1_pos_cmd = 0.0;
double m2_pos_cmd = 0.0;
double m3_pos_cmd = 0.0;
double m4_pos_cmd = 0.0;
double m5_pos_cmd = 0.0;
double m6_pos_cmd = 0.0;

double m1_sw_bias        = 0;
double m1_sw_amp         = 0;
double m1_sw_freq        = 0;
double m1_step_size      = 0;
double m1_step_period    = 0;
double m1_step_dutycycle = 0;

double m2_sw_bias        = 0;
double m2_sw_amp         = 0;
double m2_sw_freq        = 0;
double m2_step_size      = 0;
double m2_step_period    = 0;
double m2_step_dutycycle = 0;

double m3_sw_bias        = 0;
double m3_sw_amp         = 0;
double m3_sw_freq        = 0;
double m3_step_size      = 0;
double m3_step_period    = 0;
double m3_step_dutycycle = 0;

double m4_sw_bias        = 0;
double m4_sw_amp         = 0;
double m4_sw_freq        = 0;
double m4_step_size      = 0;
double m4_step_period    = 0;
double m4_step_dutycycle = 0;

double m5_sw_bias        = 0;
double m5_sw_amp         = 0;
double m5_sw_freq        = 0;
double m5_step_size      = 0;
double m5_step_period    = 0;
double m5_step_dutycycle = 0;

//**************************
/* For linear actuator
   Miniature Linear Motion Series · L12
   Option P: 
   1. Orange - feedback potentiometer negative reference rail
   2. Purple - feedback potentiometer wiper
   3. Red - motor V+ (6V or 12  V)
   4. Black - motor V- (ground)
   5. Yellow - feedback potentiometer positive reference rail
*/
// PWM pin for red and black

// Linear Actuator
// Pins for Actuator 1
const int forwardPin1 = 3;
const int backwardPin1 = 4;

// Pins for Actuator 2
const int forwardPin2 = 5;
const int backwardPin2 = 6;

// Linear Actuator 1
double pos_Lin1 = 0;
double pos_pr_Lin1 = 0;
double pos_dot_Lin1 = 0;
double setpoint1 = 0;
double cmd1 = 0;
double error1 = 0;

double LA1_Pos_cmd = 0;
double LA2_Pos_cmd = 0;

// Linear Actuator 2 
double pos_Lin2 = 0; 
double pos_pr_Lin2 = 0;   
double pos_dot_Lin2 = 0;
double setpoint2 = 0;
double cmd2 = 0;
double error2 = 0;

int pos_L_A_1 = 0;
int pos_L_A_2 = 0;


// PD control constants
const double kp1 = 12;
const double kd1 = 0.3;

// PD control constants
const double kp2 = 12;
const double kd2 = 0.25;


const double con_Num = 0.12135;


double cur_Command_1 = 0;
double vel_Command_1 = 0;
double pos_Command_1 = 0;

double cur_Command_2 = 0;
double vel_Command_2 = 0;
double pos_Command_2 = 0;

double cur_Command_3 = 0;
double vel_Command_3 = 0;
double pos_Command_3 = 0;

double cur_Command_4 = 0;
double vel_Command_4 = 0;
double pos_Command_4 = 0;

double cur_Command_5 = 0;
double vel_Command_5 = 0;
double pos_Command_5 = 0;

double cur_Command_6 = 0;
double vel_Command_6 = 0;
double pos_Command_6 = 0;

double pre_pos_motor1 = 0;
double pre_pos_motor2 = 0;
double pre_pos_motor3 = 0;
double pre_pos_motor4 = 0;
double pre_pos_motor5 = 0;
double pre_pos_motor6 = 0;

double Angle0 = 0;
double Angle1 = 0;
double Angle2 = 0;
double Angle3 = 0;
double Angle4 = 0;
double Angle5 = 0;

double delay_control = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.print("Setting up!");
  delay(100);
  Serial.begin(115200); //used for communication with computer.
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  initial_CAN();
  delay(500);

    m1.init_motor(); 
    m2.init_motor();
    m3.init_motor();
    m4.init_motor();
    m5.init_motor();
    m6.init_motor();

  delay(1000);
  reset_motor_angle();
  delay(1000);

    m1.read_PID();  
    m2.read_PID();
    m3.read_PID();
    m4.read_PID();
    m5.read_PID();
    m6.read_PID();

  Serial.println("Set up complete.");

  //linear actuator 1
  pinMode(forwardPin1, OUTPUT);
  pinMode(backwardPin1, OUTPUT);
  pinMode(A0, INPUT);
  digitalWrite(forwardPin1, LOW);
  digitalWrite(backwardPin1, HIGH);
  
  //linear actuator 2
  pinMode(forwardPin2, OUTPUT);
  pinMode(backwardPin2, OUTPUT);
  pinMode(A1, INPUT);
  digitalWrite(forwardPin2, LOW);
  digitalWrite(backwardPin2, HIGH);

  Serial.print("Controller executed at ");
  Serial.print(String(freq_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(freq_ble));
  Serial.println(" Hz");
  delay(5000);
  t_0 = micros();
}

void loop() {
  
  t = micros() - t_0;
  currentTime = micros();
//  setpoint1 = 909;
//  setpoint2 = 909;
  LA1_cdm_assignment();
  LA2_cdm_assignment();

  if (t - prev_t_ctrl > T_ctrl_micros) {

    Angle0 = m1.motorAngle;
    Angle1 = m2.motorAngle;
    Angle2 = m3.motorAngle;
    Angle3 = m4.motorAngle;
    Angle4 = m5.motorAngle;
    Angle5 = m6.motorAngle;  

    if (t - prev_t_ble > T_ble_micros) {

      Receive_ble_Data();  // Receive ble data
      Transmit_ble_Data(); // Transmit ble data
      prev_t_ble = t;
    }
    
    //For tracking A*sin(2*pi*f*t); omega = 2*pi*f
    pos_Command_1 = 360 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.2 ); // f = 0.4 Hz input with 180 deg amplitude
    pos_Command_2 = 720 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.3);
    pos_Command_3 = 450 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.3); // for micro catheter
    pos_Command_4 = 360* 9 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.1);
    pos_Command_5 = 540 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.1);
    pos_Command_6 = 810 * sin(t / 1000000.0 * 2 * 3.1415926 * 0.2);

    cdm_assignment();

    
    if (currentTime - previousTime > timeInterval) {
      normalizedTime = (currentTime - previousTime)/ 1000000.0;
      
      // --- Actuator 1 control logic ---
          pos_Lin1 = analogRead(A0);  // Read position from analog pin A0
          error1 = setpoint1 - pos_Lin1;
          
          pos_dot_Lin1 = (pos_Lin1 - pos_pr_Lin1) / (normalizedTime);  // Calculate velocity
          cmd1 = kp1 * error1 - kd1 * pos_dot_Lin1;
        
          if (cmd1 >= 0) {
            forth1(constrain(abs(cmd1), 1, 255));
          } else {
            back1(constrain(abs(cmd1), 1, 255));
          }
        
          pos_pr_Lin1 = pos_Lin1;  // Update prior position for the next iteration
        
          // --- Actuator 2 control logic ---
          pos_Lin2 = analogRead(A1);  // Read position from analog pin A1
          error2 = setpoint2 - pos_Lin2;
          pos_dot_Lin2 = (pos_Lin2 - pos_pr_Lin2) / (normalizedTime);  // Calculate velocity
          cmd2 = kp2 * error2 - kd2 * pos_dot_Lin2;
        
          if (cmd2 >= 0) {
            forth2(constrain(abs(cmd2), 1, 255));
          } else {
            back2(constrain(abs(cmd2), 1, 255));
          }
        
          pos_pr_Lin2 = pos_Lin2;  // Update prior position for the next iteration


        // Plot the linear actuator info
//          Serial.print(cmd1);
//          Serial.print("  ");
          Serial.print(pos_Lin1/10);
          Serial.print("  ");
          Serial.print(setpoint1/10);
          Serial.print(" ");
//          Serial.print(error1);
//          Serial.print("  ");
//          Serial.print(pos_dot_Lin1);
//          Serial.print("  ");
//          Serial.print(pos_pr_Lin1);
//          Serial.println("  ");

//            Serial.print(cmd2);
//          Serial.print("  ");
          Serial.print(pos_Lin2/10);
          Serial.print("  ");
          Serial.print(setpoint2/10);
          Serial.println(" ");
//          Serial.print(error2);
//          Serial.print("  ");
//          Serial.print(pos_dot_Lin2);
//          Serial.print("  ");
//          Serial.print(pos_pr_Lin2);
//          Serial.println("  ");

          pos_pr_Lin1 = pos_Lin1;
          pos_pr_Lin2 = pos_Lin2;
    
      previousTime = currentTime;
    }

    
    m1.send_position_command_2(m1_pos_cmd, 10000);  // send the command to mID1
    m2.send_position_command_2(m2_pos_cmd, 10000);  // send the command to mID2
    m3.send_position_command_2(m3_pos_cmd, 10000);  // send the command to mID4
    m4.send_position_command_2(m4_pos_cmd, 10000);  // send the command to mID7
    m5.send_position_command_2(m5_pos_cmd, 10000);  // send the command to mID8
    m6.send_position_command_2(m6_pos_cmd, 10000);

//test 

//    m1.send_position_command_2(pos_Command_1, 10000);  // send the command to mID1
//    m2.send_position_command_2(pos_Command_2, 10000);  // send the command to mID2
//    m3.send_position_command_2(pos_Command_3, 10000);  // send the command to mID4
//    m4.send_position_command_2(pos_Command_4, 10000);  // send the command to mID7
//    m5.send_position_command_2(pos_Command_5, 10000);  // send the command to mID8
//    m6.send_position_command_2(pos_Command_6, 10000);

    // For mID1
    m1.receive_CAN_data();
    Wait(delay_control);

    m1.read_motor_status_3();
    m1.receive_CAN_data();
    Wait(delay_control);

    m1.read_multi_turns_angle();
    m1.receive_CAN_data();
    Wait(delay_control);

    // For mID2
    m2.receive_CAN_data();
    Wait(delay_control);
    
    m2.read_motor_status_3();
    m2.receive_CAN_data();
    Wait(delay_control);
    
    m2.read_multi_turns_angle();
    m2.receive_CAN_data();
    Wait(delay_control);
    
    // For mID4
    m3.receive_CAN_data();
    Wait(delay_control);
    
    m3.read_motor_status_3();
    m3.receive_CAN_data();
    Wait(delay_control);
    
    m3.read_multi_turns_angle();
    m3.receive_CAN_data();
    Wait(delay_control);

    // For mID7

    m4.receive_CAN_data();
    Wait(delay_control);
    
    m4.read_motor_status_3();
    m4.receive_CAN_data();
    Wait(delay_control);
    
    m4.read_multi_turns_angle();
    m4.receive_CAN_data();
    Wait(delay_control);

    //For mID8

    m5.receive_CAN_data();
    Wait(delay_control);
    
    m5.read_motor_status_3();
    m5.receive_CAN_data();
    Wait(delay_control);
    
    m5.read_multi_turns_angle();
    m5.receive_CAN_data();
    Wait(delay_control);

    //For mID8

    m6.receive_CAN_data();
    Wait(delay_control);
    
    m6.read_motor_status_3();
    m6.receive_CAN_data();
    Wait(delay_control);
    
    m6.read_multi_turns_angle();
    m6.receive_CAN_data();
    Wait(delay_control);

    Angle0 = m1.motorAngle;
    Angle1 = m2.motorAngle;
    Angle2 = m3.motorAngle;
    Angle3 = m4.motorAngle;
    Angle4 = m5.motorAngle;
    Angle5 = m6.motorAngle;    
//    Serial.print(m1.motorAngle);
//    Serial.print(";");

///////////////////////////////
//    Serial.print(Angle1);
//    Serial.print(";");
//    Serial.print(Angle2);
//    Serial.print(";");
//    Serial.print(Angle3);
//    Serial.print(";");
//    Serial.print(Angle4);
//    Serial.print(";");
//    Serial.print(Angle5);
//    Serial.println(";");
///////////////////////////////

    prev_t_ctrl = t;

  }
}

////Linear Actuator 1 Function 
void forth1(double v1) {
  analogWrite(forwardPin1, v1);
  digitalWrite(backwardPin1, LOW);
}

void back1(double v1) {
  analogWrite(backwardPin1, v1);
  digitalWrite(forwardPin1, LOW);
}

////Linear Actuator 2 Function 
void forth2(double v2) {
  analogWrite(forwardPin2, v2);
  digitalWrite(backwardPin2, LOW);
}

void back2(double v2) {
  analogWrite(backwardPin2, v2);
  digitalWrite(forwardPin2, LOW);
}

void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(500);
  Serial.println("Can bus setup done...");
  delay(500);
}

void reset_motor_angle() {
  for (int i = 0; i < 20; i++) {
    m1.read_multi_turns_angle();
    delay(10);
    m1.receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    delay(10);

    m2.read_multi_turns_angle();
    delay(10);
    m2.receive_CAN_data();
    m2.motorAngle_offset = m2.motorAngle_raw;
    delay(10);
//    
    m3.read_multi_turns_angle();
    delay(10);
    m3.receive_CAN_data();
    m3.motorAngle_offset = m3.motorAngle_raw;
    delay(10);
    
    m4.read_multi_turns_angle();
    delay(10);
    m4.receive_CAN_data();
    m4.motorAngle_offset = m4.motorAngle_raw;
    delay(10);

    m5.read_multi_turns_angle();
    delay(10);
    m5.receive_CAN_data();
    m5.motorAngle_offset = m5.motorAngle_raw;
    delay(10);

    m6.read_multi_turns_angle();
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
//  LA1_Pos_cmd          = 340;
//  LA2_Pos_cmd          = 630;
  if (Serial5.available() >= 20) {
    // Read the incoming byte:
    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165) { // Check the first byte

      if (data_rs232_rx[1] == 90) { // Check the second byte

        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          M1_g_ID              = data_rs232_rx[3]; 
          M2_g_ID              = data_rs232_rx[4];
          M3_g_ID              = data_rs232_rx[5];
          M4_g_ID              = data_rs232_rx[6];
          M5_g_ID              = data_rs232_rx[7];
          pos_cmd_M1           = data_rs232_rx[8]; 
          pos_cmd_M2           = data_rs232_rx[9];
          pos_cmd_M3           = data_rs232_rx[10];
          pos_cmd_M4           = data_rs232_rx[11];
          pos_cmd_M5           = data_rs232_rx[12];
          sw_bias              = data_rs232_rx[13];
          sw_amp               = data_rs232_rx[14];
          sw_freq              = data_rs232_rx[15] / 100.0;
          LA1_Pos_cmd          = data_rs232_rx[16] * 10;
          LA2_Pos_cmd          = data_rs232_rx[17] * 10;
          

//          g_ID_assignment();

          Serial.print("| Motor ID ");
          Serial.print(M1_g_ID, DEC); // This contains the motor 1 ID 
          Serial.print(M2_g_ID, DEC); // This contains the motor 2 ID 
          Serial.print(M3_g_ID, DEC); // This contains the motor 3 ID 
          Serial.print(M4_g_ID, DEC); // This contains the motor 4 ID 
          Serial.print(M5_g_ID, DEC); // This contains the motor 5 ID 
          Serial.print(" Position Commands ");
          Serial.print(pos_cmd_M1, DEC); // This contains the motor 1 Pos
          Serial.print(pos_cmd_M2, DEC); // This contains the motor 2 Pos
          Serial.print(pos_cmd_M3, DEC); // This contains the motor 3 Pos
          Serial.print(pos_cmd_M4, DEC); // This contains the motor 4 Pos
          Serial.print(pos_cmd_M5, DEC); // This contains the motor 5 Pos
          Serial.print(setpoint1, DEC); // This contains the LA 1 Pos
          Serial.print(setpoint2, DEC); // This contains the LA 2 Pos
          Serial.println(" sent |");
        }
      }
    }
  }
}

void Transmit_ble_Data(){
  t_teensy = (t / 10000.0);
  M1_pos = Angle1 * 100;
  M2_pos = Angle2 * 100;
  M3_pos = Angle3 * 100;
  M4_pos = Angle4 * 100;
  M5_pos = Angle5 * 100;

  pos_L_A_1 = (int)pos_Lin1/10;
  pos_L_A_2 = (int)pos_Lin2/10;

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
  data_ble[15] = pos_L_A_1;
  data_ble[16] = pos_L_A_1 >> 8;
  data_ble[17] = pos_L_A_2;
  data_ble[18] = pos_L_A_2 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0 >> 8;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0;

  Serial5.write(data_ble, datalength_ble);
}


void M1_cdm_assignment() {
  switch (M1_g_ID) {
    case 111:
      m1_pos_cmd = pos_cmd_M1;
      break;
    case 121:
      m1_pos_cmd = m1_sw_bias + m1_sw_amp * sin((2 * PI * m1_sw_freq) * (t / 1000000.0));
      break;
    case 131:
      m1_pos_cmd = StepFunction(m1_step_size, m1_step_period, m1_step_dutycycle, t);
      break;
  }
}

void M2_cdm_assignment() {
  switch (M2_g_ID) {
    case 112:
      m2_pos_cmd = pos_cmd_M2;
      break;
    case 122:
      m2_pos_cmd = m2_sw_bias + m2_sw_amp * sin((2 * PI * m2_sw_freq) * (t / 1000000.0));
      break;
    case 132:
      m2_pos_cmd = StepFunction(m2_step_size, m2_step_period, m2_step_dutycycle, t);
      break;
  }
}

void M3_cdm_assignment() {
  switch (M3_g_ID) {
    case 113:
      m3_pos_cmd = pos_cmd_M3;
      break;
    case 123:
      m3_pos_cmd = m3_sw_bias + m3_sw_amp * sin((2 * PI * m3_sw_freq) * (t / 1000000.0));
      break;
    case 133:
      m3_pos_cmd = StepFunction(m3_step_size, m3_step_period, m3_step_dutycycle, t);
      break;
  }
}

void M4_cdm_assignment() {
  switch (M4_g_ID) {
    case 114:
      m4_pos_cmd = pos_cmd_M4;
      break;
    case 124:
      m4_pos_cmd = m4_sw_bias + m4_sw_amp * sin((2 * PI * m4_sw_freq) * (t / 1000000.0));
      break;
    case 134:
      m4_pos_cmd = StepFunction(m4_step_size, m4_step_period, m4_step_dutycycle, t);
      break;
  }
}

void M5_cdm_assignment() {
  switch (M5_g_ID) {
    case 115:
      m5_pos_cmd = pos_cmd_M5;
      break;
    case 125:
      m5_pos_cmd = m5_sw_bias + m5_sw_amp * sin((2 * PI * m5_sw_freq) * (t / 1000000.0));
      break;
    case 135:
      m5_pos_cmd = StepFunction(m5_step_size, m5_step_period, m5_step_dutycycle, t);
      break;
  }
}

void LA1_cdm_assignment() {
  setpoint1 = LA1_Pos_cmd;
//    setpoint1 = 540;
}

void LA2_cdm_assignment() {
  setpoint2 = LA2_Pos_cmd;
//  setpoint2 = 630;
}


void cdm_assignment() {
  M1_cdm_assignment();
  M2_cdm_assignment();
  M3_cdm_assignment();
  M4_cdm_assignment();
  M5_cdm_assignment();
//  LA1_cdm_assignment();
//  LA2_cdm_assignment();
}


double StepFunction(double step_size, double period, double duty_cycle, unsigned long t) {
  double t_high = period * (duty_cycle / 100.0); // Calculate the high time based on the duty cycle
  double mod_time = fmod(t / 1000000.0, period); // Determine the position in the current cycle

  if (mod_time < t_high) {
    // If within the high duration, return the step size
    return step_size;
  } else {
    // Otherwise, return 0 for the low duration
    return 0;
  }
}
