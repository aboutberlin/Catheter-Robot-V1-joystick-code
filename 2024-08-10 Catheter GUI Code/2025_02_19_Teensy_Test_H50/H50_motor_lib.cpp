/*
[2025-02-20] Ivan Lopez-Sanchez
This library was made for MyActuator motors RMD-L-4010 & H-50-15
|Driver: V3 | Version: V4.2 | Date: 2024.05|
Check the manuals for the motors:
RMD-L-4010 ("Motor Motion Protocol V4.2-250208.pdf"): https://www.myactuator.com/downloads-lseries
H-50-15    ("Motor Motion Protocol V4.2-241219.pdf"): https://www.myactuator.com/downloads-hseries
*/

#include "H50_motor_lib.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

float decode_uint32_to_float(const uint8_t Data[8]) {
    union {
        uint32_t u32;
        float f;
    } converter;
    
    // Combine bytes in little-endian format (Data[4] is LSB, Data[7] is MSB)
    converter.u32 = ((uint32_t)Data[7] << 24) |  // Most significant byte
                    ((uint32_t)Data[6] << 16) |
                    ((uint32_t)Data[5] << 8)  |
                    (uint32_t)Data[4];         // Least significant byte

    return converter.f;
}

float decode_uint32_to_int(const uint8_t Data[8]) {
    union {
        uint32_t u32;
        int i;
    } converter;
    
    // Combine bytes in little-endian format (Data[4] is LSB, Data[7] is MSB)
    converter.u32 = ((uint32_t)Data[7] << 24) |  // Most significant byte
                    ((uint32_t)Data[6] << 16) |
                    ((uint32_t)Data[5] << 8)  |
                    (uint32_t)Data[4];         // Least significant byte

    return converter.i;
}

uint8_t indexes[] = {0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09};

H50_motor_lib::H50_motor_lib(uint32_t id, int Can_id, double Gear_ratio)
{
  ID = id;
  gear_ratio = Gear_ratio;
}
H50_motor_lib::~H50_motor_lib()
{}
void H50_motor_lib::init_motor()
{
  Serial.println((String)"Starting motor initialization for Motor ID: " + ID);
  read_motor_status_1();
  read_motor_status_2();
  read_motor_status_3();

  // motor_shutdown();
  comm_baud_rate_set();
  motor_stop();
  read_PID_gains();
  read_single_turn_encoder();
  read_single_turn_angle();
  read_acceleration();
  // motor_brake_release();

  read_motor_status_1();
  read_motor_status_2();
  read_motor_status_3();


  // //clear_motor_error();
  // // motor_reset();
  // start_motor(); // This does not exist in this protocol version
  // // delay(100);
  // motor_brake_release();
  // delay(100);
  // // Read Initial Position
  // read_PID_gains();
  // read_single_turn_encoder();
  // read_single_turn_angle();
  // write_encoder_offset_RAM(circleAngle);
  // read_multi_turn_encoder_pos();
  // // write_multi_turn_encoder_pos_to_ROM_as_zero();
  // read_motor_status_1();
  // read_motor_status_2();
  // read_motor_status_3();



  // delay(100);
  // receive_CAN_data();
  // delay(10);
  // read_acceleration();
  // receive_CAN_data();
  // delay(10);
  // read_motor_status_1();
  // receive_CAN_data();
  // delay(10);
  // read_motor_status_2();
  // receive_CAN_data();
  // delay(10);
  // read_motor_status_3();
  // receive_CAN_data();
  // delay(10);
  motorAngle_offset = motorAngle_raw;
}
//////////////////Check if there is a CAN message, then read it///////////////
// void H50_motor_lib::receive_CAN_data()
// {
//  while (Can3.available() > 0)
//  {
//    Can3.read(msgR);
//    DataExplanation(msgR);
//  }
// }
////////////////////////////////////////////////////////////////////////////


////////////////Received CAN Message Decoding////////////////////////////////
void H50_motor_lib::DataExplanation(CAN_message_t msgR2)
{
  int len = msgR2.len;
  if (len == 8)
  {
    switch (msgR2.buf[0])
    {
      case 0x30: // Read PID Parameter Command
        Serial.println("Reading Controller Gains...");
        switch (msgR2.buf[1])
        {
          case 0x01:
            cur_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Current Control kp: " + cur_kp);
            break;
          case 0x02:
            cur_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Current Control ki: " + cur_ki);
            break;
          case 0x04:
            spd_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Speed Control kp: " + spd_kp);
            break;
          case 0x05:
            spd_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Speed Control ki: " + spd_ki);
            break;
          case 0x07:
            pos_kp = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kp: " + pos_kp);
            break;
          case 0x08:
            pos_ki = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control ki: " + pos_ki);
            break;
          case 0x09:
            pos_kd = decode_uint32_to_float(msgR2.buf);
            Serial.println((String)"Position Control kd: " + pos_kd);
            break;
          default:
              break;
        }
        break;

      case 0x31: // Write PID Parameters to RAM Command
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        Serial.println("Success to write PID to RAM");
        Serial.print("anglePidKp:");
        Serial.print(anglePidKp);
        Serial.print("; anglePidKi:");
        Serial.print(anglePidKi);
        Serial.print("; speedPidKp:");
        Serial.print(speedPidKp);
        Serial.print("; speedPidKi:");
        Serial.print(speedPidKi);
        Serial.print("; iqPidKp:");
        Serial.print(iqPidKp);
        Serial.print("; iqPidKi:");
        Serial.println(iqPidKi);
        break;

      case 0x32: // Write PID Parameters to ROM Command
        anglePidKp = msgR2.buf[2];
        anglePidKi = msgR2.buf[3];
        speedPidKp = msgR2.buf[4];
        speedPidKi = msgR2.buf[5];
        iqPidKp = msgR2.buf[6];
        iqPidKi = msgR2.buf[7];
        //        Serial.println("Success to write PID to ROM");
        //        Serial.print("anglePidKp:");
        //        Serial.print(anglePidKp);
        //        Serial.print("; anglePidKi:");
        //        Serial.print(anglePidKi);
        //        Serial.print("; speedPidKp:");
        //        Serial.print(speedPidKp);
        //        Serial.print("; speedPidKi:");
        //        Serial.print(speedPidKi);
        //        Serial.print("; iqPidKp:");
        //        Serial.print(iqPidKp);
        //        Serial.print("; iqPidKi:");
        //        Serial.println(iqPidKi);
        break;

      case 0x42: // Read Acceleration Command
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
               Serial.print("Success to read parameter Accel: ");
               Serial.println(Accel);
        break;

      case 0x34: //5.write Accel to RAM
        acceleration_uint32 = (uint32_t)(((uint32_t)msgR2.buf[7] << 24) | ((uint32_t)msgR2.buf[6] << 16) | ((uint32_t)msgR2.buf[5] << 8) | ((uint32_t)msgR2.buf[4]));
        acceleration_int32 = (int32_t)acceleration_uint32;
        Accel = acceleration_int32; //unit 1dps/s dps(degree per second)
               Serial.println("Success to write parameter Accel to RAM");
               Serial.print("Accel:");
               Serial.println(Accel);
        break;

      case 0x60: // Read Multi-Turn Encoder Position Data Command
        Serial.println("Read Multi-Turn Encoder Position Data Command():");
        encoder_multi_turn_pos = decode_uint32_to_int(msgR2.buf);
        Serial.println(encoder_multi_turn_pos);
        break;

      case 0x64: // Read Multi-Turn Encoder Position Data Command
        Serial.println("Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command():");
        encoder_zero_bias = decode_uint32_to_int(msgR2.buf);
        Serial.println(encoder_zero_bias);
        break;

      case 0x76: // System Reset Command
        Serial.println("System Reset Command():");
        break;

      case 0x77: // System Reset Command
        Serial.println("System Brake Release Command():");
        break;

      case 0x90: // Read Single-Turn Encoder Command
        encoder       = (uint16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        encoderRaw    = (uint16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        encoderOffset = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.println("Success to read encoder parameters:");
        Serial.print("encoder:");
        Serial.print(encoder);
        Serial.print("| encoderRaw:");
        Serial.print(encoderRaw);
        Serial.print("| encoderOffset:");
        Serial.println(encoderOffset);
        break;

      case 0x92: // Read Multi-Turn Angle Command
        motorAngle_new = decode_uint32_to_int(msgR2.buf) * 0.01;
        break;

      case 0x94: // Read Single-Turn Angle Command
        Serial.println("Read Single-Turn Angle Command()");
        circleAngle = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        Serial.println(circleAngle);
        break;

      case 0x9A: // Read Motor Status 1 and Error Flag Command
        temperature = (int8_t)msgR2.buf[1]; // Motor Temperature
        RlyCtrlRslt = (int8_t)msgR2.buf[3]; // Brake release command
        voltage     = (uint16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4])); // Voltage
        errorState  = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6])); // Error Status

        Serial.println("Successful to read motor status");
        Serial.print("Temperature is ");
        Serial.print(temperature);
        Serial.print("| Voltage is ");
        Serial.print(voltage * 0.1);
        Serial.print("| ErrorState is ");
        Serial.println(errorState);

        switch (errorState) {
          case 0x0000:
            Serial.println("No Errors Found");
            break;
          case 0x0002:
            Serial.println("Motor Stall");
            break;
          case 0x0004:
            Serial.println("Low Voltage");
            break;
          case 0x0008:
            Serial.println("Over Voltage");
            break;
          case 0x0010:
            Serial.println("Over Current");
            break;
          case 0x0040:
            Serial.println("Power Overrun");
            break;
          case 0x0080:
            Serial.println("Calibration Parameter Writing Error");
            break;
          case 0x0100:
            Serial.println("Speeding");
            break;
          case 0x1000:
            Serial.println("Motor Temperature Over Temperature");
            break;
          case 0x2000:
            Serial.println("Encoder Calibration Error");
            break;
        }
        break;
      case 0x9B: //13: clear motor error status return temperature, voltage, errorState
        temperature = (int8_t)msgR2.buf[1];
        voltage = (uint16_t)(((uint16_t)msgR2.buf[4] << 8) | ((uint16_t)msgR2.buf[3]));
        errorState = msgR2.buf[7];
        //        Serial.println("successful to clear motor error");
        //        Serial.print("temperature is ");
        //        Serial.print(temperature);
        //        Serial.print("; voltage is ");
        //        Serial.print(voltage);
        //        Serial.print("; errorState is ");
        //        Serial.println(errorState);
        if (errorState & 0x01)
        {
          //          Serial.println("low voltage protection");
        }
        else if (errorState & 0x08)
        {
          Serial.println("Over temperature protection");
        }
        break;

      case 0x9C: // Read Motor Status 2 Command: return temperature, iq current(-2048~2048 mapping-32A to 32A), speed (1dps/LSB), encoder (0~16383)
        temperature = (int8_t)msgR2.buf[1];
        iq          = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A        = ((double)iq) * 32 / 2048;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder     = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
               Serial.println("successful to read motor status 2");
        //        Serial.print("temperature is ");
        //        Serial.print(temperature);
        //        Serial.print("; iq_A is ");
        //        Serial.print(iq_A);
        //        Serial.print("; speed_value is ");
        //        Serial.print(speed_value);
        // Serial.print("; encoder is ");
        //        Serial.println(encoder);
        break;
      case 0x9D: //15: read motor status 3 return temperature, phase A B C current
        temperature = (int8_t)msgR2.buf[1];
        iA = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iA_A = ((double)iA) / 64;
        iB = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        iB_A = ((double)iB) / 64;
        iC = (int16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        iC_A = ((double)iC) / 64;
               Serial.println("successful to read motor status 3");
        //        Serial.print("temperature is ");
        //        Serial.print(temperature);
        //        Serial.print("; iA_A is ");
        //        Serial.print(iA_A);
        //        Serial.print("; iB_A is ");
        //        Serial.print(iB_A);
        //        Serial.print("; iC_A is ");
        //        Serial.println(iC_A);
        break;

      case 0x80: // Motor Shutdown Command
        Serial.println("Successful: Motor Shutdown Command()");
        break;

      case 0x81: // Motor Stop Command
        Serial.println("Successful: Motor Stop Command()");
        break;
      case 0x88: //18: start motor
        //        Serial.println("successful to start motor and resume previous control method");
        break;
      case 0xA1: //19: send current command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 32 / 2048;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA2: //20: send speed command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 32 / 2048;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      case 0xA3: //21: send position command and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 32 / 2048;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      
      case 0xA4: // Absolute Position Closed-Loop Control Command
        temperature = (int8_t)msgR2.buf[1];
        iq          = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A        = ((double)iq) * 32 / 2048;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder     = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        // Serial.println(encoder);
        break;
      
      case 0xA6: //23: send position command 3(single turn command) and return temperature, iq, speed, encoder
        temperature = (int8_t)msgR2.buf[1];
        iq = (int16_t)(((uint16_t)msgR2.buf[3] << 8) | ((uint16_t)msgR2.buf[2]));
        iq_A = ((double)iq) * 32 / 2048;
        speed_value_int16 = (int16_t)(((uint16_t)msgR2.buf[5] << 8) | ((uint16_t)msgR2.buf[4]));
        speed_value = (double)speed_value_int16;
        speed_value = speed_value / gear_ratio;
        encoder = (uint16_t)(((uint16_t)msgR2.buf[7] << 8) | ((uint16_t)msgR2.buf[6]));
        break;
      default:
        break;
    }
  }
}
/////////////////////////////////////////////////////////////////////

//******1.Read PID Data command******//
void H50_motor_lib::read_PID_gains()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x30;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  for (int i = 0; i < sizeof(indexes)/sizeof(indexes[0]); i++) {
    msgW.buf[1] = indexes[i];
    Can3.write(msgW);
    delay(10); // less than 3 ms does not work
    receive_CAN_data();
  }
}
//******2.Write PID gain to RAM******//
void H50_motor_lib::write_PID_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x31;
  msgW.buf[1] = 0;
  msgW.buf[2] = anglePidKp;
  msgW.buf[3] = anglePidKi;
  msgW.buf[4] = speedPidKp;
  msgW.buf[5] = speedPidKi;
  msgW.buf[6] = iqPidKp;
  msgW.buf[7] = iqPidKi;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******3.Write PID gain to ROM******//
void H50_motor_lib::write_PID_ROM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x32;
  msgW.buf[1] = 0;
  msgW.buf[2] = anglePidKp;
  msgW.buf[3] = anglePidKi;
  msgW.buf[4] = speedPidKp;
  msgW.buf[5] = speedPidKi;
  msgW.buf[6] = iqPidKp;
  msgW.buf[7] = iqPidKi;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******5.Write Acceleration RAM******//
void H50_motor_lib::write_acceleration_RAM()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x34;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&Accel);
  msgW.buf[5] = *((uint8_t*)(&Accel) + 1);
  msgW.buf[6] = *((uint8_t*)(&Accel) + 2);
  msgW.buf[7] = *((uint8_t*)(&Accel) + 3);
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

/*[0x42] Read Acceleration Command.
Read the acceleration parameters of the current motor.
*/
void H50_motor_lib::read_acceleration()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x42;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

//******2.11. Read Single-Turn Encoder Command******//
void H50_motor_lib::read_single_turn_encoder()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x90;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}
//******7.Write ENCODER OFFSET ROM******//
void H50_motor_lib::write_encoder_offset_RAM(uint16_t encoder_Offset )
{
  encoderOffset = encoder_Offset;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x91;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = *(uint8_t*)(&encoderOffset);
  msgW.buf[7] = *((uint8_t*)(&encoderOffset) + 1);
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
//******8.Write current postioton as Zero degree******//
void H50_motor_lib::write_current_position_as_zero_position()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x19;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}
void H50_motor_lib::read_multi_turn_encoder_pos() // Read Multi-Turn Encoder Position Data Command (0x60)
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x60;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

/* [0x64] Write the Current Multi-Turn Position of the Encoder to the ROM as the Motor Zero Command.
Write the current encoder position of the motor as the multi-turn encoder zero offset (initial position) into the ROM.
Note: After writing the new zero point position,you need to send 0x76 (system reset
command) to restart the system to be effective. Because of the change of the zero
offset,the new zero offset (initial position) should be used as a reference when setting
the target position.*/
void H50_motor_lib::write_multi_turn_encoder_pos_to_ROM_as_zero()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x64;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(100);
  receive_CAN_data();
  motor_reset();
}

/*[0x76] This command is used to reset the system program.*/
void H50_motor_lib::motor_reset()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x76;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}

/*[0x77] System Brake Release Command.*/
void H50_motor_lib::motor_brake_release()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x77;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

/*[0x92] Read Multi-Turn Angle Command.
read the current multi-turn absolute angle value of the motor.
Motor angle motorAngle, (int32_t type, value range, valid data 4 bytes), unit 0.01°/LSB.*/
void H50_motor_lib::read_multi_turn_angle()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();//////////////////////////////////////////////////////added
}

/*[0x94] Read Single-Turn Angle Command.
Read the current single-turn angle of the motor.
The single circle angle of the motor, circleAngle is int16_t type data, starting
from the zero point of the encoder, increasing clockwise, and returning to 0 when
it reaches the zero point again, the unit is 0.01°/LSB, and the value range is 0~35999.
*/
void H50_motor_lib::read_single_turn_angle()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x94;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  // delay(10);
  // receive_CAN_data();
}
//(current cannot use it)******11.clear all angle command and offset currnet position as zero command (unit 0.01deg/LSB)******//
void H50_motor_lib::clear_motor_angle_command()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x95;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
  //receive_CAN_data();
}

//******13.clear motot error and reset motor******//
void H50_motor_lib::clear_motor_error()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9B;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //  delay(1);
}

/*[0x9A] Read Motor Status 1 and Error Flag Command.
This command reads the current motor temperature, voltage and error status flags.
1. Motor temperature temperature (int8_t type,unit 1℃/LSB)
2. Brake control command: Indicates the state of the brake control command, 1 represents the brake release command, and 0 represents the brake lock command
3. Voltage (uint16_t type,unit 0.1V/LSB)
4. Error flag errorState (of type uint16_t,each bit represents a different motor state)*/
void H50_motor_lib::read_motor_status_1()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9A;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(50);
  receive_CAN_data();
}

//******14.read motor status 2 (temperature 1degreeC/LSB, iq(-2048~2048 mapping to -32A ~32A), speed(1dps/LSB), 14 bit encoder value(0~16383))******//
void H50_motor_lib::read_motor_status_2()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9C;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(50);
  receive_CAN_data();
}
//******15.read motor status 3 (temperature 1degreeC/LSB,A phase current(1A/64LSB),B phase current(1A/64LSB),C phase current(1A/64LSB) )******//
void H50_motor_lib::read_motor_status_3()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x9D;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(50);
  receive_CAN_data();
}

/*[0x80] Motor Shutdown Command.
Turns off the motor output and also clears the motor running state, not in any closed loop mode.
*/
void H50_motor_lib::motor_shutdown()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x80;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}

/* [0x81] Motor Stop Command.
Stop the motor,the closed-loop mode where the motor is still running,just stop the motor speed.
*/
void H50_motor_lib::motor_stop()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x81;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
}
//******18.start motor******//
void H50_motor_lib::start_motor()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0X88;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  if (Can3.write(msgW)) {
    char buffer[50];
    sprintf(buffer, "start_motor(): Successful - Motor ID: %d", ID);
    Serial.println(buffer);
  }
  else {
    Serial.println("Fail to send start motor command");
  }
   delay(10);
  receive_CAN_data();
}
//******19.current control: send current command current unit A(not limited by maximum Torque Current)******//
void H50_motor_lib::send_current_command(double current)
{
  current = current * 2000 / 32;
  iqControl = (int16_t)current;

  iqControli1 = iqControl >> 8;
  iqControli2 = iqControl & 0xFF;


  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA1;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&iqControl);
  msgW.buf[5] = *((uint8_t*)(&iqControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}
//******20.speed control: send speed command speed unit dps******//
void H50_motor_lib::send_speed_command(double speedvalue)
{
  speedvalue = speedvalue * 100 * gear_ratio;
  speedControl = (int32_t)speedvalue;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA2;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&speedControl);
  msgW.buf[5] = *((uint8_t*)(&speedControl) + 1);
  msgW.buf[6] = *((uint8_t*)(&speedControl) + 2);
  msgW.buf[7] = *((uint8_t*)(&speedControl) + 3);
  Can3.write(msgW);
}
//******21.position control:send position command (angle unit degree)******//
void H50_motor_lib::send_position_command(double angle)
{
  angle = angle * 100;
  angleControl = (int32_t)(angle);
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA3;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = 0x00;
  msgW.buf[3] = 0x00;
  msgW.buf[4] = (uint8_t)(angleControl);
  msgW.buf[5] = (uint8_t)(angleControl >> 8);
  msgW.buf[6] = (uint8_t)(angleControl >> 16);
  msgW.buf[7] = (uint8_t)(angleControl >> 24);
  Can3.write(msgW);
}

/* [0xA4] Absolute Position Closed-Loop Control Command
The control value angleControl is int32_t type, and the corresponding actual position is 0.01 degree/LSB, that is, 36000 represents 360°, and the rotation direction of the motor is
determined by the difference between the target position and the current position.
The control value maxSpeed limits the maximum speed of the motor output shaft rotation, which is of type uint16_t, corresponding to the actual speed of 1 dps/LSB.
According to the position planning acceleration value set by the system, different operating modes will be different:
1. If the position loop acceleration is 0, then the position loop will enter the direct tracking mode,and directly track the target position through the PI controller. Among
them, maxSpeed limits the maximum speed during the position operation process. If the maxSpeed value is 0, then it is completely output by the calculation result of the PI
controller.
2. If the position loop acceleration is not 0, then the motion mode with speed planning will be run,and the motor will complete the acceleration and deceleration process.
The maximum operating speed is determined by maxSpeed, and the acceleration is determined by the acceleration set by the position loop.
*/
void H50_motor_lib::send_position_command_2(double angle, double max_speed)
{
  angle = angle * 100;
  angleControl = (int32_t) angle;
  maxiSpeed = (int16_t) max_speed;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA4;
  msgW.buf[1] = 0x00;
  msgW.buf[2] = (uint8_t)(maxiSpeed);
  msgW.buf[3] = (uint8_t)(maxiSpeed >> 8);
  msgW.buf[4] = (uint8_t)(angleControl);
  msgW.buf[5] = (uint8_t)(angleControl >> 8);
  msgW.buf[6] = (uint8_t)(angleControl >> 16);
  msgW.buf[7] = (uint8_t)(angleControl >> 24);
  Can3.write(msgW);
  delay(5);
  // receive_CAN_data();
}
//******23.position control 3:send single-turn position command (angle unit degree 0~359.99)
//(limited by maximum acceleration unit)******//
//(limited by maximum speed unit 1dps)******//
//(limited by maximum angle)//
//(limited by maximum current)//
void H50_motor_lib::send_position_command_3(double angle, uint8_t spinDirection)
{
  angle = angle * 100;
  angleControl = (uint8_t)angle;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA6;
  msgW.buf[1] = spinDirection;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}
//******24.position control 4:send single-turn position command (angle unit degree 0~359.99)
//(limited by maximum acceleration unit)******//
//(limited by maximum speed unit 1dps)******//
//(limited by maximum angle)//
//(limited by maximum current)//
void H50_motor_lib::send_position_command_4(double angle, double max_speed, uint8_t spinDirection)
{
  angle = angle * 100;
  angleControl = (uint8_t)angle;
  maxiSpeed = (uint8_t)max_speed;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA6;
  msgW.buf[1] = spinDirection;
  msgW.buf[2] = *(uint8_t*)(&maxiSpeed);
  msgW.buf[3] = *(uint8_t*)(&maxiSpeed);
  msgW.buf[4] = *(uint8_t*)(&angleControl);
  msgW.buf[5] = *((uint8_t*)(&angleControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
}
//******25.multi-motors current control
//ID must be #1~#4 for four motor
void H50_motor_lib::send_multi_motor_current_command(double Motor1_current, double Motor2_current, double Motor3_current, double Motor4_current)
{
  iqControl_1 = (int16_t)Motor1_current;
  iqControl_2 = (int16_t)Motor2_current;
  iqControl_3 = (int16_t)Motor3_current;
  iqControl_4 = (int16_t)Motor4_current;
  msgW.id = 0x280;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = *(uint8_t *)(&iqControl_1);
  msgW.buf[1] = *((uint8_t *)(&iqControl_1) + 1);
  msgW.buf[2] = *(uint8_t *)(&iqControl_2);
  msgW.buf[3] = *((uint8_t *)(&iqControl_2) + 1);
  msgW.buf[4] = *(uint8_t *)(&iqControl_3);
  msgW.buf[5] = *((uint8_t *)(&iqControl_3) + 1);
  msgW.buf[6] = *(uint8_t *)(&iqControl_4);
  msgW.buf[7] = *((uint8_t *)(&iqControl_4) + 1);
  Can3.write(msgW);
}
void H50_motor_lib::receive_CAN_data()
{
  if (Can3.read(msgR))
  {
    Can3.read(msgR);
    DataExplanation(msgR);
  }
}


void H50_motor_lib::send_current_command_for36(double current)
{
  current = current * 2000 / 32;
  iqControl = (int16_t)current;
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xA1;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = *(uint8_t*)(&iqControl);
  msgW.buf[5] = *((uint8_t*)(&iqControl) + 1);
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //delay
}

void H50_motor_lib::read_multi_turn_angle_for36()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0x92;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = 0;
  Can3.write(msgW);
  //delay(1);
  //receive_CAN_data();
}

/*[0xB4] Communication Baud Rate Setting Command.
This instruction can set the communication baud rate of CAN and RS485 bus.
The parameters will be saved in ROM after setting,and will be saved after power off, and will run at the modified baud rate when powered on again.
CAN: 0 stands for 500Kbps baud rate, 1 stands for 1Mbps baud rate.
*/
void H50_motor_lib::comm_baud_rate_set()
{
  msgW.id = 0x140 + ID;
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;
  msgW.buf[0] = 0xB4;
  msgW.buf[1] = 0;
  msgW.buf[2] = 0;
  msgW.buf[3] = 0;
  msgW.buf[4] = 0;
  msgW.buf[5] = 0;
  msgW.buf[6] = 0;
  msgW.buf[7] = (uint8_t)(1); // 0 = 500Kbps | 1 = 1Mbps
  Can3.write(msgW);
  delay(10);
  receive_CAN_data();
  Serial.println("Baud rate = 1Mbps");
}
