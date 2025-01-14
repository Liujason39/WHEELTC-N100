# WHEELTC-N100
Arduino library for IMU(Wheeltec N100)

//example code (IMU_test.ino)
#include "WHEELTEC_N100.h"

#define Hz_to_ms 1000000 // 1 Hz = 1000000 ms
#define IMU_freq 3000     // [Hz] = times of execution each seconds
#define print_freq 10     // [Hz] = times of execution each seconds

WHEELTEC_N100 IMU(2,921600);
IntervalTimer IMU_timer,Print_timer;

void setup() {
  // put your setup code here, to run once:
  IMU.Serial_Init();
  Serial.begin(115200);
    delay(2500);
    // IMU.Get_Init_POS();
    // IMU.Print_Init_POS_RPY();
    // delay(5000);
    IMU_timer.begin(IMU_receive, Hz_to_ms/IMU_freq);  //(function, microseconds)
    Print_timer.begin(IMU_print, Hz_to_ms/print_freq);  //(function, microseconds)
  }

void loop(){
  // put your main code here, to run repeatedly:
  // IMU.Read_Data();
  // IMU.Unpack_Data();
  // IMU.Print_RPY();
}

void IMU_receive(){
  // IMU.Read_Data();
  // IMU.Unpack_Data();
  // IMU.Print_POS_RPY();
  IMU.Read_Data();
}

void IMU_print(){
  // IMU.Read_Data();
  // IMU.Unpack_Data();
  // IMU.Print_POS_RPY();
  IMU.Print_RPY_Packet();
  IMU.Print_VEL_Packet();
  IMU.Print_AVEL_Packet();
}

