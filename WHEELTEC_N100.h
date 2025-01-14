#ifndef WHEELTEC_N100_H
#define WHEELTEC_N100_H

#include "Arduino.h"
#define WHEELTEC_N100_BAUDRATE 921600 // Use WHEELTEC N100 COM1's default baudrate

// N100資料格式巨集定義
#define IMU_LEN         0x38 // IMU資料長度
#define AHRS_LEN        0x30 // AHRS資料長度
#define EULER_ORIEN_LEN 0x0C // EULER_ORIEN資料長度
#define BODY_VEL_LEN    0x0C // BODY_VEL資料長度 my own
#define ANGULAR_VEL_LEN 0x0C // ANGULAR_VEL資料長度 my own

#define FRAME_HEAD        0xFC // 資料幀頭
#define FRAME_END         0xFD // 資料幀尾

#define TYPE_IMU          0x40 // IMU資料類別
#define TYPE_AHRS         0x41 // AHRS資料類別
#define TYPE_EULER_ORIEN  0x63 // EULER_ORIEN資料類別
#define TYPE_BODY_VEL     0x60 // BODY_VEL資料類別 my own
#define TYPE_ANGULAR_VEL  0x66 // ANGULAR_VEL資料類別 my own

#define IMU_TYPE_LEN          64 // 當資料類型為IMU時，資料的總長度
#define AHRS_TYPE_LEN         56 // 當資料類型為AHRS時，資料的總長度
#define EULER_ORIEN_TYPE_LEN  20 // 當資料類型為EULER_ORIEN時，資料的總長度
#define BODY_VEL_TYPE_LEN     20 // 當資料類型為BODY_VEL時，資料的總長度 my own
#define ANGULAR_VEL_TYPE_LEN  20 // 當資料類型為ANGULAR_VEL時，資料的總長度 my own

class WHEELTEC_N100
{
public:
  WHEELTEC_N100();
  WHEELTEC_N100(int serial_port, int baudrate);
  void Serial_Init();
  void Read_Data();
  void Unpack_Data();
  void Get_Init_POS(int sample_length = 200);

  void Print_IMU_Packet();
  void Print_AHRS_Packet();
  void Print_RPY_Packet();
  void Print_VEL_Packet();  //my own
  void Print_AVEL_Packet();  //my own

  void Print_POS_RPY();
  void Print_Init_POS_RPY();
  void Print_AVEL_Deg();  //my own
  void PrintPacketBytes(const uint8_t* packet, size_t length);//my own
  uint8_t CRC8_Table(uint8_t* p, uint8_t counter); //my own
  uint16_t CRC16_Table(uint8_t* p, uint8_t counter); //my own


  typedef struct
  { /* data */
    float gyroscope_x = 0.0;          // [rad/s]
    float gyroscope_y = 0.0;          // [rad/s]
    float gyroscope_z = 0.0;          // [rad/s]
    float accelerometer_x = 0.0;      // [m/s^2]
    float accelerometer_y = 0.0;      // [m/s^2]
    float accelerometer_z = 0.0;      // [m/s^2]
    float magnetometer_x = 0.0;       // [mG]
    float magnetometer_y = 0.0;       // [mG]
    float magnetometer_z = 0.0;       // [mG]
    float imu_temperature = 0.0;      // [C]
    float Pressure = 0.0;             // [Pa]
    float pressure_temperature = 0.0; // [C]
    uint32_t Timestamp = 0;           // [us]
  }IMU_Packet;

  typedef struct
  { /* data */
    float RollSpeed = 0.0;    // [rad/s]
    float PitchSpeed = 0.0;   // [rad/s]
    float HeadingSpeed = 0.0; // [rad/s]
    float Roll = 0.0;         // [rad]
    float Pitch = 0.0;        // [rad]
    float Heading = 0.0;      // [rad]
    float Qw = 0.0;  //w      // Quaternion
    float Qx = 0.0;  //x
    float Qy = 0.0;  //y
    float Qz = 0.0;  //z
    uint32_t Timestamp = 0;   // [us]
  }AHRS_Packet;

  typedef struct
  { /* data */
    float Roll = 0.0;   // [rad]
    float Pitch = 0.0;  // [rad]
    float Yaw = 0.0;    // [rad]
  }RPY_Packet;
//my own
  typedef struct
  { /* data */
    float Vel_x = 0.0;   // [m/s]
    float Vel_y = 0.0;   // [m/s]
    float Vel_z = 0.0;   // [m/s]
  }VEL_Packet;
//my own
  typedef struct
  { /* data */
    float AVel_x = 0.0;   // [rad/s]
    float AVel_y = 0.0;   // [rad/s]
    float AVel_z = 0.0;   // [rad/s]
  }AVEL_Packet;
//my own
  typedef struct
  { /* data */
    float AVel_x = 0.0;   // [deg/s]
    float AVel_y = 0.0;   // [deg/s]
    float AVel_z = 0.0;   // [deg/s]
  }AVEL_Deg;

  typedef struct
  { /* data */
    float Roll = 0.0;   // [deg]
    float Pitch = 0.0;  // [deg]
    float Yaw = 0.0;    // [deg]
  }RPY_Deg;

  IMU_Packet PacketIMU;
  AHRS_Packet PacketAHRS;
  RPY_Packet PacketRPY;
  VEL_Packet PacketVEL; //my own
  AVEL_Packet PacketAVEL; //my own
  
  RPY_Deg POS_RPY;
  RPY_Deg Init_POS_RPY;
  AVEL_Deg Deg_AVEL; //my own
  
private:
  float _HEX_to_Float(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
  float _HEX_to_Float_New(uint8_t *data, bool mode);
  long long _timestamp(uint8_t Data_1, uint8_t Data_2, uint8_t Data_3, uint8_t Data_4);
  Stream* N100_SERIAL = &Serial2;
  int _serial_port = 0;     // N100使用的serial port
  int _baudrate = WHEELTEC_N100_BAUDRATE; // N100設定的baudrate

  // 讀取資料需要用到的相關變量
  uint8_t IMU_Data[IMU_TYPE_LEN];                 // IMU資料類型長度
  uint8_t AHRS_Data[AHRS_TYPE_LEN];               // AHRS資料類型長度
  uint8_t EULER_ORIEN_Data[EULER_ORIEN_TYPE_LEN]; // EULER_ORIEN資料類型長度
  uint8_t BODY_VEL_Data[BODY_VEL_TYPE_LEN];       // BODY_VEL資料類型長度 my own
  uint8_t ANGULAR_VEL_Data[ANGULAR_VEL_TYPE_LEN];       // BODY_VEL資料類型長度 my own
  
  uint8_t Fd_data[64];                            // 用於存放接收串口數據
  
  bool Data_of_IMU = 0;                           // 用於表示IMU資料已經就緒，可以進行解封包
  bool Data_of_AHRS = 0;                          // 用於表示AHRS資料已經就緒，可以進行解封包
  bool Data_of_EULER_ORIEN = 0;                   // 用於表示EULER_ORIEN資料已經就緒，可以進行解封包
  bool Data_of_BODY_VEL = 0;                      // 用於表示BODY_VEL資料已經就緒，可以進行解封包 my own
  bool Data_of_ANGULAR_VEL = 0;                      // 用於表示BODY_VEL資料已經就緒，可以進行解封包 my own
};

#endif

