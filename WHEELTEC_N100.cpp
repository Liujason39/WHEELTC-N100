#include "Arduino.h"
#include "WHEELTEC_N100.h"

WHEELTEC_N100::WHEELTEC_N100(){
  _serial_port = 2;
  _baudrate = WHEELTEC_N100_BAUDRATE;
}

WHEELTEC_N100::WHEELTEC_N100(int serial_port, int baudrate){
  _serial_port = serial_port;
  _baudrate = baudrate;
}

void WHEELTEC_N100::Serial_Init(){
  if(_serial_port == 1){
    N100_SERIAL = &Serial1;
    Serial1.begin(_baudrate);
  }
  else if(_serial_port == 2){
    N100_SERIAL = &Serial2;
    Serial2.begin(_baudrate);
  }
  else if(_serial_port == 3){
    N100_SERIAL = &Serial3;
    Serial3.begin(_baudrate);
    /* If using "Motor Communication Control" PCB board in NTU ASR-LAB,
    the RS485_2 is using Serial3 for communication,
    and use GPIO-13 to switch between TX and RX. */
    Serial3.transmitterEnable(13); 
  }
  else if(_serial_port == 4){
    N100_SERIAL = &Serial4;
    Serial4.begin(_baudrate);
  }
  else if(_serial_port == 5){
    N100_SERIAL = &Serial5;
    Serial5.begin(_baudrate);
    /* If using "Motor Communication Control" PCB board in NTU ASR-LAB,
    the RS485_1 is using Serial5 for communication,
    and use GPIO-2 to switch between TX and RX. */
    Serial5.transmitterEnable(2);
  }
  else if(_serial_port == 6){
    N100_SERIAL = &Serial6;
    Serial6.begin(_baudrate);
  }
  else if(_serial_port == 7){
    N100_SERIAL = &Serial7;
    Serial7.begin(_baudrate);
  }
  else{
    Serial.println("===== Serial Port Error! =====");
  }
}

// 讀取N100模組資料函數
void WHEELTEC_N100::Read_Data(){
  static uint8_t Count = 0;    // 用於計算目前所獲得的資料量(bytes)
  uint8_t Usart_Receive;       // 用於讀取串口接收到的資料
  static uint8_t Last_Receive; // 用於保存上一次的接收

  while(N100_SERIAL->available()){  // 串口接收到資料
    Usart_Receive = N100_SERIAL->read(); // 讀取串口的資料
    Fd_data[Count] = Usart_Receive;

    // 接收上一幀的幀尾和本幀幀頭則開始計數（較大程度避免資料誤讀浪費時間，造成資料丟包）
    if((Last_Receive==FRAME_END && Usart_Receive==FRAME_HEAD) || Count>0)
      Count++;
    else
      Count = 0;

    Last_Receive = Usart_Receive; // 保存本次數據
    
    // 滿足IMU數據長度
    if(Count==IMU_TYPE_LEN){
      // 資料類型、長度、幀尾均符合要求
      if(Fd_data[1]==TYPE_IMU && Fd_data[2]==IMU_LEN && Fd_data[IMU_TYPE_LEN-1]==FRAME_END){
        Count = 0;// 清空計數等待下次計數
        Data_of_IMU = 1; //標亮對應的指示值
        memcpy(IMU_Data, Fd_data, sizeof(IMU_Data));
        Unpack_Data();
      }
    }

    // 滿足AHRS數據長度
    if(Count==AHRS_TYPE_LEN){
      if(Fd_data[1]==TYPE_AHRS && Fd_data[2]== AHRS_LEN && Fd_data[AHRS_TYPE_LEN-1]==FRAME_END){
        Count = 0;// 清空計數等待下次計數
        Data_of_AHRS = 1; //標亮對應的指示值
        memcpy(AHRS_Data, Fd_data, sizeof(AHRS_Data));
        Unpack_Data();
      }
    }

    // 滿足EULER_ORIEN數據長度
    if(Count==EULER_ORIEN_TYPE_LEN){
      if(Fd_data[1]==TYPE_EULER_ORIEN && Fd_data[2]== EULER_ORIEN_LEN && Fd_data[EULER_ORIEN_TYPE_LEN-1]==FRAME_END){
        Count = 0;// 清空計數等待下次計數
        Data_of_EULER_ORIEN = 1; //標亮對應的指示值
        memcpy(EULER_ORIEN_Data, Fd_data, sizeof(EULER_ORIEN_Data));
        Unpack_Data();

        // //myown
        // Serial.print("EULER_ORIEN \t");
        // PrintPacketBytes(Fd_data, sizeof(Fd_data));
        // //print crc8
        // uint8_t crc8 = CRC8_Table(Fd_data, 4);
        // Serial.print("EULER crc8: \t");
        // Serial.println(crc8, HEX);
        // // //print crc16
        // uint8_t d16[12];
        // memcpy(d16, &Fd_data[7], 12); // 複製 12 個字節，從第 7 個開始
        // uint16_t crc16 = CRC16_Table(d16, 12);
        // Serial.print("EULER crc16: \t");
        // Serial.println(crc16, HEX);
      }
    }

    // 滿足BODY_VEL數據長度 my own
    if(Count==BODY_VEL_TYPE_LEN){
      if(Fd_data[1]==TYPE_BODY_VEL && Fd_data[2]== BODY_VEL_LEN && Fd_data[BODY_VEL_TYPE_LEN-1]==FRAME_END){
        Count = 0;// 清空計數等待下次計數
        Data_of_BODY_VEL = 1; //標亮對應的指示值
        memcpy(BODY_VEL_Data, Fd_data, sizeof(BODY_VEL_Data));
        Unpack_Data();

        // //myown
        // Serial.print("BODY_VEL \t");
        // PrintPacketBytes(Fd_data, sizeof(Fd_data));
        // //print crc8
        // uint8_t crc8 = CRC8_Table(Fd_data, 4);
        // Serial.print("B_VEL crc8: \t");
        // Serial.println(crc8, HEX);
        // // //print crc16
        // uint8_t d16[12];
        // memcpy(d16, &Fd_data[7], 12); // 複製 12 個字節，從第 7 個開始
        // uint16_t crc16 = CRC16_Table(d16, 12);
        // Serial.print("B_VEL crc16: \t");
        // Serial.println(crc16, HEX);

      }
    }

    // 滿足ANGULAR_VEL數據長度 my own
    if(Count==ANGULAR_VEL_TYPE_LEN){
      if(Fd_data[1]==TYPE_ANGULAR_VEL && Fd_data[2]== ANGULAR_VEL_LEN && Fd_data[ANGULAR_VEL_TYPE_LEN-1]==FRAME_END){
        Count = 0;// 清空計數等待下次計數
        Data_of_ANGULAR_VEL = 1; //標亮對應的指示值
        memcpy(ANGULAR_VEL_Data, Fd_data, sizeof(ANGULAR_VEL_Data)); 
        Unpack_Data();

        // //myown
        // Serial.print("ANGULAR_VEL \t");
        // PrintPacketBytes(Fd_data, sizeof(Fd_data));
        // //print crc8
        // uint8_t crc8 = CRC8_Table(Fd_data, 4);
        // Serial.print("A_VEL crc8: \t");
        // Serial.println(crc8, HEX);
        // // //print crc16
        // uint8_t d16[12];
        // memcpy(d16, &Fd_data[7], 12); // 複製 12 個字節，從第 7 個開始
        // uint16_t crc16 = CRC16_Table(d16, 12);
        // Serial.print("A_VEL crc16: \t");
        // Serial.println(crc16, HEX);
      
      }
    }

    // 均不符合要求，資料超出長度，放棄計數等待下次接收
    if(Count > IMU_TYPE_LEN) Count = 0;
  }
  
}

// 数据解封包函数
void WHEELTEC_N100::Unpack_Data(){
  if(Data_of_IMU == 1){ // IMU資料接收完畢，開始解封包
    if(IMU_Data[1]==TYPE_IMU && IMU_Data[2]==IMU_LEN){ // 再次校驗資料類型和資料長度
      uint8_t temp[4];
      temp[0] = IMU_Data[7], temp[1] = IMU_Data[8], temp[2] = IMU_Data[9], temp[3] = IMU_Data[10];
      PacketIMU.gyroscope_x = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[11], temp[1] = IMU_Data[12], temp[2] = IMU_Data[13], temp[3] = IMU_Data[14];
      PacketIMU.gyroscope_y = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[15], temp[1] = IMU_Data[16], temp[2] = IMU_Data[17], temp[3] = IMU_Data[18];
      PacketIMU.gyroscope_z = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[19], temp[1] = IMU_Data[20], temp[2] = IMU_Data[21], temp[3] = IMU_Data[22];
      PacketIMU.accelerometer_x = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[23], temp[1] = IMU_Data[24], temp[2] = IMU_Data[25], temp[3] = IMU_Data[26];
      PacketIMU.accelerometer_y = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[27], temp[1] = IMU_Data[28], temp[2] = IMU_Data[29], temp[3] = IMU_Data[30];
      PacketIMU.accelerometer_z = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[31], temp[1] = IMU_Data[32], temp[2] = IMU_Data[33], temp[3] = IMU_Data[34];
      PacketIMU.magnetometer_x = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[35], temp[1] = IMU_Data[36], temp[2] = IMU_Data[37], temp[3] = IMU_Data[38];
      PacketIMU.magnetometer_y = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[39], temp[1] = IMU_Data[40], temp[2] = IMU_Data[41], temp[3] = IMU_Data[42];
      PacketIMU.magnetometer_z = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[43], temp[1] = IMU_Data[44], temp[2] = IMU_Data[45], temp[3] = IMU_Data[46];
      PacketIMU.imu_temperature = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[47], temp[1] = IMU_Data[48], temp[2] = IMU_Data[49], temp[3] = IMU_Data[50];
      PacketIMU.Pressure = _HEX_to_Float_New(temp, true);

      temp[0] = IMU_Data[51], temp[1] = IMU_Data[52], temp[2] = IMU_Data[53], temp[3] = IMU_Data[54];
      PacketIMU.pressure_temperature = _HEX_to_Float_New(temp, true);

      PacketIMU.Timestamp = _timestamp(IMU_Data[58], IMU_Data[57], IMU_Data[56], IMU_Data[55]);
     
      // 資料解包完畢，清空標誌等待下次資料解包
      // memset(temp, 0, sizeof(temp));
      Data_of_IMU = 0;
    }
  }

  if(Data_of_AHRS==1){ // AHRS資料接收完畢，開始解封包
    if(AHRS_Data[1]==TYPE_AHRS && AHRS_Data[2]==AHRS_LEN){ // 再次校驗資料類型和資料長度
      uint8_t temp[4];
      temp[0] = AHRS_Data[7], temp[1] = AHRS_Data[8], temp[2] = AHRS_Data[9], temp[3] = AHRS_Data[10];
      PacketAHRS.RollSpeed = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[11], temp[1] = AHRS_Data[12], temp[2] = AHRS_Data[13], temp[3] = AHRS_Data[14];
      PacketAHRS.PitchSpeed = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[15], temp[1] = AHRS_Data[16], temp[2] = AHRS_Data[17], temp[3] = AHRS_Data[18];
      PacketAHRS.HeadingSpeed = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[19], temp[1] = AHRS_Data[20], temp[2] = AHRS_Data[21], temp[3] = AHRS_Data[22];
      PacketAHRS.Roll = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[23], temp[1] = AHRS_Data[24], temp[2] = AHRS_Data[25], temp[3] = AHRS_Data[26];
      PacketAHRS.Pitch = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[27], temp[1] = AHRS_Data[28], temp[2] = AHRS_Data[29], temp[3] = AHRS_Data[30];
      PacketAHRS.Heading = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[31], temp[1] = AHRS_Data[32], temp[2] = AHRS_Data[33], temp[3] = AHRS_Data[34];
      PacketAHRS.Qw = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[35], temp[1] = AHRS_Data[36], temp[2] = AHRS_Data[37], temp[3] = AHRS_Data[38];
      PacketAHRS.Qx = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[39], temp[1] = AHRS_Data[40], temp[2] = AHRS_Data[41], temp[3] = AHRS_Data[42];
      PacketAHRS.Qy = _HEX_to_Float_New(temp, true);

      temp[0] = AHRS_Data[43], temp[1] = AHRS_Data[44], temp[2] = AHRS_Data[45], temp[3] = AHRS_Data[46];
      PacketAHRS.Qz = _HEX_to_Float_New(temp, true);
      
      PacketAHRS.Timestamp = _timestamp(AHRS_Data[50], AHRS_Data[49], AHRS_Data[48], AHRS_Data[47]); // unit: us

      // 資料解包完畢，清空標誌等待下次資料解包
      // memset(temp, 0, sizeof(temp));
      Data_of_AHRS = 0;
    }
  }

  if(Data_of_EULER_ORIEN==1){ // EULER_ORIEN資料接收完畢，開始解封包
    if(EULER_ORIEN_Data[1]==TYPE_EULER_ORIEN && EULER_ORIEN_Data[2]==EULER_ORIEN_LEN){ // 再次校驗資料類型和資料長度
      uint8_t temp[4];
      temp[0] = EULER_ORIEN_Data[7], temp[1] = EULER_ORIEN_Data[8], temp[2] = EULER_ORIEN_Data[9], temp[3] = EULER_ORIEN_Data[10];
      PacketRPY.Roll = _HEX_to_Float_New(temp, true);
      POS_RPY.Roll = degrees(PacketRPY.Roll);

      temp[0] = EULER_ORIEN_Data[11], temp[1] = EULER_ORIEN_Data[12], temp[2] = EULER_ORIEN_Data[13], temp[3] = EULER_ORIEN_Data[14];
      PacketRPY.Pitch = _HEX_to_Float_New(temp, true);
      POS_RPY.Pitch = degrees(PacketRPY.Pitch);

      temp[0] = EULER_ORIEN_Data[15], temp[1] = EULER_ORIEN_Data[16], temp[2] = EULER_ORIEN_Data[17], temp[3] = EULER_ORIEN_Data[18];
      PacketRPY.Yaw = _HEX_to_Float_New(temp, true);
      POS_RPY.Yaw = degrees(PacketRPY.Yaw);

      // 資料解包完畢，清空標誌等待下次資料解包
      // memset(temp, 0, sizeof(temp));
      Data_of_EULER_ORIEN = 0;
    }
  }

  if(Data_of_BODY_VEL==1){ // BODY_VEL資料接收完畢，開始解封包 my own
    if(BODY_VEL_Data[1]==TYPE_BODY_VEL && BODY_VEL_Data[2]==BODY_VEL_LEN){ // 再次校驗資料類型和資料長度
      uint8_t temp[4];
      temp[0] = BODY_VEL_Data[7], temp[1] = BODY_VEL_Data[8], temp[2] = BODY_VEL_Data[9], temp[3] = BODY_VEL_Data[10];
      PacketVEL.Vel_x = _HEX_to_Float_New(temp, true);

      temp[0] = BODY_VEL_Data[11], temp[1] = BODY_VEL_Data[12], temp[2] = BODY_VEL_Data[13], temp[3] = BODY_VEL_Data[14];
      PacketVEL.Vel_y = _HEX_to_Float_New(temp, true);

      temp[0] = BODY_VEL_Data[15], temp[1] = BODY_VEL_Data[16], temp[2] = BODY_VEL_Data[17], temp[3] = BODY_VEL_Data[18];
      PacketVEL.Vel_z = _HEX_to_Float_New(temp, true);

      // 資料解包完畢，清空標誌等待下次資料解包
      // memset(temp, 0, sizeof(temp));
      Data_of_BODY_VEL = 0;
    }
  }

  if(Data_of_ANGULAR_VEL==1){ // ANGULAR_VEL資料接收完畢，開始解封包 my own
    if(ANGULAR_VEL_Data[1]==TYPE_ANGULAR_VEL && ANGULAR_VEL_Data[2]==ANGULAR_VEL_LEN){ // 再次校驗資料類型和資料長度
      uint8_t temp[4];
      temp[0] = ANGULAR_VEL_Data[7], temp[1] = ANGULAR_VEL_Data[8], temp[2] = ANGULAR_VEL_Data[9], temp[3] = ANGULAR_VEL_Data[10];
      PacketAVEL.AVel_x = _HEX_to_Float_New(temp, true);
      Deg_AVEL.AVel_x = degrees(PacketAVEL.AVel_x);

      temp[0] = ANGULAR_VEL_Data[11], temp[1] = ANGULAR_VEL_Data[12], temp[2] = ANGULAR_VEL_Data[13], temp[3] = ANGULAR_VEL_Data[14];
      PacketAVEL.AVel_y = _HEX_to_Float_New(temp, true);
      Deg_AVEL.AVel_y = degrees(PacketAVEL.AVel_y);

      temp[0] = ANGULAR_VEL_Data[15], temp[1] = ANGULAR_VEL_Data[16], temp[2] = ANGULAR_VEL_Data[17], temp[3] = ANGULAR_VEL_Data[18];
      PacketAVEL.AVel_z = _HEX_to_Float_New(temp, true);
      Deg_AVEL.AVel_z = degrees(PacketAVEL.AVel_z);

      // 資料解包完畢，清空標誌等待下次資料解包
      Data_of_ANGULAR_VEL = 0;
    }
  }
}
// just for inital, no transform
void WHEELTEC_N100::Get_Init_POS(int sample_length){
  RPY_Deg sum_data;
  sum_data.Pitch = 0; sum_data.Roll = 0; sum_data.Yaw = 0;
  for(int i = 0; i < sample_length; i++){
    Read_Data();
    Unpack_Data();
    sum_data.Pitch += POS_RPY.Pitch;
    sum_data.Roll  += POS_RPY.Roll;
    sum_data.Yaw   += POS_RPY.Yaw;
  }
  Init_POS_RPY.Pitch = sum_data.Pitch / sample_length;
  Init_POS_RPY.Roll  = sum_data.Roll  / sample_length;
  Init_POS_RPY.Yaw   = sum_data.Yaw   / sample_length;
}

//my own
void WHEELTEC_N100::PrintPacketBytes(const uint8_t* packet, size_t length) {
  Serial.print("Packet bytes: ");
  for (size_t i = 0; i < length; i++) {
    Serial.print("0x");
    if (packet[i] < 0x10) {
      Serial.print("0"); // 確保單位數字節補零
    }
    Serial.print(packet[i], HEX); // 打印字節內容（16進制格式）
    Serial.print(" "); // 添加空格以方便閱讀
  }
  Serial.println(); // 換行
}
// // 16進位轉浮點數
// float WHEELTEC_N100::_HEX_to_Float(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4){
//   // 資料由高位到低位排序: data1 data2 data3 data4
//   // 其中接收資料時, 高位存放在後面, 低位存放在前面
  
//   short sign;         // 符號計算
//   unsigned long temp; // 32位16進制數
//   short power;        // 存放指数
//   float number;       // 存放尾数
//   float ref;          // 存放計算結果
//   uint16_t H_16,L_16; // 存放高16位、低16位

//   H_16 = data1<<8 | data2; // 數據第一次融合
//   L_16 = data3<<8 | data4;

//   // 將融合成16位元的數據組合成32位元數據
//   temp = (unsigned long)H_16<<16 | (unsigned long)L_16;

//   // 浮點數轉換開始
//   // 先確定符號
//   sign = (temp & 0x80000000) ? -1 : 1;  // 最高位是1就是負數, 0就是正數
//   // 計算指數
//   // 127是偏移量, 使得指數有正負(指數的範圍是 -127 ~ +128)
//   power = ((temp >> 23) & 0xff) - 127;
  
//   // 取得尾數部分 將（temp & 0x7fffff）所得的值將小數點左移23位 (除以2的23次方)
//   number = 1 + ((float)(temp & 0x7fffff) / 0x800000);
  
//   // 最终的公式
//   ref = sign * number * pow(2, power);
//   return ref;
// }

// 16進位轉浮點數 (32bit data -> 32bit float)
float WHEELTEC_N100::_HEX_to_Float_New(uint8_t *data, bool mode) //mode decide if keep the order data, else flip
{
  float fa = 0;
  uint8_t uc[4];

  if(mode == false){
    uc[3] = data[0];
    uc[2] = data[1];
    uc[1] = data[2];
    uc[0] = data[3];
  }
  else{
    uc[0] = data[0];
    uc[1] = data[1];
    uc[2] = data[2];
    uc[3] = data[3];
  }
  memcpy(&fa, uc, 4);
  return fa;
}

//timestamp解封包函數
long long WHEELTEC_N100::_timestamp(uint8_t Data_1,uint8_t Data_2,uint8_t Data_3,uint8_t Data_4){
  unsigned long temp;  //32位16進制數
  uint16_t H_16, L_16; //存放高16位、低16位

  H_16 = Data_1 << 8 | Data_2;
  L_16 = Data_3 << 8 | Data_4;

  //將融合成16位元的數據組合成32位元數據
  temp = (unsigned long)H_16 << 16 | (unsigned long)L_16;

  return temp;
}

void WHEELTEC_N100::Print_IMU_Packet(){
  Serial.print("gyro_x = ");
  Serial.println(PacketIMU.gyroscope_x,7);
  Serial.print("gyro_y = ");
  Serial.println(PacketIMU.gyroscope_y,7);
  Serial.print("gyro_z = ");
  Serial.println(PacketIMU.gyroscope_z,7);
  Serial.print("accel_X = ");
  Serial.println(PacketIMU.accelerometer_x,7);
  Serial.print("accel_y = ");
  Serial.println(PacketIMU.accelerometer_y,7);
  Serial.print("accel_z = ");
  Serial.println(PacketIMU.accelerometer_z,7);
  Serial.print("mag_X = ");
  Serial.println(PacketIMU.magnetometer_x,7);
  Serial.print("mag_y = ");
  Serial.println(PacketIMU.magnetometer_y,7);
  Serial.print("mag_z = ");
  Serial.println(PacketIMU.magnetometer_z,7);
  Serial.print("temperature = ");
  Serial.println(PacketIMU.imu_temperature);
  Serial.print("Pressure = ");
  Serial.println(PacketIMU.Pressure,7);
  Serial.print("pressure_temperature = ");
  Serial.println(PacketIMU.pressure_temperature,7);
  Serial.print("Timestamp = ");
  Serial.println(PacketIMU.Timestamp);
  Serial.println();
}

void WHEELTEC_N100::Print_AHRS_Packet(){
  Serial.print("RollSpeed = ");
  Serial.println(PacketAHRS.RollSpeed,7);
  Serial.print("PitchSpeed = ");
  Serial.println(PacketAHRS.PitchSpeed,7);
  Serial.print("HeadingSpeed = ");
  Serial.println(PacketAHRS.HeadingSpeed,7);
  Serial.print("Roll = ");
  Serial.println(PacketAHRS.Roll,7);
  Serial.print("Pitch = ");
  Serial.println(PacketAHRS.Pitch,7);
  Serial.print("Heading = ");
  Serial.println(PacketAHRS.Heading,7);
  Serial.print("Qw = ");
  Serial.println(PacketAHRS.Qw,7);
  Serial.print("Qx = ");
  Serial.println(PacketAHRS.Qx,7);
  Serial.print("Qy = ");
  Serial.println(PacketAHRS.Qy,7);
  Serial.print("Qz = ");
  Serial.println(PacketAHRS.Qz,7);
  Serial.print("Timestamp = ");
  Serial.println(PacketAHRS.Timestamp);
  Serial.println();
}

void WHEELTEC_N100::Print_RPY_Packet(){
  Serial.print("Roll  = ");Serial.println(PacketRPY.Roll, 7);
  Serial.print("Pitch = ");Serial.println(PacketRPY.Pitch,7);
  Serial.print("Yaw   = ");Serial.println(PacketRPY.Yaw,7);
  Serial.println("--------------------");
}

void WHEELTEC_N100::Print_POS_RPY(){
  Serial.print("Roll  = ");Serial.println(POS_RPY.Roll, 5);
  Serial.print("Pitch = ");Serial.println(POS_RPY.Pitch,5);
  Serial.print("Yaw   = ");Serial.println(POS_RPY.Yaw,5);
  Serial.println("--------------------");
}

void WHEELTEC_N100::Print_Init_POS_RPY(){
  char print_buffer[100];
  sprintf(print_buffer, "Init_POS_RPY: Roll = %.5f, Pitch = %.5f, Yaw = %.5f", Init_POS_RPY.Roll, Init_POS_RPY.Pitch, Init_POS_RPY.Yaw);
  Serial.println(print_buffer);
  Serial.println("--------------------");
}
//my own
void WHEELTEC_N100::Print_VEL_Packet(){
  Serial.print("Vel_X = ");Serial.println(PacketVEL.Vel_x, 7);
  Serial.print("Vel_Y = ");Serial.println(PacketVEL.Vel_y,7);
  Serial.print("Vel_Z = ");Serial.println(PacketVEL.Vel_z,7);
  Serial.println("--------------------");
}
//my own
void WHEELTEC_N100::Print_AVEL_Packet(){
  Serial.print("AVel_X = ");Serial.println(PacketAVEL.AVel_x, 7);
  Serial.print("AVel_Y = ");Serial.println(PacketAVEL.AVel_y,7);
  Serial.print("AVel_Z = ");Serial.println(PacketAVEL.AVel_z,7);
  Serial.println("--------------------");
    
//   //myown
//   Serial.print("ANGULAR_VEL \t");
//   PrintPacketBytes(ANGULAR_VEL_Data, sizeof(ANGULAR_VEL_Data));
//   //print crc8
//   uint8_t crc8 = CRC8_Table(ANGULAR_VEL_Data, 4);
//   Serial.print("A_VEL crc8: \t");
//   Serial.println(crc8, HEX);
//   // //print crc16
//   uint8_t d16[12];
//   memcpy(d16, &ANGULAR_VEL_Data[7], 12); // 複製 12 個字節，從第 7 個開始
//   uint16_t crc16 = CRC16_Table(d16, 12);
//   Serial.print("A_VEL crc16: \t");
//   Serial.println(crc16, HEX);
}
//my own
void WHEELTEC_N100::Print_AVEL_Deg(){
  Serial.print("AVel_X = ");Serial.println(Deg_AVEL.AVel_x, 7);
  Serial.print("AVel_Y = ");Serial.println(Deg_AVEL.AVel_y,7);
  Serial.print("AVel_Z = ");Serial.println(Deg_AVEL.AVel_z,7);
  Serial.println("--------------------");

    //myown
  Serial.print("ANGULAR_VEL \t");
  PrintPacketBytes(ANGULAR_VEL_Data, sizeof(ANGULAR_VEL_Data));
  //print crc8
  uint8_t crc8 = CRC8_Table(ANGULAR_VEL_Data, 4);
  Serial.print("A_VEL crc8: \t");
  Serial.println(crc8, HEX);
  // //print crc16
  uint8_t d16[12];
  memcpy(d16, &ANGULAR_VEL_Data[7], 12); // 複製 12 個字節，從第 7 個開始
  uint16_t crc16 = CRC16_Table(d16, 12);
  Serial.print("A_VEL crc16: \t");
  Serial.println(crc16, HEX);
}

//my own
static const uint8_t CRC8Table[] = {
0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};
uint8_t WHEELTEC_N100::CRC8_Table(uint8_t* p, uint8_t counter){
  uint8_t crc8 = 0;
  for (int i = 0; i < counter; i++)
  {
  uint8_t value = p[i];
  uint8_t new_index = crc8 ^ value;
  crc8 = CRC8Table[new_index];
  }
  return (crc8);
}

static const uint16_t CRC16Table[256] =
{
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
uint16_t WHEELTEC_N100::CRC16_Table(uint8_t* p, uint8_t counter)
{
uint16_t crc16 = 0;
for (int i = 0; i < counter; i++)
{
uint8_t value = p[i];
crc16 = CRC16Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8);
}
return (crc16);
}