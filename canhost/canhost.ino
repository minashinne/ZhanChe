//导入exoCAN
#include <eXoCAN.h>
#define Rx0 PA10
#define TX0 PA9
#define Rx1 PA3
#define Tx1 PA2
HardwareSerial Serial2(Rx1, Tx1);  // RX1=PA3, TX1=PA2

//CAN设置
int rxMsgID = 111;
eXoCAN can(STD_ID_LEN, BR250K, PORTA_11_12_XCVR);

//CAN数据结构体
struct CanMessage {
  uint8_t data[8];  // 存储 8 字节数据
  bool updated;     // 标志位，表示数据是否更新
};

//设置CAN接收回调
int id, fltIdx = 0;
bool Rec_Flag = false;
// volatile uint8_t* rxData_2[9];  // 声明一个 9位指针数组来暂存数据
uint8_t rxData[8];        // 声明一个 8 字节一维数组来接收数据
CanMessage rxData_2[10];  // 存储数据
void canISR()             // 依照setup中的过滤器配置来接收CAN消息
{
  can.receive(id, fltIdx, rxData);  // 从CAN总线接收数据（接收到的ID，成功匹配消息的过滤器的索引，接收到的数据）
  // Serial.println(id);
  int index = id - 111;
  if (index >= 0 && index <= 9) {
    memcpy(rxData_2[index].data, rxData, 8);  //复制8位接收数据到结构体的data中。
    rxData_2[index].updated = true;           // 标记数据已更新
  }
}

void Uart_Send() {
  for (int i = 0; i < 4; i++) {
    if (rxData_2[i].updated) {
      char buffer[20];
      sprintf(buffer, "ID:%d %c %d %d %d", i + 111, rxData_2[i].data[0], (int16_t)(rxData_2[i].data[2] << 8 | rxData_2[i].data[1]), (int16_t)(rxData_2[i].data[4] << 8 | rxData_2[i].data[3]), (int16_t)(rxData_2[i].data[6] << 8 | rxData_2[i].data[5]));
      Serial.println(buffer);
      Serial.println();
      rxData_2[i].updated = false;  // 处理完成后清除标志
    }
  }
  for (int i = 4; i < 10; i++) {
    if (i == 6 || i == 7) {
      if (rxData_2[i].updated) {
        char buffer[50];
        sprintf(buffer, "ID:%d %c %PRIu64", i + 111, rxData_2[i].data[0], (uint64_t)(rxData_2[i].data[6] << 40) | (uint64_t)(rxData_2[i].data[5] << 32) | (uint64_t)(rxData_2[i].data[4] << 24) | (uint64_t)(rxData_2[i].data[3] << 16) | (uint64_t)(rxData_2[i].data[2] << 8) | (uint64_t)rxData_2[i].data[1]);
        Serial.println(buffer);
        Serial.println();
        rxData_2[i].updated = false;  // 处理完成后清除标志
      }
    } else {
      if (rxData_2[i].updated) {
        char buffer[20];
        sprintf(buffer, "ID:%d %c %d %d %d", i + 111, rxData_2[i].data[0], (int16_t)(rxData_2[i].data[2] << 8 | rxData_2[i].data[1]), (int16_t)(rxData_2[i].data[4] << 8 | rxData_2[i].data[3]), rxData_2[i].data[5]);
        Serial.println(buffer);
        Serial.println();
        rxData_2[i].updated = false;  // 处理完成后清除标志
      }
    }
  }
}

//CAN消息预设
//四驱控制
void Run(int txMsgID, int16_t Target_Speed) {
  uint8_t txData[3]{};
  int txDataLen = 3;
  txData[0] = 'S';                         //正常代码S(speed)
  txData[1] = Target_Speed & 0xFF;         // 获取最低字节
  txData[2] = (Target_Speed >> 8) & 0xFF;  // 获取最高字节
  can.transmit(txMsgID, txData, txDataLen);
}
//R指令
void Cmd_R(int txMsgID) {
  uint8_t txData[1]{};
  int txDataLen = 1;
  txData[0] = 'R';  //复位代码R(recover)
  can.transmit(txMsgID, txData, txDataLen);
}
//I指令
void Cmd_I(int txMsgID) {
  uint8_t txData[1]{};
  int txDataLen = 1;
  txData[0] = 'I';  //红外发射代码I(IR)
  can.transmit(txMsgID, txData, txDataLen);
}
//T指令
void Cmd_T(int txMsgID) {
  uint8_t txData[1]{};
  int txDataLen = 1;
  txData[0] = 'T';  //停止红外发射代码T(STOP)为与S区分
  can.transmit(txMsgID, txData, txDataLen);
}
//角度控制命令
void Angle(int txMsgID, int16_t Target_Angle_10) {
  uint8_t txData[3]{};
  int txDataLen = 3;
  txData[0] = 'A';                            //正常代码A(angle)
  txData[1] = Target_Angle_10 & 0xFF;         // 获取最低字节
  txData[2] = (Target_Angle_10 >> 8) & 0xFF;  // 获取最高字节
  can.transmit(txMsgID, txData, txDataLen);
}
//数据回报命令（通用，不止角度）
void Read_Angle(int txMsgID) {
  uint8_t txData[1]{};
  int txDataLen = 1;
  txData[0] = 'R';  //读取代码R(read)
  can.transmit(txMsgID, txData, txDataLen);
}
//零位归零命令
void Set_ZPos(int txMsgID) {
  uint8_t txData[1]{};
  int txDataLen = 1;
  txData[0] = 'Z';  //归零代码Z(Zero)
  can.transmit(txMsgID, txData, txDataLen);
}
//MAC地址发送命令
void Send_Mac(int txMsgID, uint64_t MAC) {
  uint8_t txData[7]{};
  int txDataLen = 7;
  if (MAC != 0) {
    txData[0] = 'M';                 //正常代码M
    txData[1] = MAC & 0xFF;          // 获取最低字节
    txData[2] = (MAC >> 8) & 0xFF;   // 获取9-16字节
    txData[3] = (MAC >> 16) & 0xFF;  // 获取17-24字节
    txData[4] = (MAC >> 24) & 0xFF;  // 获取25-32字节
    txData[5] = (MAC >> 32) & 0xFF;  // 获取33-40字节
    txData[6] = (MAC >> 40) & 0xFF;  // 获取41-48字节
    can.transmit(txMsgID, txData, txDataLen);
  }
}
//串口转CAN指令
void Uart_To_Can() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // 读取一整行命令
    command.trim();                                 // 去除首尾空格
    Serial.println("Received: " + command);         // 打印接收到的指令

    int id;
    uint64_t value;
    char cmd;

    // 解析串口指令格式: "ID 111 S 1000"
    if (sscanf(command.c_str(), "ID %d %c %PRIu64", &id, &cmd, &value) > 2) {
      Serial.print("Parsed ID: ");
      Serial.println(id);
      Serial.print("Parsed CMD: ");
      Serial.println(cmd);
      if (cmd == 'S') {
        Serial.print("Parsed Speed: ");
        Serial.println(value);
        Run(id, (int16_t)value);  // 发送速度命令
      } else if (cmd == 'A') {
        Serial.print("Parsed Angle: ");
        Serial.println(value);
        Angle(id, (int16_t)value);  // 发送角度命令
      } else if (cmd == 'M') {
        Send_Mac(id, value);  //发送MAC地址
      } else {
        Serial.println("⚠ 未知指令");
      }
    }
    // 解析无参数命令 (如 "ID 119 R")
    else if (sscanf(command.c_str(), "ID %d %c", &id, &cmd) == 2) {
      Serial.print("Parsed ID: ");
      Serial.println(id);
      Serial.print("Parsed CMD: ");
      Serial.println(cmd);
      if (cmd == 'R') {
        Cmd_R(id);  // 复位命令或角度读取命令
      } else if (cmd == 'Z') {
        Set_ZPos(id);  // 归零命令
      } else if (cmd == 'I') {
        Cmd_I(id);  //红外发射命令
      } else if (cmd == 'T') {
        Cmd_T(id);  //红外停发命令
      } else {
        Serial.println("⚠ 未知指令");
      }
    } else {
      Serial.println("⚠ 指令格式错误");
    }
  }
}
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);


  //CAN初始化
  can.attachInterrupt(canISR);              //注册CAN接收回调
  can.filterMask16Init(0, rxMsgID, 0x000);  // 设置CAN过滤器0，准许IDrxMsgID,掩码0x000
}

void loop() {
  Uart_Send();
  Uart_To_Can();
}
