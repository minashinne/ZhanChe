#include <Arduino.h>
//导入红外接收库
#include <IRremote.hpp>

//导入exoCAN
#include <eXoCAN.h>
#define IR_SEND_PIN PB15
#define IR_SEND_PIN_STRING "PB15"

//CAN设置
int txMsgID = 117;  //发送报文附带的ID
int rxMsgID = 117;  //准许接收指定附带ID的报文（过滤器控制，未进入loop模式，不会接收自己的报文）
eXoCAN can(STD_ID_LEN, BR250K, PORTA_11_12_XCVR);


//设置CAN接收回调
int id, fltIdx = 0;
bool Run_Flag = false;
bool Send_Data_Flag = false;
volatile uint8_t rxData[8]{};  // 声明一个 8 字节数组来接收数据
uint8_t MAC[6]{};
uint64_t macInt = 0;
void canISR()  // 依照setup中的过滤器配置来接收CAN消息
{
  can.receive(id, fltIdx, rxData);  // 从CAN总线接收数据（接收到的ID，成功匹配消息的过滤器的索引，接收到的数据）
  // Serial.println(id);
  switch (rxData[0]) {
    case 'M':      //更新MAC地址
      macInt = 0;  // 在计算前清空macInt，防止数据累积错误
      for (int i = 0; i < 6; i++) {
        MAC[i] = rxData[i + 1];
        macInt |= ((uint64_t)MAC[i] << (8 * (5 - i)));  // 组合 48 位整数
      }
      break;
    case 'R':  //回报当前MAC
      Send_Data_Flag = true;
      break;
    case 'I':  //开始发送红外信号
      Run_Flag = true;
      break;
    case 'T':  //停止发送红外信号
      Run_Flag = false;
      break;
  }
}

unsigned long lastTime = 0;  // 上次计算时间
//上报当前缓存的MAC
uint8_t txData[8]{};  //整数转化后的数组容器,第0位为状态代码，第1,2位为目标角度，第3,4位为当前角度。
int txDataLen = 8;
void Send_Data() {
  if (MAC[0] != 0 && MAC[5] != 0) {
    txData[0] = 'M';     //正常代码M
    txData[1] = MAC[0];  //MAC第一位
    txData[2] = MAC[1];  //MAC第二位
    txData[3] = MAC[2];  //MAC第三位
    txData[4] = MAC[3];  //MAC第四位
    txData[5] = MAC[4];  //MAC第五位
    txData[6] = MAC[5];  //MAC第六位
    can.transmit(txMsgID, txData, txDataLen);
  }
}

String cmd;
void setup() {

  Serial.begin(115200);
  IrSender.begin(IR_SEND_PIN);  // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin
  disableLEDFeedback();         // Disable feedback LED at default feedback LED pin
  delay(100);
}


void loop() {
  unsigned long currentTime = millis();
  //得到红外发送指令后每100ms发送一次红外编码的MAC地址（NEC协议每bit发送占时2.25ms，48比特占时108ms 起始脉冲13.5ms，总花费121.5ms，为保证信息不混淆，间隔为发送所需时间+等待时间）
  if (Run_Flag) {
    if (currentTime - lastTime >= 200) {
      IrSender.sendNECRaw(macInt, 0);  //发送MAC，重复次数0
      lastTime = currentTime;       // 更新计算时间
    }
  }
  //得到发送指令后发送一次当前所得MAC地址
  if (Send_Data_Flag) {
    Send_Data();
    Send_Data_Flag = false;
  }
}
