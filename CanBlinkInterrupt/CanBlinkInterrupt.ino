/* 
 bpCanBlinkInterrupt.ino                                                                     4/24/20
 C:\Users\jhe\Documents\PlatformIO\mySTM32LIBS\eXoCAN\bpCanBlink.cpp

 This simple example requires two STM32f103 boards.  One of the boards is loaded
 with the 'frame' using txMsg = 0x069.  The second board gets loaded with the second
 frame where tsMsgID = 0x005 by uncommenting that section.

 On each board connect the CAN default RX and TX pins(PA11, PA12) together.  Then connect
 the boards together: +5V, Gnd., and Can(PA11/12 pair).

 When a board receives a CAN message it toggles its LED.  One board sends a message
 every second and the other every five seconds.

  RAM:   1180 bytes 
  Flash: 13028 bytes 

 working                                                                            4/25

*/
#include <arduino.h>
#include <eXoCAN.h>
#define bluePillLED PC13

// tx frame setup #1
int txMsgID = 0x069;
int rxMsgID = 0x111;  // needed for rx filtering

uint8_t txData[4]{ 0x01, 0xfe, 0xdc, 0xba };
uint32_t txInt = 115200;
uint8_t txDataLen = 4;
uint32_t txDly = 100;  // mSec
//  ****** uncomment the following for the second stm32f103 board ******

// int txMsgID = 0x005;
// int rxMsgID = 0x069;   // only needed if using a filter
// uint8_t txData[8]{{0x01, 0xfe, 0xdc, 0xba, 0x11, 0x12, 0x34, 0x56};
// uint_8 txDataLen = 8;
// uint32_t txDly = 1000;  // mSec

int id, fltIdx;
volatile uint8_t rxData[4];  // 声明一个 4 字节数组来接收数据
uint32_t rxInt;              // 用于存储接收到的 32 位整数
// 11b IDs, 250k bit rate, no transceiver chip, portA pins 11,12, no external resistor
eXoCAN can(STD_ID_LEN, BR250K, PORTA_11_12_WIRE_PULLUP);

void canISR()  // get CAN bus frame passed by a filter into fifo0
{
  can.receive(id, fltIdx, rxData);  // 从 CAN 总线接收数据
  rxInt = (rxData[3] << 24) | (rxData[2] << 16) | (rxData[1] << 8) | rxData[0];
}

void setup() {
  Serial.begin(115200);
  can.attachInterrupt(canISR);
  can.filterMask16Init(0, rxMsgID, 0x7ff);  // filter bank 0, filter 0: allow ID = rxMsgID, mask = 0x7ff (must match)
                                            // without calling a filter, the default passes all IDs
  pinMode(bluePillLED, OUTPUT);
}

uint32_t last = 0;
void loop() {
  if (millis() / txDly != last)  // send every txDly, rx handled by the interrupt
  {
    txInt += 1;
    // 手动将整数拆分成字节
    txData[0] = txInt & 0xFF;          // 获取最低字节
    txData[1] = (txInt >> 8) & 0xFF;   // 获取次低字节
    txData[2] = (txInt >> 16) & 0xFF;  // 获取次高字节
    txData[3] = (txInt >> 24) & 0xFF;  // 获取最高字节
    Serial.println(rxInt);
    last = millis() / txDly;
    can.transmit(txMsgID, txData, txDataLen);
  }
}
