#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#define DECODE_NEC  // 定义解码对象为nec

// #include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>

#include "Wire.h"

#define I2C_DEV_ADDR 0x18

#define IR_RECEIVE_PIN 14

#define ESPNOW_WIFI_CHANNEL 6

#define LED_PIN 23
#define NUM_LEDS 3

#define BUZZER_PIN 26

#define TASK1_STACK_SIZE 1000

TaskHandle_t Task1;

bool IR_FLAG = 0;

// 当前发送对象设备的 MAC 地址
uint8_t peerAddress[] = {};  // 初始设备 MAC 地址

// 当前发送对象设备的 MAC 地址
uint8_t peerAddress1[] = { 28, 157, 194, 69, 187, 60 };  // 测试设备 MAC 地址

// 新的设备 MAC 地址（将动态更换）
uint8_t nextPeerAddress[6] = {};  // 下一设备的 MAC 地址

// 用于存储发送和接收数据
typedef struct struct_message {
  char txmessage[32];  // 消息内容（最大 32 字节）
  char rxmessage[32];  // 消息内容（最大 32 字节）
} struct_message;

// 初始化 WS2812 灯带对象
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

uint64_t myRawdata;
struct_message myData;
String cmd;

// void task1(void *pvParameters) {
//   struct_message *_myData = (struct_message *)pvParameters;
//   while (1) {
//     if (_myData->rxmessage == "Hit!") {
//       // 控制 WS2812 灯带变化
//       for (int i = 0; i < NUM_LEDS; i++) {
//         strip.setPixelColor(i, strip.Color(255, 0, 0));  // 设置为红色
//       }
//       strip.show();
//       // 控制蜂鸣器响声
//       tone(BUZZER_PIN, 1000);  // 设置频率为 1000Hz
//       delay(500);

//       // 关闭 WS2812 灯带
//       for (int i = 0; i < NUM_LEDS; i++) {
//         strip.setPixelColor(i, strip.Color(0, 0, 0));  // 关闭灯光
//       }
//       strip.show();
//       noTone(BUZZER_PIN);  // 停止蜂鸣器
//       delay(500);
//     }
//     Serial.println("Task 1 is running");
//     delay(500);  // 让出CPU时间
//   }
// }

void hit() {
  if (strcmp(myData.rxmessage, "Hit!") == 0) {
    // 清空收到的数据
    memset(&myData.rxmessage, 0, sizeof(myData.txmessage));
    // 控制 WS2812 灯带变化
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(255, 0, 0));  // 设置为红色
    }
    strip.show();
    // // 控制蜂鸣器响声
    // tone(BUZZER_PIN, 1000);  // 设置频率为 1000Hz
    // delay(500);

    // 关闭 WS2812 灯带
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(0, 0, 0));  // 关闭灯光
    }
    strip.show();
    // noTone(BUZZER_PIN);  // 停止蜂鸣器
    // delay(500);
  }
}
// 接收到消息时的回调函数
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  // 复制收到的数据
  memcpy(&myData.rxmessage, incomingData, sizeof(myData.txmessage));

  // 打印收到的消息内容
  Serial.print("Message: ");
  Serial.println(myData.rxmessage);
}

// 发送结果回调函数，发送完成后移除当前注册设备
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //发送结果打印
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

  // 发送完成后移除当前设备，并添加下一个设备
  esp_now_del_peer(peerAddress);  // 移除当前设备
  // Serial.println("Removed current peer.");

  // // 更换目标设备（将下一个设备的 MAC 地址添加为对端）
  // memcpy(peerAddress, nextPeerAddress, 6);  // 设置下一个设备的 MAC 地址

  // esp_now_peer_info_t peerInfo;
  // memcpy(peerInfo.peer_addr, peerAddress, 6);
  // peerInfo.channel = 6;      // 默认频道
  // peerInfo.encrypt = false;  // 不加密

  // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //   Serial.println("Failed to add new peer.");
  // } else {
  //   Serial.println("Added new peer.");
  // }
}

//IIC命令解析
void docmd(String cmdl) {
  //MAC
  if (cmdl != "" && cmdl.substring(0, 3) == "MAC") {
    Serial1.println(cmdl);
    cmd = "";
    //射击
  } else if (cmdl == "SHOOT") {
    //收到命令后开始接收红外信号0.5S
    unsigned long currentMillis = millis();
    IR();
    EspSend(myData);
    cmd = "";
    //其他
  } else if (cmdl != "") {
    Serial1.println("clear");
    cmd = "";
  }
}


//红外接收并配置接收到的ESP MAC以准备进行发送
void IR() {
  if (IR_FLAG == 0) {
    IR_FLAG = 1;
    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  }
  if (IrReceiver.decode()) {
    IrReceiver.resume();  // Early enable receiving of the next IR frame
    myRawdata = IrReceiver.decodedIRData.decodedRawData;
    if (myRawdata != 0 && (myRawdata > 0x000000000000 && myRawdata < 0xFFFFFFFFFFFF)) {
      // 将 48 位整数拆分为 6 个字节，并填充到 peerAddress 数组中
      uint8_t _peerAddress[] = {};
      for (int i = 0; i < 6; i++) {
        _peerAddress[5 - i] = (myRawdata >> (8 * i)) & 0xFF;  // 提取每个字节
      }
      //espnow配置结构
      memcpy(peerAddress, _peerAddress, 6);
      esp_now_peer_info_t peerInfo;
      memset(&peerInfo, 0, sizeof(peerInfo));      //清空结构体，不然会有意料外的垃圾数据
      memcpy(peerInfo.peer_addr, peerAddress, 6);  //注册MAC到节点树
      peerInfo.channel = 6;                        // 默认频道
      peerInfo.encrypt = false;                    // 不加密
      esp_now_add_peer(&peerInfo);                 //注册节点树
      Serial.println(myRawdata, HEX);
      Serial.println();
    }
  }
}
void setup() {
  // 初始化 WS2812 灯带
  strip.begin();
  strip.show();  // 初始化时确保灯带处于关闭状态

  // 初始化蜂鸣器引脚
  pinMode(BUZZER_PIN, OUTPUT);
  //初始化IIC
  Wire.begin(I2C_DEV_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.setClock(100000);
  Serial.begin(115200);
  //WIFI设置为STA
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  // 初始化 ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  // 注册ESPNOW发送回调函数
  esp_now_register_send_cb(onDataSent);

  // 注册ESPNOW接收回调函数
  esp_now_register_recv_cb(onDataReceived);

  // 添加初始对端设备
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 6;      // 默认频道
  peerInfo.encrypt = false;  // 不加密
  esp_now_add_peer(&peerInfo);

  // IR初始化
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  IrReceiver.end();
  // 填充发送的数据
  strcpy(myData.txmessage, "Hit!");

  //   xTaskCreate(task1,        // 任务函数
  //               "Task with Struct",  // 任务名称
  //               2000,                // 堆栈大小
  //               (void *)&myData,  // 传递的参数（结构体指针）
  //               1,                   // 优先级
  //               &Task1);             // 任务句柄
}

void loop() {
  hit();
  docmd(cmd);
  IR();
  // //espnow配置结构
  // esp_now_peer_info_t peerInfo;
  // memset(&peerInfo, 0, sizeof(peerInfo));  //清空结构体，不然会有意料外的垃圾数据
  // memcpy(peerInfo.peer_addr, peerAddress1, 6);
  // peerInfo.channel = 6;      // 默认频道
  // peerInfo.encrypt = false;  // 不加密
  // esp_now_add_peer(&peerInfo);
  // esp_now_send(peerAddress1, (uint8_t *)&myData.txmessage, sizeof(myData.txmessage));
}



//IIC中断处理
void receiveEvent(int howMany) {
  while (0 < Wire.available())  // loop through all but the last
  {
    cmd += (char)Wire.read();  // receive byte as a character
  }
}
//MAC地址
void getMac() {
  uint8_t mac[6];
  uint64_t macint = 0;
  WiFi.macAddress(mac);
  for (int i = 0; i < 6; i++) {
    macint |= (uint64_t(mac[i]) << (8 * (5 - i)));  // 位移并合并
  }
  Serial.println(macint, HEX);
}
//EspNow发送
void EspSend(struct_message _myData) {
  // // 打印结果
  // Serial.print("Parsed MAC Address: ");
  // for (int i = 0; i < 6; i++) {
  //   Serial.print(peerAddress[i]);
  //   if (i < 5) Serial.print(":");
  // }
  // Serial.println();
  // 发送消息
  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&_myData.txmessage, sizeof(_myData.txmessage));  //从节点树中找到对应MAC的节点进行信息发送
  if (result == ESP_OK) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Message failed to send");
  }
}