#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#define DECODE_NEC  // 定义解码对象为nec

// #include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 13
//串口配置
#define Rx_Pin 16
#define Tx_Pin 17
//ESPNOW信道配置
#define ESPNOW_WIFI_CHANNEL 6
//LED矩阵配置 引脚14 23 27
#define LED_PIN_L 14
#define LED_PIN_R 23
#define LED_PIN_B 27
#define NUM_LED_LR 48
#define NUM_LED_B 30
#define INTERVAL 200  // LED 切换时间间隔 (ms)
//蜂鸣器配置
#define BUZZER_PIN 26

bool IR_FLAG = 0;

// 当前发送对象设备的 MAC 地址
uint8_t peerAddress[]{};  // 初始设备 MAC 地址

// 当前发送对象设备的 MAC 地址
// uint8_t peerAddress1[] = { 28, 157, 194, 69, 187, 60 };  // 测试设备 MAC 地址

// 新的设备 MAC 地址（将动态更换）
// uint8_t nextPeerAddress[6] = {};  // 下一设备的 MAC 地址

// 用于存储发送和接收数据
typedef struct struct_message {
  char txmessage[32];  // 消息内容（最大 32 字节）
  char rxmessage[32];  // 消息内容（最大 32 字节）
} struct_message;

// 初始化 WS2812 灯带对象
Adafruit_NeoPixel strip_l(NUM_LED_LR, LED_PIN_L, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_r(NUM_LED_LR, LED_PIN_R, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_b(NUM_LED_B, LED_PIN_B, NEO_GRB + NEO_KHZ800);
//初始化红外接收对象
uint64_t myRawdata;
//初始化ESPNWO接收结构体
struct_message myData;
String cmd;


//收到对方ESP32发来的命中信息后对LED矩阵进行控制
//LED播放动画线程
unsigned long previousMillis = 0;
int step = 0;            // 状态机步骤
bool ledActive = false;  // 仅在收到 "Hit!" 时激活 LED
void ledTask(void *pvParameters) {
  while (1) {
    if (strcmp(myData.rxmessage, "Hit!") == 0) {
      ledActive = true;
      memset(myData.rxmessage, 0, sizeof(myData.rxmessage));  // 清空消息
      step = 0;                                               // 重新开始 LED 序列
    }

    if (ledActive) {
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= INTERVAL) {
        previousMillis = currentMillis;  // 更新时间

        switch (step) {
          case 0:
            Led_1(strip_l.Color(255, 255, 255));  // 第一圈亮白色
            Led_1(strip_r.Color(255, 255, 255));  // 第一圈亮白色
            break;
          case 1:
            Led_2(strip_l.Color(255, 255, 255));  // 第二圈亮白色
            Led_2(strip_r.Color(255, 255, 255));  // 第二圈亮白色
            break;
          case 2:
            Led_3(strip_l.Color(255, 255, 255));  // 第三圈亮白色
            Led_3(strip_r.Color(255, 255, 255));  // 第三圈亮白色
            break;
          case 3:
            Led_1(strip_l.Color(0, 0, 0));  // 第一圈熄灭
            Led_1(strip_r.Color(0, 0, 0));  // 第一圈熄灭
            break;
          case 4:
            Led_2(strip_l.Color(0, 0, 0));  // 第二圈熄灭
            Led_2(strip_r.Color(0, 0, 0));  // 第二圈熄灭
            break;
          case 5:
            Led_3(strip_l.Color(0, 0, 0));  // 第三圈熄灭
            Led_3(strip_r.Color(0, 0, 0));  // 第三圈熄灭
            ledActive = false;              // 结束 LED 序列
            break;
        }

        step++;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // 避免任务占用过多 CPU 资源
  }
}

// 接收到消息时的回调函数
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  // 复制收到的数据
  memcpy(&myData.rxmessage, incomingData, sizeof(myData.txmessage));

  // 打印收到的消息内容
  Serial1.print("Message: ");
  Serial1.println(myData.rxmessage);
}

// 发送结果回调函数，发送完成后移除当前注册设备
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // 发送完成后移除当前设备，并添加下一个设备
  esp_now_del_peer(peerAddress);  // 移除当前设备
  memset(peerAddress, 0, 6);      //清空缓存数组
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
      // Serial1.println(myRawdata, HEX);
      // Serial1.println();
    }
  }
}


//LED矩阵控制
//预设WS2812下标（6x8）
uint8_t Led_Frame1[8]{ 14, 15, 20, 21, 26, 27, 32, 33 };
uint8_t Led_Frame2[16]{ 7, 8, 9, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 38, 39, 40 };
uint8_t Led_Frame3[24]{ 0, 1, 2, 3, 4, 5, 6, 11, 12, 17, 18, 23, 24, 29, 30, 35, 36, 41, 42, 43, 44, 45, 46, 47 };
//第一圈控制，依照RGB颜色值
void Led_1(uint32_t color) {
  for (int i = 0; i < 8; i++) {
    strip_l.setPixelColor(Led_Frame1[i], color);
    strip_r.setPixelColor(Led_Frame1[i], color);
  }
  strip_l.show();
  strip_r.show();
}


//第二圈控制
void Led_2(uint32_t color) {
  for (int i = 0; i < 16; i++) {
    strip_l.setPixelColor(Led_Frame2[i], color);
    strip_r.setPixelColor(Led_Frame2[i], color);
  }
  strip_l.show();
  strip_r.show();
}


//第三圈控制
void Led_3(uint32_t color) {
  for (int i = 0; i < 24; i++) {
    strip_l.setPixelColor(Led_Frame3[i], color);
    strip_r.setPixelColor(Led_Frame3[i], color);
  }
  strip_l.show();
  strip_r.show();
}

//背部LED控制
void Led_Back(uint32_t color) {
  for (int i = 0; i < 30; i++) {
    strip_b.setPixelColor(i, color);
  }
  strip_b.show();
}
//红外接收任务
void IRReceiveTask(void *pvParameters) {
  unsigned long startMillis = millis();

  while (millis() - startMillis < 500) {  // 0.5s 内检测红外信号
    IR();
    vTaskDelay(pdMS_TO_TICKS(10));  // 避免占用太多 CPU
  }

  EspSend(myData);    // 发送数据
  vTaskDelete(NULL);  // 任务结束并删除自身
}


//串口命令接收
void docmd(String cmdl) {
  cmdl.trim();  // 去除首尾空格
  if (cmdl == "MAC") {
    getMac();  // 发送 MAC 地址
  } else if (cmdl == "SHOOT") {
    xTaskCreate(IRReceiveTask, "IRTask", 2048, NULL, 2, NULL);  // 启动独立任务
  } else if (cmdl.length() >= 5 && cmdl.substring(0, 5) == "COLOR") {
    uint32_t color = 0;
    if (sscanf(cmdl.c_str() + 6, "%x", &color) == 1) {  // 跳过 "COLOR "
      Led_Back(color);
    }else if(){

    } else {
      Serial1.println("Invalid COLOR format! Use: COLOR RRGGBB");
    }
  } else if (cmdl.length() > 0) {
    Serial1.println("Unknown Command");  // 未知命令反馈
  }
}

//获取MAC地址转长整数
// void getMac() {
//   uint8_t mac[6];
//   uint64_t macint = 0;
//   WiFi.macAddress(mac);
//   for (int i = 0; i < 6; i++) {
//     macint |= (uint64_t(mac[i]) << (8 * (5 - i)));  // 位移并合并
//   }
//   Serial1.println(macint, HEX);
// }


//获取MAC转字符串
void getMac() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial1.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

//EspNow发送
void EspSend(struct_message _myData) {
  //打印结果
  Serial1.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", peerAddress[0], peerAddress[1], peerAddress[2], peerAddress[3], peerAddress[4], peerAddress[5]);
  // 发送消息
  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&_myData.txmessage, sizeof(_myData.txmessage));  //从节点树中找到对应MAC的节点进行信息发送
  if (result == ESP_OK) {
    Serial1.println("sent successfully");
  } else {
    Serial1.println("sent failed");
  }
}


//初始化配置
void setup() {
  // 初始化 WS2812 灯带
  strip_b.begin();
  strip_b.show();
  strip_r.begin();
  strip_r.show();
  strip_l.begin();
  strip_l.show();  // 初始化时确保灯带处于关闭状态

  // 初始化蜂鸣器引脚
  pinMode(BUZZER_PIN, OUTPUT);
  //初始化串口
  Serial1.begin(115200, SERIAL_8N1, Rx_Pin, Tx_Pin);  //8位数据1停止位，无校验，RX16 TX17.
  Serial.begin(115200);
  //WIFI设置为STA
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  // 初始化 ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial1.println("ESP-NOW initialization failed");
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

  // 创建ESPNOW信息监听及LED 控制任务
  xTaskCreatePinnedToCore(
    ledTask,     // 任务函数
    "LED Task",  // 任务名称
    1000,        // 栈大小
    NULL,        // 任务参数
    3,           // 任务优先级
    NULL,        // 任务句柄
    0            // 绑定 CPU 核心 0
  );
}


//主任务
void loop() {
  if (Serial1.available()) {
    cmd = Serial1.readStringUntil('\n');  // 读取串口命令
    docmd(cmd);                           // 解析并执行命令
  }
}