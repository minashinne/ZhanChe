#include <Arduino.h>
//导入PID库
#include <QuickPID.h>
//导入exoCAN
#include <eXoCAN.h>
// 引脚定义
#define MOTOR_POWER_PIN PA4       // 电机电源控制
#define MOTOR_PWM_PIN PA1         // 电机PWM控制速度
#define MOTOR_DIR_PIN PB12        // 电机方向控制
#define MOTOR_HALL_PIN PA2        // 电机霍尔传感器
#define MOTOR_LOCK_PIN PC0        //电机解锁引脚（开机先1后0进行解锁（开机默认锁定态），后续解锁也是先1后0）
#define MOTOR_LOCK_STATE_PIN PC1  //电机锁定状态引脚（0锁定，1解锁）
// PWM相关设置
#undef PWM_FREQUENCY
#define PWM_FREQUENCY 20000  // 20kHz PWM频率
#define PWM_RESOLUTION 8     // 8位PWM分辨率 (0-255)
// HardwareSerial Serial1(PA10, PA9);编译选择bule button F103RCT，optimize选择faster（-O2，debug时选择debug以便报错输出，USB support必须关闭为CAN让出IO口，此板子的模块设置了使用外部晶振并9倍频为72Mhz（generic 板子的晶振设置为内部晶振8/2*16=64Mhz，未修改编译文件会导致许多频率敏感的库不可使用）。同时设置了默认的串口为PA10,PA9故不用再定义串口。-2025.02.21
// 电机结构体
struct Motor {
  uint8_t powerPin;
  uint8_t pwmPin;
  uint8_t dirPin;
  uint8_t hallPin;
  uint8_t speed;   // 电机速度
  bool direction;  // 电机方向
};

//电机设置
Motor motor = { MOTOR_POWER_PIN, MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_HALL_PIN, 255, true };
volatile uint16_t pulseCount = 0;  // 霍尔脉冲计数
// String cmd;
int16_t Target_Speed = 0;    //目标速度（电机输出轴朝你，-为顺时针，+为逆时针）,空载极限约6000RPM
int16_t Current_Speed = 0;   //当前速度
int16_t Current_PWM = 0;     //当前PWM值
unsigned long lastTime = 0;  // 上次计算时间
bool Recover_Flag = false;   //来自上位机的电机解锁命令标志
bool Error_Flag = false;     //电机错误标志
//CAN设置
int txMsgID = 111;  //发送报文附带的ID
int rxMsgID = 111;  //准许接收指定附带ID的报文（过滤器控制，未进入loop模式，不会接收自己的报文）
int Time_Conut = 0;

uint8_t txData[8]{};  //整数转化后的数组容器,第0位为状态代码，第1,2位为目标速度，第3,4位为当前速度，第5,6位为输出PWM。

uint8_t txDataLen = 8;  //发送数组长度（最长8位uint8数组）

int id, fltIdx = 0;
volatile uint8_t rxData[8]{};  // 声明一个 8 字节数组并初始化为0来接收数据
int16_t rxInt;                 // 用于存储接收到的 32 位整数

eXoCAN can(STD_ID_LEN, BR250K, PORTA_11_12_XCVR);
//设置CAN接收回调
void canISR()  // 依照setup中的过滤器配置来接收CAN消息
{
  can.receive(id, fltIdx, rxData);  // 从CAN总线接收数据（接收到的ID，成功匹配消息的过滤器的索引，接收到的数据）
  switch (rxData[0]) {
    case 'S':
      Target_Speed = (int16_t)((rxData[2] << 8) | rxData[1]);  //将接受数据转为16进制整数目标速度值
      break;
    case 'R':
      Recover_Flag = true;  //上位机电机解锁复位命令
      break;
  }
}

// 中断服务函数：每当霍尔传感器检测到一个脉冲时调用
void countPulse() {
  pulseCount++;  // 每个脉冲计数加1
}

//PID设置
float Setpoint, Input, Output;           //PID目标值、输入、输出
float Kp = 0.002, Ki = 0.1, Kd = 0.003;  //比例参数0.0015、积分参数0.1、微分参数0.003

//定义PID控制器，并传入三个参数的指针
QuickPID myPID(&Input, &Output, &Setpoint);

//PID
void PID() {
  Input = Current_Speed;  //更新PID输入（输入为转速信息不带符号）
  if (Target_Speed >= -6000 && Target_Speed <= 6000 && Target_Speed != 0) {
    Setpoint = abs(Target_Speed);  //更新PID目标值（输入为转速信息不带符号）
  } else if (Target_Speed == 0 && Output != 0) {
    myPID.Reset();
    Output = 0;
  }
  if (Target_Speed != 0) {
    myPID.Compute();  //计算输出（输出0-255PWM）
  }

  if (Target_Speed < 0) {
    digitalWrite(motor.dirPin, HIGH);
  } else {
    digitalWrite(motor.dirPin, LOW);
  }
  analogWrite(motor.pwmPin, (int)(255 - Output));
}

//发送正常代码、当前目标速度、实际速度、输出PWM
void SendData_1000ms() {
  txData[0] = Error_Flag ? 'E' : 'S';       //依据Error_Flag状态进行三元运算。正常代码A(angle)，错误代码E
  txData[1] = Target_Speed & 0xFF;          // 获取最低字节
  txData[2] = (Target_Speed >> 8) & 0xFF;   // 获取最高字节
  txData[3] = Current_Speed & 0xFF;         // 获取最低字节
  txData[4] = (Current_Speed >> 8) & 0xFF;  // 获取最高字节
  txData[5] = (int)Output & 0xFF;           // 获取最低字节
  txData[6] = ((int)Output >> 8) & 0xFF;    // 获取最高字节
  can.transmit(txMsgID, txData, txDataLen);
}


//堵转检测
int Dog_Count = 0;
int Dog_Try = 0;
unsigned long Dog_Time[3]{};
void Watch_Dog() {
  if (digitalRead(MOTOR_LOCK_STATE_PIN) == 0 && Error_Flag != true) {
    Dog_Count += 1;
    //电机锁死1S后尝试解锁脱困并将尝试次数加1
    if (Dog_Count >= 10) {
      digitalWrite(MOTOR_LOCK_PIN, HIGH);
      delay(10);
      digitalWrite(MOTOR_LOCK_PIN, LOW);
      Dog_Count = 0;
      Dog_Try += 1;
      Dog_Time[Dog_Try - 1] = millis();
      //如果锁死间隔大于30S，重置计数（即必须在30S内连续堵转3次才触发锁定）
      if (Dog_Try >= 2) {
        Serial.println("DG>=2");
        if ((Dog_Time[Dog_Try - 1] - Dog_Time[Dog_Try - 2]) >= 30000) {
          Serial.println("DG DROP");
          Dog_Try = 1;
          Dog_Time[0] = millis();
          Dog_Time[1] = 0;
          Dog_Time[2] = 0;
        }
      }
    }
    //尝试三次后错误标志置1，这将使loop中对错误标志的检测触发并开始持续1S间隔发送错误信号
    if (Dog_Try >= 3) {
      Dog_Try = 0;
      Error_Flag = true;
    }
  }
}

//死锁复位
void Recover() {
  if (Recover_Flag) {
    Recover_Flag = false;
    Error_Flag = false;
    digitalWrite(MOTOR_LOCK_PIN, HIGH);
    delay(1);
    digitalWrite(MOTOR_LOCK_PIN, LOW);
  }
}
//SETUP
void setup() {
  // 配置串口通信
  Serial.begin(115200);

  //PID初始化
  myPID.SetTunings(Kp, Ki, Kd);                      //传入KP.KI.KD
  myPID.SetOutputLimits(0, 255);                     //设置PID控制器输出范围为0-255
  myPID.SetSampleTimeUs(90000);                      //最少90MS可被调用一次
  myPID.SetMode(myPID.Control::automatic);           //设置PID控制器计算模式为自动计时控制（未到时间调用返回False，调用成功返回True）
  myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);  //设置防积分风暴模式为上下值控制（防止系统在受到外部扰动时导致积分项无限增大而导致系统失衡）
  //CAN初始化
  can.attachInterrupt(canISR);              //注册CAN接收回调
  can.filterMask16Init(0, rxMsgID, 0x7ff);  // 设置CAN过滤器0，准许ID rxMsgID,掩码0x7ff
  //  配置引脚模式
  pinMode(motor.powerPin, OUTPUT);
  pinMode(motor.pwmPin, OUTPUT);
  pinMode(motor.dirPin, OUTPUT);
  pinMode(MOTOR_LOCK_PIN, OUTPUT);
  pinMode(MOTOR_LOCK_STATE_PIN, INPUT);
  pinMode(motor.hallPin, INPUT_PULLUP);

  // 初始化电机状态
  digitalWrite(motor.powerPin, HIGH);  // 电源
  digitalWrite(motor.dirPin, LOW);     // 默认方向
  digitalWrite(MOTOR_LOCK_PIN, HIGH);
  delay(1);
  digitalWrite(MOTOR_LOCK_PIN, LOW);
  // 设置PWM频率
  analogWriteFrequency(PWM_FREQUENCY);
  analogWrite(motor.pwmPin, 255);  // 默认最小速度(MAX0,MIN255)

  // 配置霍尔传感器中断
  attachInterrupt(digitalPinToInterrupt(motor.hallPin), countPulse, FALLING);
}
//LOOP
void loop() {
  // unsigned long startTime = millis();  //运行时间测试
  //堵转检测，1S内一直保持0速度，关机1S（速度降为0）后续每个1S都尝试重启脱困。发送信息：数组第一位为标识码，默认为‘S’，错误为‘E’。
  unsigned long currentTime = millis();
  //PID
  if (currentTime - lastTime >= 100) {  // 每0.1秒钟计算一次
    Current_Speed = pulseCount * 100;   // 每转6个脉冲/6，计算转速*60*10=*100
    //PID
    Recover();
    PID();
    //Watch_Dog
    Watch_Dog();
    //打印
    // Serial.println(Output);
    // Serial.print("Current_Speed: ");
    // Serial.print(Current_Speed);
    // Serial.println(" RPM");
    // Serial.print("Target_Speed: ");
    // Serial.print(Target_Speed);
    // Serial.println(" RPM");
    ////每秒上报一次状态
    if (Time_Conut >= 30) {
      //2025.4.7电机往复测试修改时间计数为30，增加速度往复
      if (Target_Speed == 1000) {
        Target_Speed = -1000;
      } else {
        Target_Speed = 1000;
      }
      Time_Conut = 0;
      SendData_1000ms();
      // Serial.println("OK");
    }
    pulseCount = 0;          // 重置脉冲计数
    lastTime = currentTime;  // 更新计算时间
    Time_Conut += 1;         //每0.1秒计数+1,到达10次进行上报
  }
}