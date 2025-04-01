#include <AccelStepper.h>

//导入exoCAN
#include <eXoCAN.h>

//导入AS5600库
#include <AS5600.h>
// 定义 DRV8825 驱动引脚
#define STEP_PIN PA8  // 步进脉冲引脚
#define DIR_PIN PB14  // 方向控制引脚
// #define ENABLE_PIN 8  // 启用控制引脚
#define M0 PC7
#define M1 PC8
#define M2 PC9                       //步进模式控制引脚
#define Step_Angle 62               //步角比为200*8*14/360=160000/360约=62步/度 需要根据电机减速比进行调整
const int32_t Circle_Step = 160000;  //一圈步进值
// 创建一个 AccelStepper 对象
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

//CAN设置
int txMsgID = 115;  //发送报文附带的ID
int rxMsgID = 115;  //准许接收指定附带ID的报文（过滤器控制，未进入loop模式，不会接收自己的报文）
eXoCAN can(STD_ID_LEN, BR250K, PORTA_11_12_XCVR);

//霍尔设置
int16_t Offset_Angle_10 = 0;  //10倍目标偏移角000--3600
float Offset_Angle = 0;       //目标偏移角度-360.00--360.00
float Current_Angle = 0;      //当前角度 0.00--360.00
float Raw_Angle = 0;          //读取的原始角度数值（0.00--360.00）
AS5600 as5600;


//设置CAN接收回调
int id, fltIdx = 0;
bool Target_Flag = true;
bool Set_Zpos_Flag = false;
bool Send_Data_Flag = false;
volatile uint8_t rxData[8];  // 声明一个 8 字节数组来接收数据
void canISR()                // 依照setup中的过滤器配置来接收CAN消息
{
  can.receive(id, fltIdx, rxData);  // 从CAN总线接收数据（接收到的ID，成功匹配消息的过滤器的索引，接收到的数据）
  // Serial.println(id);
  switch (rxData[0]) {
    case 'A':  //更新目标位置
      Target_Flag = true;
      Offset_Angle_10 = (int16_t)((rxData[2] << 8) | rxData[1]);  //将接受数据转为16进制整数目标角度值
      Offset_Angle = Offset_Angle_10 / 10;
      break;
    case 'R':  //上报当前角度
      Send_Data_Flag = true;
      break;
    case 'Z':  //执行零位归零
      Set_Zpos_Flag = true;
      break;
  }
}


//电机控制（结合AS5600读数进行闭环控制（对角度和步数进行实时同步））
int32_t Offset_Step = 0;  //需要对目标步数进行偏移的上位机命令步数
void Ctrl_42() {
  //如果接收到新的Target,设定一次新目标。
  if (Target_Flag) {
    Target_Flag = false;  //执行后标志置否
    Offset_Step = (int)(Offset_Angle * Step_Angle);
    stepper.move(Offset_Step);  //move函数的本质是对targetposition的值进行加减
  }
}


//不需要进行闭环位置控制，只需要实时上报目前位置和预测位置即可（故此处的所谓闭环其实是基于人类控制的闭环）
unsigned long lastTime = 0;          // 上次计算时间
float Target_Offset_Angle = 0;       //目标偏移角度
int16_t Target_Offset_Angle_10 = 0;  //10倍目标偏移角度000--3600
void Closed_Loop() {
  Current_Angle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;
  Target_Offset_Angle = (float)stepper.distanceToGo() / (float)Circle_Step * 360;
  Target_Offset_Angle_10 = Target_Offset_Angle * 10;
  Send_Data();
}

//将霍尔传感器当前位置设为0位，危险操作，请确认角度正确后再执行烧写
void Set_Zpos() {
  if (Set_Zpos_Flag) {
    Set_Zpos_Flag = false;
    int a = as5600.rawAngle();
    as5600.setZPosition(a);
    as5600.burnAngle();
  }
}
//上报当前角度数据
uint8_t txData[8]{};  //整数转化后的数组容器,第0位为状态代码，第1,2位为剩余偏移角度，第3,4位为当前角度的10倍值。
int txDataLen = 8;
void Send_Data() {
  int Current_Angle_10 = (int)(Current_Angle * 10);
  txData[0] = 'A';                                   //正常代码A(angle)
  txData[1] = Target_Offset_Angle_10 & 0xFF;         // 获取最低字节
  txData[2] = (Target_Offset_Angle_10 >> 8) & 0xFF;  // 获取最高字节
  txData[3] = Current_Angle_10 & 0xFF;               // 获取最低字节
  txData[4] = (Current_Angle_10 >> 8) & 0xFF;        // 获取最高字节
  can.transmit(txMsgID, txData, txDataLen);
}
void setup() {
  Serial.begin(115200);

  //CAN初始化
  can.attachInterrupt(canISR);              //注册CAN接收回调
  can.filterMask16Init(0, rxMsgID, 0x7ff);  // 设置CAN过滤器0，准许IDrxMsgID,掩码0x7ff（即11位‘1’）,基础CAN协议 ID 11位。

  //设置DRV8825为8步进模式
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);

  //AS5600 IIC设置及启动
  Wire.setSDA(PB11);  //SDA
  Wire.setSCL(PB10);  //SCL
  Wire.begin();

  as5600.begin();                                 //可传入引脚作为dir引脚，此处未连接物理dir引脚，将自动切换到软件方向控制。
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  //设置逆时针或顺时针为正方向。AS5600_CLOCK_WISE|AS5600_COUNTERCLOCK_WISE

  // 设置步进电机的最大速度、加速度和初始速度
  stepper.setPinsInverted(false);  //设置步进电机正方向
  stepper.setMaxSpeed(10000);      // 设置最大速度（步/秒）
  stepper.setAcceleration(10000);  // 设置加速度（步/秒^2）
  stepper.setSpeed(10000);         // 设置初始速度（步/秒）减速比50,步进8,200步/圈，转速=speed/50/8/200*60（建议保持在步进电机转——矩曲线峰值范围内200-400rpm）
}

void loop() {
  Set_Zpos();     //AS5600零位归零
  Ctrl_42();      //42目标位置更新
  stepper.run();  // 使电机朝着目标位置移动（自动计算加减速，每次调用只走一步）
  unsigned long currentTime = millis();
  // 每0.5秒钟上报一次角度
  if (currentTime - lastTime >= 500 || Send_Data_Flag == true) {
    if (Send_Data_Flag) {
      Send_Data_Flag = false;
    }
    // Serial.println("close running");
    Closed_Loop();
    lastTime = currentTime;  // 更新计算时间
  }
}
