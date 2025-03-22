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
#define M2 PC9          //步进模式控制引脚
#define Step_Angle 444  //步角比
// 创建一个 AccelStepper 对象
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

//CAN设置
int txMsgID = 119;  //发送报文附带的ID
int rxMsgID = 119;  //准许接收指定附带ID的报文（过滤器控制，未进入loop模式，不会接收自己的报文）
eXoCAN can(STD_ID_LEN, BR250K, PORTA_11_12_XCVR);

//霍尔设置
int16_t Target_Angle_10 = 0;
float Target_Angle = 0;   //目标角度-450-450
float Current_Angle = 0;  //当前角度-45-45
float Raw_Angle = 0;      //读取的原始角度数值（315.00-360.00,0.00-45.00）
uint8_t ZMCO = 0;
AS5600 as5600;


//设置CAN接收回调
int id, fltIdx = 0;
bool Request_Flag = false;
bool Target_Flag = true;  //每次重启后默认设置一次目标进行归位
bool Set_Zpos_Flag = false;
bool Send_Data_Flag = false;
bool Error_Flag = false;       //错误标志
volatile uint8_t rxData[8]{};  // 声明一个 8 字节数组来接收数据
void canISR()                  // 依照setup中的过滤器配置来接收CAN消息
{
  can.receive(id, fltIdx, rxData);  // 从CAN总线接收数据（接收到的ID，成功匹配消息的过滤器的索引，接收到的数据）
  // Serial.println(id);
  switch (rxData[0]) {
    case 'A':  //更新目标位置
      Target_Flag = true;
      Target_Angle_10 = (int16_t)((rxData[2] << 8) | rxData[1]);  //将接受数据转为16进制整数目标角度值
      Target_Angle = Target_Angle_10 / 10;
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
//步角比为200*8*100/360=160000/360约=444步/度
int16_t Target_Step = 0;  //转化后的目标位置（步数）
void Ctrl_42() {
  //如果接收到新的Target,设定一次新目标。
  if (Target_Flag) {
    Target_Flag = false;                              //执行后标志置否
    Target_Angle = constrain(Target_Angle, -45, 45);  //确保最终角度值范围在-45.0-45.0之间
    Target_Step = (int)(Target_Angle * Step_Angle);
    stepper.moveTo(Target_Step);
  }
}


//每0.1秒将当前position和Current_Angle进行同步
unsigned long lastTime = 0;  // 上次计算时间
int Time_Conut = 0;
int16_t Current_Step = 0;  //霍尔获取的实际位置
void Closed_Loop() {
  Raw_Angle = as5600.readAngle() * AS5600_RAW_TO_DEGREES;  //传感器角度范围为315.0.——360.00,0——45.00。
  //实际角度范围为-45——45
  Current_Angle = (Raw_Angle > 180) ? (Raw_Angle - 360) : Raw_Angle;  //Raw_Angle > 180 时，Current_Angle = Raw_Angle - 360。否则，直接赋值 Raw_Angle。
  Error_Flag = (Current_Angle > 45 || Current_Angle < -45);           //如果超过实际可达范围，错误标志置真
  Current_Step = Current_Angle * Step_Angle;                          //霍尔获取的实际位置

  // stepper.updateCurrentPosition(Current_Step);
  if ((abs(stepper.currentPosition() - Current_Step) >= 400) && stepper.distanceToGo() == 0) {  //考虑到转向以流畅性为优先，故选择在一次完整运动完成后再进行闭环控制补齐缺失的步数
    // stepper.setCurrentPosition(Current_Step);
    stepper.updateCurrentPosition(Current_Step);
  }
  // stepper.moveTo(Target_Step);
  // Serial.println("UPDATE");

  // Serial.print("Step_Angle");
  // Serial.println(Step_Angle);
  // Serial.print("CurrentPosition:");
  // Serial.println(stepper.currentPosition());
  // Serial.print("Distancetogo:");
  // Serial.println(stepper.distanceToGo());
  // Serial.print("Current_Step:");
  // Serial.println(Current_Step);
  // Serial.print("Target_Step:");
  // Serial.println(Target_Step);
  // Serial.print("speed:");
  // Serial.println(stepper.speed());
  // Serial.print("targetPosition:");
  // Serial.println(stepper.targetPosition());
  // Serial.print("rawAngle:");
  // Serial.println(as5600.rawAngle());
  // Serial.println(Raw_Angle);
  // Serial.println(Current_Angle);
}

//将霍尔传感器当前位置设为0位，危险操作，请确认角度正确后再执行烧写
void Set_Zpos() {
  if (Set_Zpos_Flag) {
    Set_Zpos_Flag = false;
    int a = as5600.rawAngle();
    as5600.setZPosition(a);
    as5600.burnAngle();
    delay(100);
  }
}
//上报当前角度数据
uint8_t txData[8]{};  //整数转化后的数组容器,第0位为状态代码，第1,2位为目标角度，第3,4位为当前角度。
int txDataLen = 8;
void Send_Data() {
  int Current_Angle_10 = (int)(Current_Angle * 10);
  txData[0] = Error_Flag ? 'E' : 'A';  //依据Error_Flag状态进行三元运算。正常代码A(angle)，错误代码E
  Error_Flag = false;
  txData[1] = Target_Angle_10 & 0xFF;          // 获取最低字节
  txData[2] = (Target_Angle_10 >> 8) & 0xFF;   // 获取最高字节
  txData[3] = Current_Angle_10 & 0xFF;         // 获取最低字节
  txData[4] = (Current_Angle_10 >> 8) & 0xFF;  // 获取最高字节
  txData[5] = ZMCO;                            //当前传感器归零次数
  can.transmit(txMsgID, txData, txDataLen);
}
void setup() {
  Serial.begin(115200);

  //CAN初始化
  can.attachInterrupt(canISR);              //注册CAN接收回调
  can.filterMask16Init(0, rxMsgID, 0x7ff);  // 设置CAN过滤器0，准许IDrxMsgID,掩码0x7ff

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

  as5600.begin();                          //可传入引脚作为dir引脚，此处未连接物理dir引脚，将自动切换到软件方向控制。
  as5600.setDirection(AS5600_CLOCK_WISE);  //设置顺时针或逆时针为正方向。AS5600_CLOCK_WISE|AS5600_COUNTERCLOCK_WISE
  ZMCO = as5600.getZMCO();                 //获取当前传感器已归零次数
  // 设置步进电机的最大速度、加速度和初始速度
  stepper.setPinsInverted(false);  //设置步进电机正方向
  stepper.setMaxSpeed(10000);      // 设置最大速度（步/秒）
  stepper.setAcceleration(10000);  // 设置加速度（步/秒^2）
  stepper.setSpeed(10000);         // 设置初始速度（步/秒）减速比50,步进8,200步/圈，转速=speed/50/8/200
}

void loop() {
  Set_Zpos();     //AS5600零位归零
  Ctrl_42();      //42目标更新
  stepper.run();  // 使电机朝着目标位置移动（自动计算加减速，每次调用只走一步）
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 100) {  // 每0.2秒钟计算一次
    // Serial.println("close running");
    Closed_Loop();
    ////每秒自动上报一次状态或收到指令后上报
    if (Time_Conut >= 10 || Send_Data_Flag) {
      Send_Data();
      Time_Conut = 0;
      Send_Data_Flag = false;
      // Serial.println("sending");
    }
    Time_Conut += 1;
    lastTime = currentTime;  // 更新计算时间
  }
}
