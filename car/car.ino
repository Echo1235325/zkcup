#include <FlexiTimer2.h>
#include <Servo.h> //舵机操作



// Servo
Servo STE_turn; //云台舵机
Servo STE_cat;  //爪子舵机
int Flag_STEturn = 0;
int pos = 0;
int angle = 0;

// 超声波
unsigned int H_echoPin = 12;   // 将Arduino 的Pin2 连接至US-100 的Echo/RX
unsigned int H_trigPin = 13;   // 将Arduino 的Pin3 连接至US-100 的Trig/TX
unsigned int L_echoPin = 10;   // 将Arduino 的Pin2 连接至US-100 的Echo/RX
unsigned int L_trigPin = 11;   // 将Arduino 的Pin3 连接至US-100 的Trig/TX
int Flag = 0;

// Stepper
const int PLS = 5;
const int DIR = 6;
const int STEP_ENA = 23;
int Step_val = 0;
int Step_Flag = 0;

typedef struct point{
  int x;
  int y;
} Point;

typedef struct quene{     // 用于储存路径的队列
  int front;
  int rear;
  Point Road[80];
} Quene;

bool Map[12][12] = {{0,0,0,0,0,0,0,0,0,0,0,0},
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,1,1,1,0,0,0,0,1,1,1,0},
                    {0,1,1,1,0,0,0,0,1,1,1,0},
                    {0,1,1,1,0,0,0,0,1,1,1,0},
                    {0,1,1,1,0,0,0,0,1,1,1,0},
                    {1,1,1,1,1,1,1,1,1,1,1,0},
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,0,0,0,0,0,0,0,0,0,0,0}};

static bool IsFullA[2][6];     
static bool IsFullB[2][6];
static bool IsFullC[2][6];
static bool IsFullD[2][6];


// 中央购物车是否已经抓完
static bool Haved_Been_CatchA[3];
static bool Haved_Been_CatchB[3];
static bool Haved_Been_CatchC[3];
static bool Haved_Been_CatchD[3];


/* 引脚定义 */
int ENCODER_L_A = 20;
int ENCODER_L_B = 22;
int ENCODER_R_A = 21;
int ENCODER_R_B = 19;
int collide_1 = 4; //碰撞开关
int collide_2 = 2; 
int IN1 = 32;
int IN2 = 34;
int IN3 = 36;
int IN4 = 38; //引脚改动
int PWMA = 7; //左轮
int PWMB = 8;

//下为红外部分引脚
const int Red_Forward_0 = 24;   // 背对从左往右为  ， 0->3 
const int Red_Forward_1 = 25;
const int Red_Forward_2 = 26;
const int Red_Forward_3 = 27;
const int Red_Center_0 = 28;   // 从前开始，逆时针，0->3
const int Red_Center_1 = 29;
const int Red_Center_2 = 30;
const int Red_Center_3 = 31;


// 超声波
bool Scan_State[3];     // 三个超声波的检测状态

//控制标志
int Flag_Turn = 0;
int Flag_Forward = 0,Flag_Backward = 0,Flag_Left = 0,Flag_Right = 0;
int Receive_Data = 0;
int Flag_Begin = 0;
int Flag_Count = 0;

//运行数值
int V_NOW_L ,V_NOW_R ;//车轮当前速度 ，R -> PWMA
int setpoint_L = 12;
int setpoint_R = 12;
#define NormalSpeed 40        // PID启动时存在较大超调量可能是NormalSpeed过大
#define Value_Setpoint 12
int Value_Red_ForWard[4] = {0};
int Value_Red_Center[4] = {0};
int Direction = 0;  //车头方向


//  5-1 运动模态
int M;  // 底层运动=1, 上层运动=2, 拍摄=3
int move_state;   //

//  pid
double PulseCounter_L = 0, PulseCounter_R = 0;    //左右电机编码器脉冲计数器
int time;
int count = 0;
static int error_last1, error_int1, derror1;
static int error_last2, error_int2, derror2;
int count2 = 0;
int count_Ti = 5;

// 位置坐标管理
Quene Q;
Point Target_Point, tem_Point;      // 目标点和临时点
Point Next_Point, Now_Point;

// 物体识别得到的货架编号：A B C D
char Shelf;

// 找空货架的时候返回的空货架列编号
int Empty_Column;      

void Enquene( Point P ){
    Q.Road[Q.front++] = P;
    return;
}

Point Dequene(){
    return Q.Road[++Q.rear];
}

bool QueneIsEmpty(){
    return Q.front - Q.rear == 1;
}


void Start_PID(void){
  PulseCounter_L = 0, PulseCounter_R = 0;
  error_last1 = error_int1 = derror1 = 0;
  error_last2 = error_int2 = derror2 = 0;
  FlexiTimer2::start();
}

void Stop_PID(void){
  FlexiTimer2::stop();
}





 /**************************************************************************
函数功能：读取红外传感当前电平  作者：Ding
入口参数：无
返回  值：无
**************************************************************************/
void Read_RedValue(){
    Value_Red_ForWard[0] = digitalRead(Red_Forward_0);
    Value_Red_ForWard[1] = digitalRead(Red_Forward_1);
    Value_Red_ForWard[2] = digitalRead(Red_Forward_2);
    Value_Red_ForWard[3] = digitalRead(Red_Forward_3);
    Value_Red_Center[0] = digitalRead(Red_Center_0);
    Value_Red_Center[1] = digitalRead(Red_Center_1);
    Value_Red_Center[2] = digitalRead(Red_Center_2);
    Value_Red_Center[3] = digitalRead(Red_Center_3);
}


/**************************************************************************
函数功能：车轮电机停止  作者：Ding
入口参数：无
返回  值：无
**************************************************************************/
void Stop1(){
    Receive_Data = 0;
    Flag_Begin = 0;
    digitalWrite(PWMA, LOW);          //TB6612控制引脚拉低
    digitalWrite(PWMB, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void Stop(){
    Stop_PID();
    digitalWrite(PWMA, LOW);          //TB6612控制引脚拉低
    digitalWrite(PWMB, LOW);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    // delay(300);
    // Start_PID();
}



/**************************************************************************
函数功能：限制PWM大小 
入口参数：无
返回  值：无
**************************************************************************/
void Limit_Pwm(void)
{
  int Amplitude = 250;  //===PWM满幅是255 限制在250
  if (V_NOW_R < -Amplitude) V_NOW_R = -Amplitude;
  if (V_NOW_R > Amplitude)  V_NOW_R = Amplitude;
  if (V_NOW_L < -Amplitude) V_NOW_L = -Amplitude;
  if (V_NOW_L > Amplitude)  V_NOW_L = Amplitude;
}

/**************************************************************************
函数功能：pid速度计算  
入口参数：无
返回  值：无
**************************************************************************/
double pid1(int Pulse, float Kp, float Ki, float Kd){
    int error;
    static float v;
    error = setpoint_L - Pulse;
    error_int1 += error;
    derror1 = error - error_last1;
    v = Kp * error + Ki * error_int1 + Kd * derror1;
    error_last1 = error;
// //    Serial.print("error = ");
//     Serial.print(' ');
//     Serial.print(error);
// //    Serial.print("    error_int1 = ");
//     Serial.print(' ');
//     Serial.print(error_int1);
// //    Serial.print("    derror1 = ");
//     Serial.print(' ');
//     Serial.print(derror1);
    return v;
}

double pid2(int Pulse, float Kp, float Ki, float Kd){
    int error;
    static float v;
    error = setpoint_R - Pulse;
    error_int2 += error;
    derror2 = error - error_last2;
    v = Kp * error + Ki * error_int2 + Kd * derror2;
    error_last2 = error;
// //    Serial.print("error = ");
//     Serial.print(' ');
//     Serial.print(error);
// //    Serial.print("    error_int2 = ");
//     Serial.print(' ');
//     Serial.print(error_int2);
// //    Serial.print("    derror2 = ");
//     Serial.print(' ');
//     Serial.println(derror2);
    return v;
}

void calculate_pid(){
    double Kp = 1.65, Ki = 1/count_Ti*Kp, Kd = 5*Kp;
    if (Flag_Begin) {
        if(count2 >= 0){
        // time = millis();
        // Serial.print(time);
        // Serial.print(' ');
        // Serial.print(PulseCounter_L);
        // Serial.print(' ');
        // Serial.print(PulseCounter_R);
        // Serial.print(' ');
        // Serial.print(V_NOW_L);
        // Serial.print(' ');
        // Serial.print(V_NOW_R);
        count2 = 0;
      }
  
      V_NOW_L += pid1(PulseCounter_L, Kp, Ki, Kd);
      V_NOW_R += pid2(PulseCounter_R, Kp, Ki, Kd);

      Limit_Pwm();
      Set_Pwm();
      PulseCounter_L = PulseCounter_R = 0;
      if (count > count_Ti) {
        count = 0;
        error_int1 = error_int2 = 0;
      }
      count++;
      count2++;
    }
}



/**************************************************************************
函数功能：这里根据V_NOW_L和V_NOW_R的正负决定电机的转动方向, 并且给PWMA、PWMB赋值
入口参数：MotorA, MotorB
返回  值：无
**************************************************************************/
void Set_Pwm(){
  int DIFFERENCE = 1;
  if (V_NOW_L > 0)     digitalWrite(IN1, HIGH),      digitalWrite(IN2, LOW);  //TB6612的电平控制
  else             digitalWrite(IN1, LOW),       digitalWrite(IN2, HIGH); //TB6612的电平控制
  analogWrite(PWMA, abs(V_NOW_L) - DIFFERENCE); 
  if (V_NOW_R > 0) digitalWrite(IN3, HIGH),     digitalWrite(IN4, LOW); //TB6612的电平控制
  else        digitalWrite(IN3, LOW),      digitalWrite(IN4, HIGH); //TB6612的电平控制
  analogWrite(PWMB, abs(V_NOW_R));
}


/**************************************************************************
函数功能：设置直行倒车，左右转所使用的PID setpoint
入口参数：Receive_Data
返回  值：无
**************************************************************************/
void Control(int Receive_Data){
    switch (Receive_Data)   {
      case 0x01: Flag_Forward = 1, Flag_Backward = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
      case 0x02: Flag_Forward = 0, Flag_Backward = 1, Flag_Left = 0, Flag_Right = 0;   break;              //后转
      case 0x03: Flag_Forward = 0, Flag_Backward = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
      case 0x04: Flag_Forward = 0, Flag_Backward = 0, Flag_Left = 1, Flag_Right = 0;   break;              //左转
      default: break;
  }
    if(Flag_Forward) {
        setpoint_L = Value_Setpoint;
        setpoint_R = Value_Setpoint;    // 改了这里 想看一下左右轮的一致性, 但会影响超调量
    }
    if(Flag_Backward) {
        V_NOW_L = -160;
        V_NOW_R = -160;
        Limit_Pwm();
        Set_Pwm();
    }
    if(Flag_Left) {
        V_NOW_L = -165;
        V_NOW_R = 165;
        Limit_Pwm();
        Set_Pwm();
//        setpoint_L = -Value_Setpoint;
//        setpoint_R = Value_Setpoint;
    }
    if(Flag_Right) {
        V_NOW_L = 165;
        V_NOW_R = -165;
        Limit_Pwm();
        Set_Pwm();
//        setpoint_L = Value_Setpoint;
//        setpoint_R = -Value_Setpoint;
    }
}


/**************************************************************************
函数功能：控制小车转向  作者：Ding
入口参数：Receive_Data
返回  值：无
**************************************************************************/
void Turn_Left(){
//  Serial.println("左转！");
 Stop_PID();
 Control(2);
 delay(380);
 Control(4);
 delay(380);
 Read_RedValue();
 while(!( Value_Red_ForWard[0] && !Value_Red_ForWard[1] && 
            Value_Red_ForWard[2] && Value_Red_ForWard[3] )){
            Read_RedValue();
      }
  Direction = (Direction + 1) % 4;
  Start_PID();
  delay(150);
}

void Turn_Right(){
 Stop_PID();
 Control(2);
 delay(380);
 Control(3);
 delay(380);
 Read_RedValue();
 while(!( Value_Red_ForWard[0] && Value_Red_ForWard[1] && 
            !Value_Red_ForWard[2] && Value_Red_ForWard[3] )){
            Read_RedValue();
      }
  Direction = (Direction - 1) % 4;
  if (Direction < 0) {
    Direction += 4;
  }
//    Stop1();
  Start_PID();
  delay(150);     // 防止转弯后误触发
}




void Turn_Around(){
  Stop_PID();
   V_NOW_L = -155;
   V_NOW_R = 155;
   Limit_Pwm();
   Set_Pwm();
   delay(350);
   Read_RedValue();
   while(!( !Value_Red_ForWard[0] && Value_Red_ForWard[1] && 
              Value_Red_ForWard[2] && Value_Red_ForWard[3] )){
              Read_RedValue();
        }
  Direction = (Direction + 1) % 4;
  delay(100);
  Read_RedValue();
   while(!( !Value_Red_ForWard[0] && Value_Red_ForWard[1] && 
              Value_Red_ForWard[2] && Value_Red_ForWard[3] )){
              Read_RedValue();
        }
  Direction = (Direction + 1) % 4;
  Start_PID();
}

/**************************************************************************
函数功能：红外传感纠偏  作者：Ding
入口参数：无
返回  值：无
**************************************************************************/
void Rectify(){
    int bias = 1;
    if(Value_Red_ForWard[0] && Value_Red_ForWard[1] &&//左中出界
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                setpoint_R = Value_Setpoint - bias - 1;
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] &&//右中出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                setpoint_L = Value_Setpoint - bias - 1;
    }
//    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //停止
//            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
//                Stop();
//    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //正常
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                setpoint_L = setpoint_R = Value_Setpoint;
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //过线的时候, 正常
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                setpoint_L = setpoint_R = Value_Setpoint;
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左小出界
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 setpoint_R = Value_Setpoint - bias;
    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //右小出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 setpoint_L = Value_Setpoint - bias;
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左大出界
            Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                 setpoint_R = Value_Setpoint - bias-2;
    }
    else if(!Value_Red_ForWard[0] && Value_Red_ForWard[1] && //右大出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 setpoint_L = Value_Setpoint - bias-2;
    }
 }
 
void Exam_arrval_Point(void){
    if( Value_Red_Center[1] && Value_Red_Center[3]){
        Flag_Count = 0;
    }
    if( !Value_Red_Center[1] && !Value_Red_Center[3]){
            if(Flag_Count == 0){
              Flag_Count = 1;
              Now_Point = Next_Point;
              if (!QueneIsEmpty()) {
                Next_Point = Dequene();
                if(!QueneIsEmpty() && (Now_Point.x == Next_Point.x  &&  Now_Point.y == Next_Point.y)){
                  Next_Point = Dequene();
                }
                Serial.println("Dequene:----------------------------");
              }
              else{
                Now_Point = Next_Point;
                Serial.println("Quene Is Empty!");
              }
                Serial.print("Now_Point = (");
                Serial.print(Now_Point.x);
                Serial.print(", ");
                Serial.print(Now_Point.y);
                Serial.print(")   ");
                Serial.print("Next_Point = (");
                Serial.print(Next_Point.x);
                Serial.print(", ");
                Serial.print(Next_Point.y);
                Serial.println(")");
            }
       }
}



Point T_Point;

void Movement_block(int x1, int y1, int x2, int y2) {       // 从A到B的巡线模块
  Path_Planning(x1, y1, x2, y2);
  Now_Point = Dequene();
  Next_Point = Dequene();
  Start_PID();
  T_Point = {x2, y2};
  int end_while = 0;
  Flag_Count = 1;
  while(end_while == 0){
    Read_RedValue();
    Exam_arrval_Point();
    Cal_Direction();
    switch (move_state){
      case 1:
        Read_RedValue();
        Rectify();
        break;
      case 2:
        Turn_Left();
        break;
      case 3:
        Turn_Right();
        break;
      case 4:
        Turn_Around();
        break;
      case 5:
        Stop_PID();
        Control(2);
        delay(300);
        Stop();
        end_while = 1;
        break;      
      default:
        Serial.println("Movement_block, case异常情况");
        break;
    }
  }
}


//
//
//void Catch_Move(char BuyCar){
//    // 先决定左转 右转 或不转
//    switch (BuyCar)
//    {
//      case 'A':
//        switch (Direction)
//        {
//          case 0:
//            Turn_Left();
//            Serial.println("Catch_Move: A, Direction = 0, Turn_Left");
//            break;
//          case 1:
//            Serial.println("Catch_Move: A, Direction = 1, Nop");
//            break;
//          case 2:
//            Turn_Right();
//            Serial.println("Catch_Move: A, Direction = 2, Turn_Right");
//            break;
//          case 3:
//            Serial.println("Catch_Move: A 旋转方向异常");
//        }
//        break;
//      case 'B':
//        switch (Direction)
//        {
//          case 0:
//            Serial.println("Catch_Move: B 旋转方向异常");
//            break;
//          case 1:
//            Turn_Left();
//            Serial.println("Catch_Move: B, Direction = 1, Turn_Left");
//            break;
//          case 2:
//            Serial.println("Catch_Move: B, Direction = 2, Nop");
//            break;
//          case 3:
//            Turn_Right();
//            Serial.println("Catch_Move: B, Direction = 3, Turn_Right");
//            break;
//        }
//        break;
//      case 'C':
//        switch (Direction)
//        {
//          case 0:
//            Turn_Right();
//            Serial.println("Catch_Move: C, Direction = 0, Turn_Right");
//            break;
//          case 1:
//            Serial.println("Catch_Move: C, Direction = 1, 旋转方向异常");
//            break;
//          case 2:
//            Turn_Left();
//            Serial.println("Catch_Move: C, Direction = 2, Turn_Left");
//            break;
//          case 3:
//            Serial.println("Catch_Move: C, Direction = 3, Nop");
//            break;
//        }
//        break;
//      case 'D':
//        switch (Direction)
//        {
//          case 0:
//            Serial.println("Catch_Move: D, Direction = 0, Nop");
//            break;
//          case 1:
//            Turn_Right();
//            Serial.println("Catch_Move: D, Direction = 1, Turn_Right");
//            break;
//          case 2:
//            Serial.println("Catch_Move: D, Direction = 2, 旋转方向异常");
//            break;
//          case 3:
//            Turn_Left();
//            Serial.println("Catch_Move: D, Direction = 3, Turn_Left");
//            break;
//        }
//        break;
//    }
//    Control(1);
//    Start_PID();
//    int time3;
//    time3 = millis();
//    while(millis() - time3 <= 1500){
//      Read_RedValue();
//      Rectify();
//    }
//    Stop();  
//
//    ///// 拍摄 + 抓货物 + 给出下一个目标点
//    Catch_items(BuyCar);
//    Control(2);     // 倒车
//    delay(1500);
//    Stop();         // 完成
//    Movement_block(Now_Point.x, Now_Point.y, Target_Point.x, Target_Point.y);
//    
//}
//
//
///* ********************************************************************
// * 抓取部分, 包括识别和并给出下一个目标点的坐标 Target_Point
// * *******************************************************************/
//void Catch_items(char BuyCar){
//  int Left = 0;
//  int Center = 1;
//  int Right = 2;
//  int *p;
//  int t;
//  Point QuWuDian[4] = {{8, 5}, {6, 8}, {3, 6}, {5, 3}};
//  int QuWuDianBianHao;
//  switch (BuyCar)
//  {
//    case 'A':
//      *p = Haved_Been_CatchA;
//      QuWuDianBianHao = 0;
//      break;
//    case 'B':
//      *p = Haved_Been_CatchB;
//      QuWuDianBianHao = 1;
//      break;
//    case 'C':
//      *p = Haved_Been_CatchC;
//      QuWuDianBianHao = 2;
//      break;
//    case 'D':
//      *p = Haved_Been_CatchD;
//      QuWuDianBianHao = 3;
//      break;
//  }
//  if ( p[Center] == false ) {
//    // 云台转到中央物体位置
//    t = Center;
//  }
//  else if ( p[Left] == false ) {
//    // 转到左侧物体位置
//    t = Left;
//  }
//  else if ( p[Right] == false ) {
//    // 转到右侧物体位置
//    t = Right;
//  }
//  Classfy_block();       // 识别
//  p[t] = true;           // 标记这个物品已经被识别过了
//
//
//  if (Empty_Column == -1){
//    // 这个物品对应货架的已经满
//    // 找另一个购物车的货物去抓取
//    // 逻辑应该是看当前购物车是否还有可以抓取的货物，可以抓取的话则去识别并抓取，不然的话换一个购物车抓取。
//    QuWuDianBianHao = (QuWuDianBianHao + 1) % 4;
//    Target_Point = QuWuDian[QuWuDianBianHao]; 
//  }
//  else {
//    // 抓物体的函数
//    switch (Shelf){
//      case 'A':
//        Target_Point = {10, Empty_Column + 1};
//        break;
//      case 'B':
//        Target_Point = {10 - Empty_Column, 10};
//        break;
//      case 'C':
//        Target_Point = {1, 10 - Empty_Column};
//        break;
//      case 'D':
//        Target_Point = {Empty_Column + 1 , 1};
//        break;
//    }
//    Serial.print("开始从");
//    Serial.print("Now_Point = (");
//    Serial.print(Now_Point.x);
//    Serial.print(", ");
//    Serial.print(Now_Point.y);
//    Serial.print(")  到 ");
//    Serial.print("Target_Point = (");
//    Serial.print(Target_Point.x);
//    Serial.print(", ");
//    Serial.print(Target_Point.y);
//    Serial.println(")");
//  }
//  // 移动到对应货架部分未写
//}
//
//
//
///* **********************************************************************
// * 识别部分
// * 返回值: 该物品对应的货架Shelf, 空货架的列编号Empty_Column
// * *********************************************************************/
//void Classfy_block(void){
//// 可能需要做一些检查
//int time1;
//int good;    // 物品代号
//int Flag_Recognize = false;
//time1 = millis();
//Serial.println("C");        // 给nnpred程序发送开始识别指令C
//while(1){  // 等待nnpred计算完成返回计算结果
//  if (Serial.available()) {
//    good = Serial.read();
//    Flag_Recognize = true;
//    break;
//  }
//  if (millis() - time1 >= 10000) {    // 10秒超时
//    break;
//  }
//}
//
//// class_names = ['ADCamilk', 'Blue cube', 'Deluxe', 'Green cube', 'HappyTiger', 'Red cube', 'RedBull', "Rubik's cube", 'SYY', 'Tennis', 'XHPJ', 'Yakult']
////                     a           b          c           d             f             g           h           i           j        k        l        m
//if (Flag_Recognize) {
//  switch (good){
//    case 'a':
//      Shelf = 'B';
//      break;
//    case 'b':
//      Shelf = 'A';
//      break;
//    case 'c':
//      Shelf = 'D';
//      break;
//    case 'd':
//      Shelf = 'A';
//      break;
//    case 'f':
//      Shelf = 'C';
//      break;
//    case 'g':
//      Shelf = 'A';
//      break;
//    case 'h':
//      Shelf = 'C';
//      break;
//    case 'i':
//      Shelf = 'D';
//      break;
//    case 'j':
//      Shelf = 'B';
//      break;
//    case 'k':
//      Shelf = 'D';
//      break;
//    case 'l':
//      Shelf = 'C';
//      break;
//    case 'm':
//      Shelf = 'B';
//      break;
//    default:
//      break;
//  }
//  Serial.print("已识别货物为:");
//  Serial.println(good);
//
//  Empty_Column = Find_Empty_Shelf(Shelf);
//  if (Empty_Column == -1) {
//    return;
//  }
//}
//else{
//  //转串口通信超时处理
//  Serial.println("串口通信超时");
//  Stop1();
//  }
//}
//
//
//// 用来找某一个对应货架中空的货架的列编号，具体的是上货架是空还是下货架是空可以直接由列编号查询
//int Find_Empty_Shelf( char Shelf ){
//  int i;
//  switch (Shelf)
//  {
//    case 'A':
//      for(i = 0; i <= 5; i++)
//      {
//        if(IsFullA[i][0] == false || IsFullA[i][1] == false){
//          break;
//        }
//      }
//      break;
//    case 'B':
//      for(i = 0; i <= 5; i++)
//      {
//        if(IsFullB[i][0] == false || IsFullB[i][1] == false){
//          break;
//        }
//      }
//      break;
//    case 'C':
//      for(i = 0; i <= 5; i++)
//      {
//        if(IsFullC[i][0] == false || IsFullC[i][1] == false){
//          break;
//        }
//      }
//      break;
//    case 'D':
//      for(i = 0; i <= 5; i++)
//      {
//        if(IsFullD[i][0] == false || IsFullD[i][1] == false){
//          break;
//        }
//      }
//      break;
//  }
//  if (i <= 5) {
//    return i;
//  }
//  // 货架没有空位置可放直接返回-1
//  else {
//    return -1;
//  }
//}



/****************************************************
 * 函数功能：超声波检测
 * 参数：对应超声波 TR口，EC口，判断距离distance
 * 返回值: 小于distance时返回true
 ****************************************************/
bool Exam_items(int US_tr_num,int US_ec_num,int distance){
      unsigned long time_echo_us = 0;
      unsigned long dist_mm = 0;
      unsigned long avg_dist = 0;
      unsigned long last_dist = 0;
      int count = 0;
      while(count <20){
        digitalWrite(US_tr_num, LOW);   // 先拉低，以确保脉冲识别正确
        delayMicroseconds(2);         // 等待2us
        digitalWrite(US_tr_num, HIGH);  // 开始通过Trig/Pin 发送脉冲
        delayMicroseconds(12);        // 设置脉冲宽度为12us (>10us)
        digitalWrite(US_tr_num, LOW);   // 结束脉冲
        time_echo_us = pulseIn(US_ec_num, HIGH);          // 计算US-100 返回的脉冲宽度
        if((time_echo_us < 60000) && (time_echo_us > 1))// 脉冲有效范围(1, 60000).
        {
          // dist_mm = (time_echo_us * 0.34mm/us) / 2 (mm)
          dist_mm = time_echo_us*5/29;        // 通过脉冲宽度计算距离.
          count++;
          avg_dist += dist_mm;  
        }
     }
        avg_dist = avg_dist/20; 
        Serial.println(avg_dist);
        if(avg_dist < distance)   return true;
        else return false;
}



void Scan_Echo(void){
  Scan_State[0] = Exam_items(H_trigPin, H_echoPin, 250);
  Scan_State[1] = Exam_items(L_trigPin, L_echoPin, 250);
//  Scan_State[2] = Exam_items(Left_trigPin, Left_echoPin, 100);
}

//bool Init_Scan_Shelf_Flag = 0;
//void Init_Scan_Shelf(void){
// // 扫 A 货架, 高的超声波为0, 低的超声波为1
// Movement_block(8, 1, 10, 1);
// Turn_Left();
// Stop();
// delay(50);
// Scan_Echo();
// IsFullA[0][0] = Scan_State[0];
// IsFullA[1][0] = Scan_State[1];
// 
// for(int i = 1; i <= 5; i++){
//   Movement_block(10, i, 10, i+1);
//   delay(50); 
//   Scan_Echo();
//   IsFullA[0][i] = Scan_State[0];
//   IsFullA[1][i] = Scan_State[1];
//   // Scan_Shelf
// }
//
// // 扫 B 货架
// Movement_block(10, 6, 10, 10);
// Turn_Left();
// Stop();
// delay(50);
// Scan_Echo();
// IsFullB[0][0] = Scan_State[0];
// IsFullB[1][0] = Scan_State[1];
//
// for(int i = 10; i >=6 ; i--){
//   Movement_block(i, 10, i-1, 10);
//   delay(50); 
//   Scan_Echo();
//   IsFullB[0][11 - i] = Scan_State[0];
//   IsFullB[1][11 - i] = Scan_State[1];
//   // Scan_Shelf
// }
//
// // 扫 C 货架
// Movement_block(5, 10, 1, 10);
// Turn_Left();
// Stop();
// delay(50);
// Scan_Echo();
// IsFullC[0][0] = Scan_State[0];
// IsFullC[1][0] = Scan_State[1];
// for(int i = 10; i >= 6; i--){
//   Movement_block(1, i, 1, i - 1);
//   delay(50); 
//   Scan_Echo();
//   IsFullC[0][11 - i] = Scan_State[0];
//   IsFullC[1][11 - i] = Scan_State[1];
//   // Scan_Shelf
// }
//
//
// Movement_block(1, 5, 1, 1);
// Turn_Left();
// Stop();
// delay(50);
// Scan_Echo();
// IsFullD[0][0] = Scan_State[0];
// IsFullD[1][0] = Scan_State[1];
// for(int i = 1; i <= 5; i++){
//   Movement_block(1, i, 1, i + 1);
//   delay(50); 
//   Scan_Echo();
//   IsFullD[0][i] = Scan_State[0];
//   IsFullD[1][i] = Scan_State[1];
// }
//
//
// Init_Scan_Shelf_Flag = 1;   // 初始循环结束标志位
// Movement_block(1, 6, 5, 3);   // 直接到D区拿货
//
// while(Count_Get_Car_A + Count_Get_Car_B + Count_Get_Car_C + Count_Get_Car_D != 12){
//   // 没有拿够12件货物之前一直做这个循环
//   // 抓货程序
//   Catch_Move();
//   // 放到货架中
//   Put_Item_To_Shelf();
//   // 返回离货架最近的取货点
// }
//}

void Put_Item_To_Shelf(){
  Movement_block(Now_Point.x, Now_Point.y, Target_Point.x, Target_Point.y);   // Target_Point 在识别函数中给出
  Stop();
  // 这里需要加入接近货架的程序
  // 根据 Shelf 和 Empty_Column 决定是往高出放还是往低处放
}


/* ---------------------------------------------------------------------- */
/* 方向计算程序：根据当前坐标和下一个临接点的坐标计算得到应走的方向 */
/* 输入: Now.Point, Next_Point */
/* 影响: M, move_state */
/* ---------------------------------------------------------------------- */

// 标志当前车头朝向
// Direction = 0 -> y轴正向  右
// Direction = 1 -> x轴负向  上
// Direction = 2 -> y轴负向  左
// Direction = 3 -> x轴正向  下
// 左转时 Direction = ( Direction+1 ) % 4

// 右转时 Direction = ( Direction-1 ) % 4
// if Direction < 0{ Direction += 4 }

void Cal_Direction(void){
/* 先从Next_Point和当前坐标之间的关系得到Target_Direction */

  int Target;
  if (Now_Point.x == Next_Point.x && Now_Point.y == Next_Point.y) {
    move_state = 5;
    return;
  }
//  if (QueneIsEmpty()){
//    return;
//  }
  if (Now_Point.x == Next_Point.x - 1 && Now_Point.y == Next_Point.y) {
    Target = 3;
  }
  else if (Now_Point.x == Next_Point.x + 1 && Now_Point.y == Next_Point.y) {
    Target = 1;
  }
  else if (Now_Point.x == Next_Point.x && Now_Point.y == Next_Point.y - 1) {
    Target = 0;
  }
  else if (Now_Point.x == Next_Point.x && Now_Point.y == Next_Point.y + 1) {
    Target = 2;
  }
  else {
    Serial.println("Error: Cal_Direction中Now_Point和Next_Point的关系错误");
    Serial.print("Now_Point = (");
    Serial.print(Now_Point.x);
    Serial.print(", ");
    Serial.print(Now_Point.y);
    Serial.print(")   ");
    Serial.print("Next_Point = (");
    Serial.print(Next_Point.x);
    Serial.print(", ");
    Serial.print(Next_Point.y);
    Serial.println(")");
    return;
  }
//   Serial.print("当前位置 ");
//   Serial.print("(X ,Y) == (");
//   Serial.print(Position_X);
//   Serial.print(" ,");
//   Serial.print(Position_Y);
//   Serial.println(")");
//   Serial.print("开始调整车头方向,当前Direction: ");
//   Serial.print(Direction);
//   Serial.print("    Target: ");
//   Serial.println(Target);
  if (Target == Direction) {
    // 保持巡线
    M = 1;
    move_state = 1;
  }
  else if (Target - Direction == 2 || Target - Direction == -2) {
    // Serial.println("转180度");
    //转180度
    M = 1;
    move_state = 4;
  }
  else if (Target - Direction == 1 || Target - Direction == -3) {
    // Serial.println("左转90度");
    //左转90度
    M = 1;
    move_state = 2;
  }
  else if (Target - Direction == -1 || Target - Direction == 3) {
    // Serial.println("右转90度");
    //右转90度
    M = 1;
    move_state = 3;
  }
  else{
     Serial.println("Cal_Direction,调整方向异常");
  }
  Serial.print("Cal_Direction: M = ");
  Serial.print(M);
  Serial.print("  move_state = ");
  Serial.print(move_state);
  Serial.print("  Target = ");
  Serial.print(Target);
  Serial.print("  Direction = ");
  Serial.println(Direction);
}




/* ---------------------------------------------------------------------- */
/* 障碍物检查程序，检查地图中的一条直线上是否存在障碍物 */
/* ---------------------------------------------------------------------- */

int i;

bool Check_obstacle_x(int x0,int y1,int y2){  // 检查从(x0, y1)到(x0, y2)的水平运动直线上是否有障碍物
  // 遍历Map中的x0行即可
  bool HaveObastacle = false;
  if (y1 > y2){
    int t;
    t = y1;
    y1 = y2;
    y2 = t;
  }
  for(i = y1; i <= y2; i++){
    if (Map[x0][i] == false) {
      HaveObastacle = true;
      break;
    }
  }
  return HaveObastacle;
}

bool Check_obstacle_y(int y0,int x1,int x2){  // 检查从(x1, y0)到(x2, y0)的垂直运动直线上是否有障碍物
  // 遍历Map中的y0列即可
  bool HaveObastacle = false;
  if (x1 > x2){
    int t;
    t = x1;
    x1 = x2;
    x2 = t;
  }
  for(i = x1; i <= x2; i++){
    if (Map[i][y0] == false) {
      HaveObastacle = true;
      break;
    }
  }
  return HaveObastacle;
}


/* ------------------------------------------------------------------------------------ */
/* 路径规划程序：计算路径并放入队列Q中 */
/* 输入：起始点坐标(x1, y1)， 终点坐标(x2, y2) */
/* 影响：路径队列Q */
/* ------------------------------------------------------------------------------------ */

void Path_Planning_Horizontal(int x0, int y1, int y2){ // 水平移动(x0, y1) -->(x0, y2)
//   Serial.print("水平移动: ");
//   Serial.print("(");
//   Serial.print(x0);
//   Serial.print(",");
//   Serial.print(y1);
//   Serial.print(") --> ");
//   Serial.print("(");
//   Serial.print(x0);
//   Serial.print(",");
//   Serial.print(y2);
//   Serial.println(")");
  if (Check_obstacle_x(x0, y1, y2) == false) {  // 没有障碍物, 直接运动
    //直接运动, 到达指定坐标结束
    tem_Point.x = x0;
     if(y2 > y1){
    // //   Adjust_Direction(0);
    // //   //向右走
    // //   //向右直行直到 y == y2  （while?）
    for( i = y1; i <= y2; i++){
            tem_Point.y = i;
            Enquene(tem_Point);
        }
     }
      //向右直行直到 y == y2  （while?）    
     else if (y2 < y1){
    // //   Adjust_Direction(2);
    // //   //向左走
    // //   //向左直行直到 y == y2  （while?） 
    for( i = y1; i >= y2; i--){
            tem_Point.y = i;
            Enquene(tem_Point);
        }
     }
     }   

  else{ // 如果有障碍物
    if(x0 <= 5){
      // 移动到x = 3, 即(x0, y1) --> (3, y1)
      Path_Planning_Vertical(y1, x0, 3);
      // 然后再从 (3, y1) --> (3, y2)
      Path_Planning_Horizontal(3, y1, y2);
      // (3, y2) --> (x0, y2)
      Path_Planning_Vertical(y2, 3, x0);
    }
    else{
      // (x0, y1) --> (8, y1)
      Path_Planning_Vertical(y1, x0, 8);
      // (8, y1) --> (8, y2)
      Path_Planning_Horizontal(8, y1, y2);
      // (8, y2) --> (x0, y2)
      Path_Planning_Vertical(y2, 8, x0);
    }
  }
}

void Path_Planning_Vertical(int y0, int x1, int x2){ // 垂直移动(x1, y0) -->(x2, y0)
//   Serial.print("垂直移动: ");
//   Serial.print("(");
//   Serial.print(x1);
//   Serial.print(",");
//   Serial.print(y0);
//   Serial.print(") --> ");
//   Serial.print("(");
//   Serial.print(x2);
//   Serial.print(",");
//   Serial.print(y0);
//   Serial.println(")");
  if (Check_obstacle_y(y0, x1, x2) == false) {  // 没有障碍物, 直接运动
//   Serial.println("不存在障碍");
    //直接运动, 到达指定坐标结束
    tem_Point.y = y0;
    if(x2 > x1){
      //向下走
      //直到y == y2
    //   Adjust_Direction(3);
        for(i = x1; i <= x2; i++){
            tem_Point.x = i;
            Enquene(tem_Point);
        }
    }
    else if (x2 < x1){
      //向上走
    //   Adjust_Direction(1);
        for(i = x1; i >= x2; i--){
            tem_Point.x = i;
            Enquene(tem_Point);
        }
    }
  }
  else{ // 如果有障碍物
//   Serial.println("存在障碍");  
    if(y0 <= 5){
    //   // 移动到y = 3, 即(x1, y0) --> (x1, 3)
      Path_Planning_Horizontal(x1, y0, 3);
    //   // 然后再从 (x1, 3) --> (x2, 3)
      Path_Planning_Vertical(3, x1, x2);
    //   // (x2, 3) --> (x2, y0)
      Path_Planning_Horizontal(x2, 3, y0);
    }
    else{
    //   // (x1, y0) --> (x1, 8)
      Path_Planning_Horizontal(x1, y0, 8);
    //   // (x1, 8) --> (x2, 8)
      Path_Planning_Vertical(8, x1, x2);
    //   // (x2, 8) --> (x2, y0)
      Path_Planning_Horizontal(x2, 8, y0);
    }
  }
}

void Path_Planning(int x1, int y1, int x2, int y2){
  Q.rear = -1;
  Q.front = 0;
  Path_Planning_Horizontal(x1, y1, y2);
  Path_Planning_Vertical(y2, x1, x2);
}

/* ---------------------------------------------------------------------- */
/* 左右轮电机脉冲数读取 */
/* ---------------------------------------------------------------------- */
void ReadEncoder_L(void){
  /* A相检测到上升沿时，B为LOW为正转，B为HIGH为反转
     A相检测到下降沿时，B为HIGH为正转，B为LOW为反转 */
  if (digitalRead(ENCODER_L_A) == HIGH) {
    //A相检测到上升沿
    if (digitalRead(ENCODER_L_B) == LOW) {
      //正转
      PulseCounter_L--;
    }
    else{
      //反转
      PulseCounter_L++;
    }
  }

  else{
    //A相检测到下降沿
    if(digitalRead(ENCODER_L_B) == LOW){
      //反转
      PulseCounter_L++;
    }
    else{
      //正转
      PulseCounter_L--;
    }
  }
}

void ReadEncoder_R(void){
  /* A相检测到上升沿时，B为LOW为正转，B为HIGH为反转
     A相检测到下降沿时，B为HIGH为正转，B为LOW为反转 */
  if (digitalRead(ENCODER_R_A) == HIGH) {
    //A相检测到上升沿
    if (digitalRead(ENCODER_R_B) == LOW) {
      //正转
      PulseCounter_R++;
    }
    else{
      //反转
      PulseCounter_R--;
    }
  }

  else{
    //A相检测到下降沿
    if(digitalRead(ENCODER_R_B) == LOW){
      //反转
      PulseCounter_R--;
    }
    else{
      //正转
      PulseCounter_R++;
    }
  }
}


void setup() {
    // put your setup code here, to run once:
    pinMode(IN1, OUTPUT);        //TB6612控制引脚，控制电机1的方向，01为正转，10为反转
    pinMode(IN2, OUTPUT);          //TB6612控制引脚，
    pinMode(IN3, OUTPUT);          //TB6612控制引脚，控制电机2的方向，01为正转，10为反转
    pinMode(IN4, OUTPUT); 
    pinMode(collide_1,INPUT); 
    pinMode(collide_2,INPUT); 
    pinMode(Red_Forward_0,INPUT);
    pinMode(Red_Forward_1,INPUT);
    pinMode(Red_Forward_2,INPUT);
    pinMode(Red_Forward_3,INPUT);
    pinMode(Red_Center_0,INPUT);
    pinMode(Red_Center_1,INPUT);
    pinMode(Red_Center_2,INPUT);
    pinMode(Red_Center_3,INPUT);
    pinMode(ENCODER_L_A, INPUT);
    pinMode(ENCODER_R_A, INPUT); 
    pinMode(ENCODER_L_B, INPUT);
    pinMode(ENCODER_R_B, INPUT); 
    STE_turn.attach(9);  // attaches the servo on pin 9 to the servo object
    STE_cat.attach(3);
    pinMode(PLS,OUTPUT);
    pinMode(DIR,OUTPUT);
    pinMode(STEP_ENA,OUTPUT);

    //初始化
    digitalWrite(IN1, LOW);          //TB6612控制引脚拉低
    digitalWrite(IN2, LOW);          //TB6612控制引脚拉低
    digitalWrite(IN3, LOW);          //TB6612控制引脚拉低
    digitalWrite(IN4, LOW);          //TB6612控制引脚拉低
    analogWrite(PWMA, LOW);          //TB6612控制引脚拉低
    analogWrite(PWMB, LOW);          //TB6612控制引脚拉低

    pinMode(H_echoPin, INPUT);  // 设置echoPin 为输入模式。
    pinMode(H_trigPin, OUTPUT); // 设置trigPin 为输出模式。
    pinMode(L_echoPin, INPUT);  // 设置echoPin 为输入模式。
    pinMode(L_trigPin, OUTPUT); // 设置trigPin 为输出模式。


    Serial.begin(9600);
    attachInterrupt(0,Stop1,LOW); //外部中断，停止
    attachInterrupt(3,ReadEncoder_L, CHANGE);
    attachInterrupt(2,ReadEncoder_R, CHANGE);
    FlexiTimer2::set(10, calculate_pid);   // 定时中断计算pid
    angle_ini();
}


void loop(){
    if(digitalRead(collide_1) == LOW) {
        Flag_Begin = 1;
    }
    if(Flag_Begin == true && move_state == 0){
        move_state = 1;
        Direction = 0;
         Movement_block(10, 9, 10, 10);
         Turn_Left();
         Stop();
         delay(50);
         Scan_Echo();
         IsFullB[0][0] = Scan_State[0];
         IsFullB[1][0] = Scan_State[1];
         for(int i = 10; i >=6 ; i--){
           Movement_block(i, 10, i-1, 10);
           delay(50); 
           Scan_Echo();
           IsFullB[0][11 - i] = Scan_State[0];
           IsFullB[1][11 - i] = Scan_State[1];
           // Scan_Shelf
         }
         for(int i = 0; i < 6; i++){
          Serial.print(IsFullB[0][i]);
          Serial.print(' ');
         }
         Serial.print('\n');
         for(int i = 0; i < 6; i++){
          Serial.print(IsFullB[1][i]);
          Serial.print(' ');
         }
         Serial.print('\n');
    }
 }
