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
unsigned int Left_trigPin = 4;
unsigned int Left_echoPin = 2;

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



Point QuWuDian[4] = {{8, 5}, {6, 8}, {3, 6}, {5, 3}};
int QuWuDianBianHao;

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

static bool IsFullA[2][6] = {0};  
static bool IsFullB[2][6] = {0};   
static bool IsFullC[2][6] = {0};     
static bool IsFullD[2][6] = {0};     

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
int collide_1 = 40; //碰撞开关

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
int setpoint_L = 10;
int setpoint_R = 10;
#define NormalSpeed 40        // PID启动时存在较大超调量可能是NormalSpeed过大
#define Value_Setpoint 10
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

// 购物车编号：A B C D
char BuyCar;

// 找空货架的时候返回的空货架列编号
int Empty_Column;      


// 特殊情况标志位
bool Fangbuxia;
bool BuyCarEmpty;

// 防误触时间
 int time_interval;
 int dtime_interval;

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
//// //    Serial.print("error = ");
//     Serial.print(' ');
//     Serial.print(error);
//// //    Serial.print("    error_int1 = ");
//     Serial.print(' ');
//     Serial.print(error_int1);
//// //    Serial.print("    derror1 = ");
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
//// //    Serial.print("error = ");
//     Serial.print(' ');
//     Serial.print(error);
//// //    Serial.print("    error_int2 = ");
//     Serial.print(' ');
//     Serial.print(error_int2);
//// //    Serial.print("    derror2 = ");
//     Serial.print(' ');
//     Serial.println(derror2);
    return v;
}

void calculate_pid(){
    double Kp = 1.15, Ki = 1/count_Ti*Kp, Kd = 5*Kp;
    if (Flag_Begin) {
        if(count2 >= 0){
//         time = millis();
//         Serial.print(time);
//         Serial.print(' ');
//         Serial.print(PulseCounter_L);
//         Serial.print(' ');
//         Serial.print(PulseCounter_R);
//         Serial.print(' ');
//         Serial.print(V_NOW_L);
//         Serial.print(' ');
//         Serial.print(V_NOW_R);
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
        V_NOW_L = -170;
        V_NOW_R = 170;
        Limit_Pwm();
        Set_Pwm();
//        setpoint_L = -Value_Setpoint;
//        setpoint_R = Value_Setpoint;
    }
    if(Flag_Right) {
        V_NOW_L = 170;
        V_NOW_R = -170;
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
//   Serial.println(" 左转！");
 Stop_PID();
 Control(2);
 delay(200);
 Stop();
 delay(150);
 Control(4);
 delay(450);
 Read_RedValue();
 while(!(!Value_Red_ForWard[0] )){
            Read_RedValue();
      }
  Direction = (Direction + 1) % 4;
  setpoint_L = setpoint_R = Value_Setpoint;
  Start_PID();
  delay(100);
  time_interval = millis();
}

void Turn_Right(){
//   Serial.println(" 右转！");
 Stop_PID();
  Control(2);
  delay(200);
  Stop();
  delay(150);
 Control(3);
 delay(450);
 Read_RedValue();
 while( !(!Value_Red_ForWard[3] ) ){
            Read_RedValue();
      }
  Direction = (Direction - 1) % 4;
  if (Direction < 0) {
    Direction += 4;
  }
//    Stop1();
  setpoint_L = setpoint_R = Value_Setpoint;
  Start_PID();
  delay(100);     // 防止转弯后误触发
  time_interval = millis();
}

void Turn_Left_Not_Back(){
//   Serial.println("not back 左转！");
 Stop_PID();
 Control(4);
 delay(450);
 Read_RedValue();
 while(!( !Value_Red_ForWard[0] )){
            Read_RedValue();
      }
  Direction = (Direction + 1) % 4;
  setpoint_L = setpoint_R = Value_Setpoint;
  Start_PID();
//  Stop();
  delay(100);
    time_interval = millis();
}


void Turn_Right_Not_Back(){
//  Serial.println("not back 右转！");
 Stop_PID();
 Control(3);
 delay(450);
 Read_RedValue();
 while(!(!Value_Red_ForWard[3] )){
            Read_RedValue();
      }
  Direction = (Direction - 1) % 4;
  if (Direction < 0) {
    Direction += 4;
  }
//    Stop1();
  setpoint_L = setpoint_R = Value_Setpoint;
  Start_PID();
  delay(100);     // 防止转弯后误触发
    time_interval = millis();
}



void Turn_Around(){
//  Serial.println("转180度");
   Stop_PID();
   Control(4);
   delay(450);
   Read_RedValue();
   while(!(!Value_Red_ForWard[0] )){
              Read_RedValue();
        }
  Direction = (Direction + 1) % 4;
  delay(380);
  Read_RedValue();
   while(!(!Value_Red_ForWard[1] )){
              Read_RedValue();
        }
  Direction = (Direction + 1) % 4;
  setpoint_L = setpoint_R = Value_Setpoint;
  Start_PID();
  delay(100);
  time_interval = millis();
}

void Tui1Ge(){
   setpoint_L = setpoint_R = -Value_Setpoint + 2;
   Start_PID();
   delay(400);
   Read_RedValue();
   while(!(!Value_Red_Center[1] && !Value_Red_Center[3])){
    Read_RedValue();
    Rectify_I();
   }
   Stop();
   time_interval = millis();
}

void Tui1Ge_Point(){    /// 用于倒退1格之后
   switch(Direction){
    case 0:
      Now_Point.y = Now_Point.y - 1;
    break;
    case 1:
      Now_Point.x = Now_Point.x + 1;
    break;
    case 2:
      Now_Point.y = Now_Point.y + 1;
    break;
    case 3:
      Now_Point.x = Now_Point.x - 1;
    break;
   }
}

/**************************************************************************
函数功能：红外传感纠偏  作者：Ding
入口参数：无
返回  值：无
**************************************************************************/
void Rectify(){
    int bias = 2;
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
//                setpoint_L  = setpoint_L *1.2;
//                setpoint_R  = setpoint_R *1.2;
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


void Rectify_I(){   // 倒车循迹
    int bias = 0;
    if(Value_Red_ForWard[0] && Value_Red_ForWard[1] &&//左中出界
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                setpoint_L = -(Value_Setpoint - bias - 1);
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] &&//右中出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                setpoint_R = -(Value_Setpoint - bias - 1);
    }
//    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //停止
//            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
//                Stop();
//    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //正常
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                setpoint_L = setpoint_R = -Value_Setpoint+1;
    }
    else if(!Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //过线的时候, 正常
            !Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                setpoint_L = setpoint_R = -Value_Setpoint+1;
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左小出界
            !Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 setpoint_L = -(Value_Setpoint - bias);
    }
    else if(Value_Red_ForWard[0] && !Value_Red_ForWard[1] && //右小出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 setpoint_R = -(Value_Setpoint - bias);
    }
    else if(Value_Red_ForWard[0] && Value_Red_ForWard[1] && //左大出界
            Value_Red_ForWard[2] && !Value_Red_ForWard[3]){
                 setpoint_L = -(Value_Setpoint - bias - 2);
    }
    else if(!Value_Red_ForWard[0] && Value_Red_ForWard[1] && //右大出界
            Value_Red_ForWard[2] && Value_Red_ForWard[3]){
                 setpoint_R = -(Value_Setpoint - bias - 2);
    }
 }


 
void Exam_arrval_Point(void){
    if( Value_Red_Center[1] && Value_Red_Center[3]){
        Flag_Count = 0;
    }
    if( !Value_Red_Center[1] && !Value_Red_Center[3]){
            if(Flag_Count == 0){
              dtime_interval = millis() - time_interval;
              time_interval = millis();
              if(dtime_interval < 700){
                return;
              }
              Flag_Count = 1;
              Now_Point = Next_Point;
              if (!QueneIsEmpty()) {
                Next_Point = Dequene();
                if(!QueneIsEmpty() && (Now_Point.x == Next_Point.x  &&  Now_Point.y == Next_Point.y)){
                  Next_Point = Dequene();
                }
//                 Serial.println("Dequene:----------------------------");
              }
              else{
                Now_Point = Next_Point;
//                 Serial.println("Quene Is Empty!");
              }
//                 Serial.print("Now_Point = (");
//                 Serial.print(Now_Point.x);
//                 Serial.print(", ");
//                 Serial.print(Now_Point.y);
//                 Serial.print(")   ");
//                 Serial.print("Next_Point = (");
//                 Serial.print(Next_Point.x);
//                 Serial.print(", ");
//                 Serial.print(Next_Point.y);
//                 Serial.println(")");
            }
       }
}



Point T_Point;

void Movement_block(int x1, int y1, int x2, int y2) {       // 从A到B的巡线模块
  if(x1 == x2 && y1 == y2)
    return ;
  Path_Planning(x1, y1, x2, y2);
  Now_Point = Dequene();
  Next_Point = Dequene();
  Start_PID();
//  delay(200);
  T_Point = {x2, y2};
  int end_while = 0;
  Flag_Count = 1;
  time_interval = millis();
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
        if(Q.rear == 1 ){
          Turn_Left_Not_Back();
        }
        else {
          Turn_Left();
        }
        break;
      case 3:
        if(Q.rear == 1 ){
          Turn_Right_Not_Back();
        }
        else {
          Turn_Right();
        }
        break;
      case 4:
        Turn_Around();
        break;
      case 5:
        Stop_PID();
        Control(2);
        delay(100);
        Stop();
        end_while = 1;
        break;      
      default:
 //       Serial.println("Movement_block, case异常情况");
        break;
    }
  }
}


bool f;

void Catch_Move(){
   // 先决定左转 右转 或不转
   switch (BuyCar)
   {
     case 'A':
       f = 0;
       for (int i = 0; i < 3; i++){
         if (Haved_Been_CatchA[i] == false){
           f = 1;
           break;
         }
       }
       if (f == 0){
         BuyCarEmpty = 1;
         Target_Point = QuWuDian[1];
         BuyCar =  'B';
         return;      // 当前购物车已经全部抓完了
       }
       switch (Direction)
       {
         case 0:
           Turn_Left_Not_Back();
          //  Serial.println("Catch_Move: A, Direction = 0, Turn_Left");
           break;
         case 1:
          //  Serial.println("Catch_Move: A, Direction = 1, Nop");
           break;
         case 2:
           Turn_Right_Not_Back();
          //  Serial.println("Catch_Move: A, Direction = 2, Turn_Right");
           break;
         case 3:
          //  Serial.println("Catch_Move: A 旋转方向异常");
          break;
       }
       break;
     case 'B':
       f = 0;
       for (int i = 0; i < 3; i++){
         if (Haved_Been_CatchB[i] == false){
           f = 1;
           break;
         }
       }
       if (f == 0){
         BuyCarEmpty = 1;
         Target_Point = QuWuDian[2];
         BuyCar =  'C';
         return;      // 当前购物车已经全部抓完了
       }
       switch (Direction)
       {
         case 0:
          //  Serial.println("Catch_Move: B 旋转方向异常");
           break;
         case 1:
           Turn_Left_Not_Back();
          //  Serial.println("Catch_Move: B, Direction = 1, Turn_Left");
           break;
         case 2:
          //  Serial.println("Catch_Move: B, Direction = 2, Nop");
           break;
         case 3:
           Turn_Right_Not_Back();
          //  Serial.println("Catch_Move: B, Direction = 3, Turn_Right");
           break;
       }
       break;

     case 'C':
       f = 0;
       for (int i = 0; i < 3; i++){
         if (Haved_Been_CatchC[i] == false){
           f = 1;
           break;
         }
       }
       if (f == 0){
         BuyCarEmpty = 1;
         Target_Point = QuWuDian[3];
         BuyCar =  'D';
         return;      // 当前购物车已经全部抓完了
       }     
       switch (Direction)
       {
         case 0:
           Turn_Right_Not_Back();
          //  Serial.println("Catch_Move: C, Direction = 0, Turn_Right");
           break;
         case 1:
          //  Serial.println("Catch_Move: C, Direction = 1, 旋转方向异常");
           break;
         case 2:
           Turn_Left_Not_Back();
          //  Serial.println("Catch_Move: C, Direction = 2, Turn_Left");
           break;
         case 3:
          //  Serial.println("Catch_Move: C, Direction = 3, Nop");
           break;
       }
       break;

     case 'D':
       f = 0;
       for (int i = 0; i < 3; i++){
         if (Haved_Been_CatchD[i] == false){
           f = 1;
           break;
         }
       }
       if (f == 0){
         BuyCarEmpty = 1;
         Target_Point = QuWuDian[0];
         BuyCar =  'D';
         return;      // 当前购物车已经全部抓完了
       }     
       switch (Direction)
       {
         case 0:
          //  Serial.println("Catch_Move: D, Direction = 0, Nop");
           break;
         case 1:
           Turn_Right_Not_Back();
          //  Serial.println("Catch_Move: D, Direction = 1, Turn_Right");
           break;
         case 2:
          //  Serial.println("Catch_Move: D, Direction = 2, 旋转方向异常");
           break;
         case 3:
           Turn_Left_Not_Back();
          //  Serial.println("Catch_Move: D, Direction = 3, Turn_Left");
           break;
       }
       break;
   }
   
   ///// 拍摄 + 抓货物 + 给出下一个目标点
   Catch_items();
}


/* ********************************************************************
* 抓取部分, 包括识别和并给出下一个目标点的坐标 Target_Point
* *******************************************************************/
void Catch_items(){
 int Left = 0;
 int Center = 1;
 int Right = 2;
 bool *p;
 int t;
 bool Cat_Flag = 0;     // 云台是否转动过
 if (Step_Flag == 1){     // 如果滑台处于高处, 那么把滑台往下滑
     digitalWrite(STEP_ENA, HIGH);
     Control_Step(0);
     delay(300);
   }
 switch (BuyCar)
 {
   case 'A':
     p = Haved_Been_CatchA;
     QuWuDianBianHao = 0;
     break;
   case 'B':
     p = Haved_Been_CatchB;
     QuWuDianBianHao = 1;
     break;
   case 'C':
     p = Haved_Been_CatchC;
     QuWuDianBianHao = 2;
     break;
   case 'D':
     p = Haved_Been_CatchD;
     QuWuDianBianHao = 3;
     break;
 }
 BuyCarEmpty = 0;
 if ( p[Center] == false ) {
   // 云台转到中央物体位置
   Turn_STE(82,STE_turn);
   delay(400);
   t = Center;
 }
 else if ( p[Left] == false ) {
     // 转到左侧物体位置
     Turn_STE(120,STE_turn);
     delay(400);
     Cat_Flag = 1;
   t = Left;
 }
 else if ( p[Right] == false ) {
   // 转到右侧物体位置
      Turn_STE(60,STE_turn);
      delay(300);
      Cat_Flag = 1;
   t = Right;
 }
 else {     // 这个购物车已经空了, 转到下一个购物车去取货
    BuyCarEmpty = 1;
    QuWuDianBianHao = (QuWuDianBianHao + 1) % 4;
    Target_Point = QuWuDian[QuWuDianBianHao];
    BuyCar =  BuyCar + 1;
    if (BuyCar == 'E'){
      BuyCar = 'A';
    }
    return;       // 直接返回
 }
 
   int time3;           // 往前面开一格巡线
   int time2;
   Control(1);
   Start_PID();
   time3 = millis();
   time2 = millis();
  while(time2 - time3 <= 1750){
    Read_RedValue();
    Rectify();
    time2 = millis();
  }

  //  delay(1050);
   Stop();
   delay(200);

   Classfy_block();       // 识别
//   Shelf = 'A';
   p[t] = true;           // 标记这个物品已经被识别过了
   Fangbuxia = 0;

  // 找空货架
  Empty_Column = Find_Empty_Shelf(Shelf);
  if (Empty_Column == -1){
    // 这个物品对应货架的已经满, 倒车出去重新做Catch_Items
    Fangbuxia = 1;
    return;
  }

   // 抓物体的函数
   Catch();  
   Tui1Ge();        // 退一格
   delay(200);
   if (Cat_Flag == 1){      // 如果爪子运动过 则复位
     Turn_STE(82,STE_turn);
     delay(500);
   }
   switch (Shelf){
     case 'A':
       Target_Point = {10, Empty_Column + 1};
       break;
     case 'B':
       Target_Point = {10 - Empty_Column, 10};
       break;
     case 'C':
       Target_Point = {1, 10 - Empty_Column};
       break;
     case 'D':
       Target_Point = {Empty_Column + 1 , 1};
       break;
   }
 }



/* **********************************************************************
* 识别部分
* 返回值: 该物品对应的货架Shelf, 空货架的列编号Empty_Column
* *********************************************************************/
void Classfy_block(void){
// 可能需要做一些检查
int time1;
int good;    // 物品代号
int Flag_Recognize = false;

delay(200);
Serial.print("C");        // 给nnpred程序发送开始识别指令C
time1 = millis();
while(1){  // 等待nnpred计算完成返回计算结果
  if (Serial.available()) {
    good = Serial.read();
    Flag_Recognize = true;
    break;
  }
}

// class_names = ['ADCamilk', 'Blue cube', 'Deluxe', 'Green cube', 'HappyTiger', 'Red cube', 'RedBull', "Rubik's cube", 'SYY', 'Tennis', 'XHPJ', 'Yakult']
//                     a           b          c           d             e             f           g           h           i        j        k        l
if (Flag_Recognize) {
  switch (good){
    case 'a':
      Shelf = 'B';
      break;
    case 'b':
      Shelf = 'A';
      break;
    case 'c':
      Shelf = 'D';
      break;
    case 'd':
      Shelf = 'A';
      break;
    case 'e':
      Shelf = 'C';
      break;
    case 'f':
      Shelf = 'A';
      break;
    case 'g':
      Shelf = 'C';
      break;
    case 'h':
      Shelf = 'D';
      break;
    case 'i':
      Shelf = 'B';
      break;
    case 'j':
      Shelf = 'D';
      break;
    case 'k':
      Shelf = 'C';
      break;
    case 'l':
      Shelf = 'B';
      break;
    default:
      break;
  }
  Serial.print("B");
}
else{
//  //转串口通信超时处理
////  Serial.println("E: 串口通信超时");
//  Stop1();
  }
}

/* ***************************************************************
 * 用来找某一个对应货架中空的货架的列编号，具体的是上货架是空还是下货架是空可以直接由列编号查询
 * ***************************************************************/
int Find_Empty_Shelf( char Shelf ){
 int i;
 int j;
 switch (Shelf)
 {
   case 'A':
   for (j = 1; j >= 0; j--)
   {
     for(i = 2; i <= 5; i++)
     {
       if(IsFullA[j][i] == false){
         return i;
         break;
       }
     }
     if ( i > 5 ) {
       for(i = 1; i >= 0; i--)
       {
       if(IsFullA[j][i] == false){
         return i;
         break;
         }
       }
     }
   }
     break;

   case 'B':
   for (j = 1; j >= 0; j--)
   {
     for(i = 2; i <= 5; i++)
     {
       if(IsFullB[j][i] == false){
         return i;
         break;
       }
     }
     if ( i > 5 ) {
       for(i = 1; i >= 0; i--)
       {
       if(IsFullB[j][i] == false){
         return i;
         break;
         }
       }
     }
   }
     break;

   case 'C':
     for (j = 1; j >= 0; j--)
   {
     for(i = 2; i <= 5; i++)
     {
       if(IsFullC[j][i] == false){
         return i;
         break;
       }
     }
     if ( i > 5 ) {
       for(i = 1; i >= 0; i--)
       {
       if(IsFullC[j][i] == false){
         return i;
         break;
         }
       }
     }
   }
     break;

   case 'D':
  for (j = 1; j >= 0; j--)
   {
     for(i = 2; i <= 5; i++)
     {
       if(IsFullD[j][i] == false){
         return i;
         break;
       }
     }
     if ( i > 5 ) {
       for(i = 1; i >= 0; i--)
       {
       if(IsFullD[j][i] == false){
         return i;
         break;
         }
       }
     }
   }
     break;
 }

 return -1;
}

Point T1;
Point T2;
void Move_To_Shelf(char Shelf){     // 移动到货架前方, 根据空货架（上/下）决定是否移动滑台
  // 已有 Now_Point 和 Target_Point 以及 Shelf
  bool (*p)[6];
  bool pos;
  switch (Shelf){
     case 'A':
     // 9 <--> 8, 3 <--> 2 可以调
       T1 = {9, 3};
       T2 = {9, Target_Point.y};
       p = IsFullA;
       break;
     case 'B':
       T1 = {8, 8};
       T2 = {Target_Point.x, 8};
       p = IsFullB;
       break;
     case 'C':
       T1 = {3, 8};
       T2 = {3, Target_Point.y};
       p = IsFullC;
       break;
     case 'D':
       T1 = {3, 3};
       T2 = {Target_Point.x, 3};
       p = IsFullD;
       break;
   }
   // 先移动到中继点
   Movement_block(Now_Point.x, Now_Point.y, T1.x, T1.y);
   Movement_block(T1.x, T1.y, T2.x, T2.y);

   
   // 放货函数
   if ( p[1][Empty_Column] == false) {
     pos = 1;
   }
   else if ( p[0][Empty_Column] == false){
     pos = 0;
   }
   else {
//     Serial.println("Error: 尝试把货物放入一个已经满的货架");
   }

   if (pos == 0) {
     // 步进电机上升的函数
     Control_Step(1);
     delay(300);
     p[0][Empty_Column] = true;
   }
   else{
     p[1][Empty_Column] = true;
   }

   // 步进电机下降应该在抓物体的时候完成
   Movement_block(T2.x, T2.y, Target_Point.x, Target_Point.y);      // 进目标点
   delay(100);
   Loose();     // 放货
   delay(300);
   // 倒车一格
   Tui1Ge();
   Tui1Ge_Point();
   delay(230);
}



/************************************************************************
 * 放货后返回取物点的函数
 * **********************************************************************/

void Back_To_QWD(){
  switch (Shelf)
  {
  case 'A':
    Target_Point = QuWuDian[0];
    break;
  case 'B':
    Target_Point = QuWuDian[1];
    break;
  case 'C':
    Target_Point = QuWuDian[2];
    break;
  case 'D':
    Target_Point = QuWuDian[3];
    break;
  }
  BuyCar = Shelf;
  Movement_block(Now_Point.x, Now_Point.y, Target_Point.x, Target_Point.y);
}





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
      for(int i = 0;i<5;i++){
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
        else
          return false;
     }
        if(count!=0)
          avg_dist = avg_dist/count;
         
//        Serial.println(avg_dist);
//        Serial.println(count);
        if(avg_dist < distance)   return true;
        else return false;
}



void Scan_Echo(void){
  Scan_State[0] = Exam_items(H_trigPin, H_echoPin, 150);
  Scan_State[1] = Exam_items(L_trigPin, L_echoPin, 150);
  Scan_State[2] = Exam_items(Left_trigPin, Left_echoPin, 150);
}


void Init_Scan_Shelf(void){
// 扫 A 货架, 高的超声波为0, 低的超声波为1
Movement_block(8, 1, 8,  2);
  Turn_STE(142,STE_turn); 
  Loose();
  delay(200);
Movement_block(8, 2, 9,  2);
Movement_block(9, 2, 9, 1);
Movement_block(9, 1, 10, 1);
Turn_Left_Not_Back();
Stop();
delay(50);
Scan_Echo();
IsFullA[0][0] = Scan_State[0];
IsFullA[1][0] = Scan_State[1];

for(int i = 1; i <= 5; i++){
  Movement_block(10, i, 10, i+1);
  delay(50); 
  Scan_Echo();
  IsFullA[0][i] = Scan_State[0];
  IsFullA[1][i] = Scan_State[1];
}

// 扫 B 货架
Movement_block(10, 6, 10, 10);
Turn_Left_Not_Back();
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
  if (Scan_State[2] == true && (Now_Point.x == 9 && Now_Point.y == 10)){
    Map[9][9] = 0;
    IsFullB[0][1] = IsFullB[1][1] = true;     // 1号障碍物
  }
  if (Scan_State[2] == true && (Now_Point.x == 6 && Now_Point.y == 10)){
    Map[6][9] = 0;
    IsFullB[0][4] = IsFullB[1][4] = true;     // 2号障碍物
  }
}

// 扫 C 货架
Movement_block(5, 10, 1, 10);
Turn_Left_Not_Back();
Stop();
delay(50);
Scan_Echo();
IsFullC[0][0] = Scan_State[0];
IsFullC[1][0] = Scan_State[1];
for(int i = 10; i >= 6; i--){
  Movement_block(1, i, 1, i - 1);
  delay(50); 
  Scan_Echo();
  IsFullC[0][11 - i] = Scan_State[0];
  IsFullC[1][11 - i] = Scan_State[1];
  if (Scan_State[2] == true && (Now_Point.x == 1 && Now_Point.y == 9)){
    Map[2][9] = 0;
    IsFullC[0][1] = IsFullC[1][1] = true;  // 第3号障碍物
  }
  if (Scan_State[2] == true && (Now_Point.x == 1 && Now_Point.y == 6)){
    Map[2][6] = 0;
    IsFullC[0][4] = IsFullC[1][4] = true;
  }
}


Movement_block(1, 5, 1, 1);
Turn_Left_Not_Back();
Stop();
delay(50);
Scan_Echo();
IsFullD[0][0] = Scan_State[0];
IsFullD[1][0] = Scan_State[1];
for(int i = 1; i <= 5; i++){
  Movement_block(i, 1, i + 1, 1);
  delay(50); 
  Scan_Echo();
  IsFullD[0][i] = Scan_State[0];
  IsFullD[1][i] = Scan_State[1];
  if (Scan_State[2] == true && (Now_Point.x == 2 && Now_Point.y == 1)){
    Map[2][2] = 0;
    IsFullD[0][1] = IsFullD[1][1] = true;   // 5号障碍物
  }
  if (Scan_State[2] == true && (Now_Point.x == 5 && Now_Point.y == 1)){
    Map[5][2] = 0;
    IsFullD[0][4] = IsFullD[1][4] = true;   // 6号障碍物
  }
}
    STE_turn.write(82);
    delay(200);
Movement_block(6, 1, 5, 3);   // 直接到D区拿货
BuyCar = 'D';
}

void MAIN_LOOP(){
    while(1){
      // 没有拿够12件货物之前一直做这个循环
      // 抓货程序
      Catch_Move();
      if (Fangbuxia == 1){
         Tui1Ge();        // 退一格
         delay(500);
         Turn_STE(82,STE_turn);
         delay(300);
         Catch_Move();        // 尝试重新抓一次
      }
      while (BuyCarEmpty == 1){
        // Serial.println("购物车已经空, 转到下一个购物车去抓");
        Movement_block(Now_Point.x, Now_Point.y, Target_Point.x, Target_Point.y);
        Catch_Move();
      }
      // 放到货架中
      Move_To_Shelf(Shelf);
      // 返回离货架最近的取货点
      Back_To_QWD();
    }
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
//   
    return;
  }
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
//     Serial.println("Cal_Direction,调整方向异常");
  }
//  Serial.print("Cal_Direction: M = ");
//  Serial.print(M);
//  Serial.print("  move_state = ");
//  Serial.print(move_state);
//  Serial.print("  Target = ");
//  Serial.print(Target);
//  Serial.print("  Direction = ");
//  Serial.println(Direction);
}




/* ---------------------------------------------------------------------- */
/* 障碍物检查程序，检查地图中的一条直线上是否存在障碍物 */
/* ---------------------------------------------------------------------- */


int i;

bool Check_obstacle_x(int x0,int y1,int y2){  // ????(x0, y1)??(x0, y2)??????????????????????
  // ????Map?��?x0?��???
  bool HaveObastacle = false;
  if (y1 > y2)
  {
    int t;
    t = y1;
    y1 = y2;
    y2 = t;
  }
  
  for(int i = y1; i <= y2; i++){
    if (Map[x0][i] == false) {
      HaveObastacle = true;
      break;
    }
  }
  return HaveObastacle;
}

bool Check_obstacle_y(int y0,int x1,int x2){  // ????(x1, y0)??(x2, y0)??????????????????????
  // ????Map?��?y0?��???
  bool HaveObastacle = false;
  if (x1 > x2)
  {
    int t;
    t = x1;
    x1 = x2;
    x2 = t;
  }
  for(int i = x1; i <= x2; i++){
    if (Map[i][y0] == false) {
      HaveObastacle = true;
      break;
    }
  }
  return HaveObastacle;
}

bool Path_Planning_H_V(int x1, int y1, int x2, int y2){
	 if (!Check_obstacle_x(x1, y1, y2) &&!Check_obstacle_y(y2, x1, x2)) {  // ��Y��x 
    //??????, ??????????????
    tem_Point.x = x1;
     if(y2 > y1){
    // //   Adjust_Direction(0);
    // //   //??????
    // //   //?????????? y == y2  ??while???
    for( i = y1; i <= y2; i++){
            tem_Point.y = i;
            Enquene(tem_Point);
        }
     }
      //?????????? y == y2  ??while???    
     else if (y2 <= y1){
    // //   Adjust_Direction(2);
    // //   //??????
    // //   //?????????? y == y2  ??while??? 
    for( i = y1; i >= y2; i--){
            tem_Point.y = i;
            Enquene(tem_Point);
        }
     }
	//   Serial.println("?????????");
	    //??????, ??????????????
	    tem_Point.y = y2;
	    if(x2 > x1){
	      //??????
	      //???y == y2
	    //   Adjust_Direction(3);
	        for(int i = x1 + 1; i <= x2; i++){
	            tem_Point.x = i;
	            Enquene(tem_Point);
	        }
	    }
	    else if (x2 <= x1){
	      //??????
	    //   Adjust_Direction(1);
	        for(int i = x1 - 1; i >= x2; i--){
	            tem_Point.x = i;
	            Enquene(tem_Point);
	        }
	    }
	  
    }   

  else if(!Check_obstacle_y(y1, x1, x2) &&!Check_obstacle_x(x2, y1, y2)) {  // ��x��y 
//   Serial.println("?????????");
    //??????, ??????????????
    tem_Point.y = y1;
    if(x2 > x1){
      //??????
      //???y == y2
    //   Adjust_Direction(3);
        for(int i = x1; i <= x2; i++){
            tem_Point.x = i;
            Enquene(tem_Point);
        }
    }
    else if (x2 <= x1){
      //??????
    //   Adjust_Direction(1);
        for(int i = x1; i >= x2; i--){
            tem_Point.x = i;
            Enquene(tem_Point);
        }
    }
    //??????, ??????????????
    tem_Point.x = x2;
     if(y2 > y1){
    // //   Adjust_Direction(0);
    // //   //??????
    // //   //?????????? y == y2  ??while???
    for( i = y1 + 1; i <= y2; i++){
            tem_Point.y = i;
            Enquene(tem_Point);
        }
     }
      //?????????? y == y2  ??while???    
     else if (y2 <= y1){
    // //   Adjust_Direction(2);
    // //   //??????
    // //   //?????????? y == y2  ??while??? 
    for( i = y1 - 1; i >= y2; i--){
            tem_Point.y = i;
            Enquene(tem_Point);
        }
     }
  }
  else if(y1==2){
  	Path_Planning_H_V(x1,y1,x1,3);
  	Path_Planning_H_V(x1,3,x2,y2);
  }
  else if(y1==9){
  	Path_Planning_H_V(x1,y1,x1,8);
  	Path_Planning_H_V(x1,8,x2,y2);
  }
  else if(x1==2){
  	Path_Planning_H_V(x1,y1,3,y1);
  	Path_Planning_H_V(3,y1,x2,y2);
  }
  else return false;
  return true;
}

void Path_Planning(int x1, int y1, int x2, int y2){
  Q.rear = -1;
  Q.front = 0;
  int Path_Flag = 0; 
  Path_Flag = Path_Planning_H_V(x1,y1,x2,y2);
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
    pinMode(PLS, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(STEP_ENA, OUTPUT);

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
    attachInterrupt(3,ReadEncoder_L, CHANGE);
    attachInterrupt(2,ReadEncoder_R, CHANGE);
    FlexiTimer2::set(10, calculate_pid);   // 定时中断计算pid
    angle_ini();
    digitalWrite(STEP_ENA, HIGH);
}


void loop(){
    if(digitalRead(collide_1) == LOW) {
        Flag_Begin = 1;
    }
    if(Flag_Begin == true && move_state == 0){
          delay(3000);
          move_state = 1;
          Direction = 0;
          Init_Scan_Shelf();
          Movement_block(8,1,8,5);
          BuyCar = 'A';
          Loose();
          MAIN_LOOP();
    }
 }
