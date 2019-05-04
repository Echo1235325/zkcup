#include <FlexiTimer2.h>

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
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,1,1,1,1,1,1,1,1,1,1,0},
                    {0,0,0,0,0,0,0,0,0,0,0,0}};




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

int TrigPin = 8;
int EchoPin = 9;
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
float cm;

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


// 识别部分标志位
bool Flag_Recognize_C = 0;

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
        V_NOW_L = -150;
        V_NOW_R = -150;
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
 delay(210);
 Control(4);
 delay(400);
 Read_RedValue();
 while(!( Value_Red_ForWard[0] && !Value_Red_ForWard[1] && 
            !Value_Red_ForWard[2] && Value_Red_ForWard[3] )){
            Read_RedValue();
      }
  Direction = (Direction + 1) % 4;
  Start_PID();
//  Stop1();
}

void Turn_Right(){
 Stop_PID();
 Control(2);
 delay(210);
 Control(3);
 delay(400);
 Read_RedValue();
 while(!( Value_Red_ForWard[0] && !Value_Red_ForWard[1] && 
            !Value_Red_ForWard[2] && Value_Red_ForWard[3] )){
            Read_RedValue();
      }
  Direction = (Direction - 1) % 4;
  if (Direction < 0) {
    Direction += 4;
  }
//    Stop1();
  Start_PID();
}

void Turn_Around(){
  Turn_Left();
  Stop_PID();
  // 中间需要一些调整代码
  Start_PID();
  Turn_Left();
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
    pinMode(TrigPin,OUTPUT);
    pinMode(EchoPin,INPUT);
    //初始化
    digitalWrite(IN1, LOW);          //TB6612控制引脚拉低
    digitalWrite(IN2, LOW);          //TB6612控制引脚拉低
    digitalWrite(IN3, LOW);          //TB6612控制引脚拉低
    digitalWrite(IN4, LOW);          //TB6612控制引脚拉低
    analogWrite(PWMA, LOW);          //TB6612控制引脚拉低
    analogWrite(PWMB, LOW);          //TB6612控制引脚拉低


    Serial.begin(9600);
    attachInterrupt(0,Stop1,LOW); //外部中断，停止
    attachInterrupt(3,ReadEncoder_L, CHANGE);
    attachInterrupt(2,ReadEncoder_R, CHANGE);
    FlexiTimer2::set(10, calculate_pid);   // 定时中断计算pid

    move_state = 0;
    M = 0;
    Now_Point.x = 8;
    Now_Point.y = 0;
    Path_Planning(8, 1, 6, 3);
    Next_Point = Dequene();
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
              else{
                Serial.print("Quene Is Empty!");
                
              }
            }
       }
}


void Movement_block(void) {
  // 检查是否到点
    Read_RedValue();
    Exam_arrval_Point();
//  // 先计算下个状态的direction
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
      Stop();
      break;
    default:
      Serial.println("Movement_block, case异常情况");
      break;
  }
}


void Servo_Stepper_block(void){
}


void loop(){
    if(digitalRead(collide_1) == LOW) {
        Flag_Begin = 1;
    }
    if(Flag_Begin == true && move_state == 0){
        M = 1;
        move_state = 1;
        Start_PID();
        Control(move_state);
    }
    if (Flag_Begin == 1) {
      switch (M){
        case 1:
          Movement_block();
          break;
        case 2:
          Servo_Stepper_block();
          break;
        case 3:
//          Classfy_block();
        default:
          break;
      }
  }
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
  if (Now_Point.x == Next_Point.x  && Now_Point.y == Next_Point.y) {
    move_state = 5;   // 转停车
    M = 1;
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
//  Serial.print("Cal_Direction: M = ");
//  Serial.print(M);
//  Serial.print("  move_state = ");
//  Serial.print(move_state);
//  Serial.print("  Target = ");
//  Serial.print(Target);
//  Serial.print("  Direction = ");
//  Serial.println(Direction);
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

//
//void Classfy_block(void){
// // 可能需要做一些检查
// int time1;
// int good;    // 物品代号
// int Flag_Recognize = false;
// time1 = millis();
// Serial.println("C");        // 给nnpred程序发送开始识别指令C
// while(1){  // 等待nnpred计算完成返回计算结果
//   if (Serial.available()) {
//     good = Serial.read();
//     Flag_Recognize = true;
//     break;
//   }
//   if (millis() - time1 >= 10000) {    // 10秒超时
//     break;
//   }
// }
//
// if (Flag_Recognize) {
//   switch (good){
//     case '0':
//       break;
//     case '1':
//       break;
//     case '2':
//       break;
//     case '3':
//       break;
//     case '4':
//       break;
//     case '5':
//       break;
//     case '6':
//       break;
//     case '7':
//       break;
//     case '8':
//       break;
//     case '9':
//       break;
//     case '10':
//       break;
//     case '11':
//       break;
//     default:
//       break;
//   }
// }
// else{
//   //转串口通信超时处理
//   Serial.print("串口通信超时");
// }
//}
