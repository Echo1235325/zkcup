#include <Servo.h> //舵机操作

Servo STE_turn; //云台舵机
Servo STE_cat;  //爪子舵机
int Flag_STEturn = 0;
int pos = 0;
int collide_1 = 4; //碰撞开关
int collide_2 = 2; 
int angle = 0;


/****************************************************
 * 函数功能：使舵机旋转
 * 参数：angle旋转角度; STE舵机代号
 * 返回值: void 
 ****************************************************/
void Turn_STE(int angle,Servo STE) {
  if(angle > 0)
    for (pos = 0; pos <= angle; pos += 1) { // goes from 0 degrees to 180 degrees
                        // in steps of 1 degree
      STE.write(pos);              // tell servo to go to position in variable 'pos'
      // waits 15ms for the servo to reach the position
    }
  else{
    int i = pos;
    for(; pos >= i+angle; pos -= 1){
      STE.write(pos);              // tell servo to go to position in variable 'pos'
    }
  }
}

/****************************************************
 * 函数功能：云台控制
 * 参数：
 * 返回值: void 
 ****************************************************/
void Control_STE(){
  if(Flag_STEturn){ 
      Turn_STE(180,STE_turn);
      delay(1000);
      Turn_STE(120,STE_turn);
      delay(1000);
  }
}


/****************************************************
 * 函数功能：抓
 * 参数：
 * 返回值: void 
 ****************************************************/
void Catch(){
    Turn_STE(180,STE_cat);
    delay(200);
}
void Loose(){
    Turn_STE(-180,STE_cat);
    delay(200);
}
/****************************************************
 * 函数功能：爪子与云台位置初始化
 * 参数：
 * 返回值: void 
 ****************************************************/
void angle_ini(){
  Turn_STE(-180,STE_cat);
  delay(200);
  STE_turn.write(150);  
}
/**************************************************************************
函数功能：舵机停止  作者：Ding
入口参数：无
返回  值：无
**************************************************************************/
void Stop(){
    Flag_STEturn = 0;
    angle = 0;
}

void setup() {
  // put your setup code here, to run once:
  STE_turn.attach(9);  // attaches the servo on pin 9 to the servo object
  STE_cat.attach(3);
  pinMode(collide_1,INPUT); 
  attachInterrupt(0,Stop,LOW); //外部中断，停止
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Flag_STEturn == 0)
    angle_ini();
  if(digitalRead(collide_1) == LOW){
    Flag_STEturn = 1;
    Catch();
    Serial.print("1");
    Control_STE();
  }
}
