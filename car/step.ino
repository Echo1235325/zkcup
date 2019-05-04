
const int PLS = 5;
const int DIR = 6;
const int STEP_ENA = 23;
int Step_val = 0;
int Step_Flag = 0;
int collide_1 = 4; //碰撞开关

/**************************************************************************
函数功能：步进电机运行  作者：Ding
入口参数：前进方向   direct == HIGH向下，LOW向上
返回  值：无
**************************************************************************/
void Run_Step(bool direct){
  digitalWrite(DIR,direct);
  for(int i=0;i < 3200;i++){
    digitalWrite(PLS,HIGH);
    delayMicroseconds(20);
    digitalWrite(PLS,LOW);
    delayMicroseconds(20);
  }
}

/**************************************************************************
函数功能：步进电机控制 0表示取货高度，1第二层货架  作者：Ding
入口参数：前进方向
返回  值：无
**************************************************************************/
void Control_Step(int moveto){
  if(Step_Flag == 0){
     switch(moveto){
      case 0: break;
      case 1: 
            for(int i = 0;i<42;i++)  
                Run_Step(LOW);
              delay(300);
              Step_Flag = 1;
              break;
      }
  }
  else if(Step_Flag == 1){
      switch(moveto){
      case 0: 
              for(int i = 0;i<42;i++)  
                Run_Step(HIGH);
              delay(300);
              Step_Flag = 0;
              break;
      case 1: break;
      }
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PLS,OUTPUT);
  pinMode(DIR,OUTPUT);
  pinMode(STEP_ENA,OUTPUT);
  pinMode(collide_1,INPUT); 
  Serial.begin(9600);       // 测量结果将通过此串口输出至PC 上的串口监视器
  
}


void loop() {
    // put your main code here, to run repeatedly:
  if(digitalRead(collide_1) == LOW){
    digitalWrite(STEP_ENA,HIGH);
    // Control_Step(1); 
    for(int i = 0;i<42;i++)  
      Run_Step(HIGH);
  }
  digitalWrite(STEP_ENA,HIGH);

}
