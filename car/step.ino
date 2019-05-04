

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
