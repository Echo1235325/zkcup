
/****************************************************
 * 函数功能：使舵机旋转
 * 参数：angle旋转角度; STE舵机代号
 * 返回值: void 
 ****************************************************/
int Turn_STE(int angle,Servo STE) {
  if(angle >= 0)
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
 * 函数功能：抓
 * 参数：
 * 返回值: void 
 ****************************************************/
void Catch(){
    Turn_STE(180,STE_cat);
    delay(100);
}
void Loose(){
    Turn_STE(40,STE_cat);
    delay(300);
}
/****************************************************
 * 函数功能：爪子与云台位置初始化
 * 参数：
 * 返回值: void 
 ****************************************************/
void angle_ini(){
  STE_cat.write(0);
  delay(200);
  STE_turn.write(82);
}
