
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
    delay(2000);
}
void Loose(){
    Turn_STE(-180,STE_cat);
    delay(2000);
}
/****************************************************
 * 函数功能：爪子与云台位置初始化
 * 参数：
 * 返回值: void 
 ****************************************************/
void angle_ini(){
  Turn_STE(-180,STE_cat);
  delay(200);
  Turn_STE(90,STE_turn); 
}
