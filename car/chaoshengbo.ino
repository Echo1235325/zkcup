/* ----------------------------------------
* 超声波测距模块US-100电平方式测距
* 选择电平方式需要去掉模块背面的跳线
* US-100的探头面向自己时，从左到右Pin脚依次为：
* VCC / Trig(Tx) / Echo(Rx) / GND / GND
* 两个GND只需要一个接地即可
* ----------------------------------------- */



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
