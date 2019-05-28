import serial
import pred
import time

since = time.time()

# 打开串口
ser = serial.Serial()
ser.baudrate = 9600
ser.port = "COM1"
ser.open()

ReciveData = ''
WriteData = ''
ExitFlag = 'E'
BeginJudge = 'B'
Stop = False

class_name = {0: "ADmilk", 1: "Deluxe", 2: "RedBull",
               3: "Tennis", 4: "Yakult"}

if ser.is_open == False:
    print("Open Serial Failed!")
else:
    print("Serial Parameter:", ser)
    while Stop == False:
        if ser.in_waiting != 0:
            ReciveData = ser.read(1).decode("gbk")
            print("Recive:", ReciveData)
            if ReciveData == ExitFlag:
                Stop = True
            if ReciveData == BeginJudge:
                result = pred.nnpred()
                result = chr(result + 97)       # 数字0~11转成a~m
                ser.write(str(result).encode("gbk"))

print("-----"*20+"Run Complete"+"-----"*20)
end = time.time()
print("Total run time: {}".format(end-since))

