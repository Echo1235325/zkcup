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
ExitFlag = '0xff'
BeginJudge = '0xfe'
Stop = False

class_name = {0: "ADmilk", 1: "Deluxe", 2: "RedBull",
               3: "Tennis", 4: "Yakult"}

if ser.is_open == False:
    print("Open Serial Failed!")
else:
    print("Serial Parameter:", ser)
    while Stop == False:
        if ser.in_waiting:
            ReciveData = ser.read(ser.in_waiting).decode("gbk")
            print("Recive: ", ReciveData)
            if ReciveData == ExitFlag:
                Stop = True
            if ReciveData == BeginJudge:
                result = pred.nnpred()
                ser.write(str(result).encode("utf-8"))
                time.sleep(0.5)

print("-----"*20+"Run Complete"+"-----"*20)
end = time.time()
print("Total run time: {}".format(end-since))

