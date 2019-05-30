import serial
import numpy as np
import serial.tools.list_ports
import matplotlib.pyplot as plt
portlist = list(serial.tools.list_ports.comports())
if len(portlist) == 0:
    print("NONE")
else:
    for port in portlist:
        print(port)

ser = serial.Serial()
ser.baudrate = 9600
ser.port = str(portlist[-1])[:21]
ser.timeout = 10
ser.open()
count = 0
Recordcount = 200
datalist = []
Stop = True
StopFlag = 'S'
if  ser.is_open == False:
    print("Open Serial failed")
else:
    print("Serial parameters:", ser)
    while Stop == True:
        if ser.in_waiting != 0 :
            RecieveData = ser.readline().decode('gbk')
            if len(RecieveData) > 25:
                RecieveData = RecieveData.split(' ')
                RecieveData[-1] = RecieveData[-1][:-2]
                RecieveData = [float(data) for data in RecieveData]
                print(RecieveData)
                datalist.append(RecieveData)
                count += 1
            else:
                continue
            if count >= Recordcount:
                Stop = False

print("Stop Recieve Data!")



datalist = np.array(datalist)
print("datalist = ", datalist)

time = datalist[:, 0]
time = time - time[0]       #时间坐标从0开始
Pulse_L = datalist[:, 1]
Pulse_R = datalist[:, 2]
pwmL = datalist[:, 3]
pwmR = datalist[:, 4]
erorr_L = datalist[:, 5]
erorr_int_L = datalist[:, 6]
derorr_L = datalist[:, 7]
erorr_R = datalist[:, 8]
erorr_int_R = datalist[:, 9]
derorr_R = datalist[:, 10]

setpoint = [9, 9]
time2 = [0, time[-1]]
xticks = np.arange(0, count*10, 100)


figure = plt.figure()
plt.subplot(411)
l1, = plt.plot(time2, setpoint, color='g', linewidth=1.0, label='Setpoint')
l2, = plt.plot(time, Pulse_L, color='r',linewidth=1.0, label = 'Pulse_L')
l3, = plt.plot(time, Pulse_R, color='b',linewidth=1.0, label = 'Pulse_R')
plt.xlabel('t/ms')
linelist = [l1, l2, l3]
plt.legend(linelist, [x.get_label() for x in linelist])

plt.subplot(412)
l4, = plt.plot(time, erorr_L, 'g',linewidth = 1.0, label = 'erorr_L')
l5, = plt.plot(time, erorr_int_L,'r', linewidth = 1.0, label = 'erorr_int_L')
l6, = plt.plot(time, derorr_L,'b' ,linewidth = 1.0, label = 'derorr_L')
plt.legend()
plt.xlabel('t/ms')

plt.subplot(413)
l4, = plt.plot(time, erorr_R, 'g',linewidth = 1.0, label = 'erorr_R')
l5, = plt.plot(time, erorr_int_R,'r', linewidth = 1.0, label = 'erorr_int_R')
l6, = plt.plot(time, derorr_R,'b', linewidth = 1.0, label = 'derorr_R')
plt.legend()
plt.xlabel('t/ms')

plt.subplot(414)
plt.plot(time, pwmL, 'b', linewidth = 1.0, label = 'pwmL')
plt.plot(time, pwmR, 'r', linewidth = 1.0, label = 'pwmR')
plt.legend()
plt.xlabel('t/ms')


#设置主刻度标签的位置,标签文本的格式

plt.show()
