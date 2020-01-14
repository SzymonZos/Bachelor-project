import os
import serial
import numpy as np
import matplotlib.pyplot as plt
import time
import statistics


projectPath = os.path.dirname(os.path.abspath(__file__))
changeParameters = True
systemParameters = {'A': [-1, 0.2, 1, -0.2, 0, 0.5, 0.1, 1, 0, 0, 1, 0, 0, 0, 0, 1],
                    'B': [0, 0, 1, 1],
                    'C': [1, 1, 0, 0],
                    'setPoint': [100],
                    'controlExtremeValues': [-0.25, 0.25],
                    'horizons': [15, 3]}
systemParametersToSend = str(systemParameters)[1:-1] + '\n\0'
A = np.array(systemParameters['A']).\
    reshape(-1, int(np.sqrt(len(systemParameters['A']))))
B = np.array(systemParameters['B']).\
    reshape(len(systemParameters['B']), -1)
C = np.array(systemParameters['C'])
x = np.zeros((len(systemParameters['B']), 1))
y = []
u = []
timer = []

with serial.Serial('COM3', 115200, timeout=20) as ser:
    if changeParameters:
        ser.write(bytes([len(systemParametersToSend)]))
        ser.write(systemParametersToSend.encode())
        startTimer = time.time()
        print(ser.readline().decode('utf-8'))
        endTimer = time.time()
        initTimer = endTimer - startTimer
    for i in range(200):
        xToSend = np.array2string(
            x.flatten(), formatter={'float_kind': lambda number: "%.4f" % number})
        xToSend = xToSend[1:-1] + '\0'
        ser.write(bytes([len(xToSend)]))
        ser.write(xToSend.encode())
        startTimer = time.time()
        v = float(ser.readline().decode('utf-8'))
        endTimer = time.time()
        x = np.dot(A, x) + v * B
        y.append(np.dot(C, x).item(0))
        timer.append(endTimer - startTimer)
        u.append(v)

_, fig = plt.subplots(2, 1, figsize=(6.4, 9.6))
fig[0].set_title('Wartość wyjścia obiektu w zależności od chwili czasu')
fig[0].plot(y, 'ro'), fig[0].set_ylabel('y'), fig[0].set_xlabel('i')
fig[1].set_title('Wartość sterowania w zależności od chwili czasu')
fig[1].plot(u, 'bo'), fig[1].set_ylabel('u'), fig[1].set_xlabel('i')
plt.show()
#plt.savefig(r'{0}\plots\A_{1}.png'.format
#            (projectPath, systemParameters['A']), format='png', bbox_inches='tight')
print(y[-1])

with open('log.txt', 'a') as logger:
    logger.write(str(systemParameters['A']) + '\n')
    logger.write('Init time, Mean of timestamps,'
                 ' Stdev of timestamps, Max of timestamps\n')
    logger.write(f"{initTimer:.4} {statistics.mean(timer):.4}"
                 f"{statistics.stdev(timer):.4} {max(timer):.4}\n")
