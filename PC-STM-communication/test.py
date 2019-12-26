import serial
import numpy as np
import matplotlib.pyplot as plt

changeParameters = True
systemParameters = {'A': [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1],
                    'B': [0, 0, 1, 0],
                    'C': [1, 1, 0, 0],
                    'setPoint': [15],
                    'controlExtremeValues': [-5, 5]}
systemParametersToSend = str(systemParameters)[1:-1] + '\n\0'
A = np.array(systemParameters['A']).reshape(-1, int(np.sqrt(len(systemParameters['A']))))
B = np.array(systemParameters['B']).reshape(len(systemParameters['B']), -1)
C = np.array(systemParameters['C'])
x = np.zeros((4, 1))
y = []
print(systemParametersToSend)

with serial.Serial('COM3', 115200, timeout=20) as ser:
    if changeParameters:
        ser.write(bytes([len(systemParametersToSend)]))
        ser.write(systemParametersToSend.encode())
    for i in range(101):
        xToSend = np.array2string(x.flatten(), formatter={'float_kind': lambda number: "%.4f" % number})
        xToSend = xToSend[1:-1] + '\0'
        ser.write(bytes([len(xToSend)]))
        ser.write(xToSend.encode())
        v = float(ser.readline().decode('utf-8'))
        x = np.dot(A, x) + v * B
        y.append(np.dot(C, x).item(0))
plt.plot(y, 'ro'), plt.ylabel('y'), plt.xlabel('i')
plt.show()
