import serial
import time
import numpy as np
import matplotlib.pyplot as plt

systemParameters = {'A': [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1],
                    'B': [0, 0, 1, 0],
                    'C': [1, 1, 0, 0],
                    'setPoint': 4,
                    }
A = [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1]
B = [0, 0, 1, 0]
C = [1, 1, 0, 0]
setPoint = 4
dataToSend = 'A = ' + str(A) + '; B = ' + str(B) + '; C = ' + str(C) + ';\n\0'  # TODO: send this to STM

A = np.array(A).reshape(-1, int(np.sqrt(len(A))))
B = np.array(B).reshape(len(B), -1)
C = np.array(C)
x = np.zeros((4, 1))
y = []
with serial.Serial('COM3', 115200, timeout=1) as ser:
    # ser.write(chr(len(dataToSend)).encode()) TODO: handle this at STM side
    # ser.write(dataToSend.encode())
    for i in range(101):
        xToSend = np.array2string(x.flatten(), formatter={'float_kind': lambda number: "%.4f" % number})
        xToSend = xToSend[1:-1] + '\0'
        ser.write(chr(len(xToSend)).encode())
        ser.write(xToSend.encode())
        v = float(ser.readline().decode('utf-8'))
        x = np.dot(A, x) + v * B
        y.append(np.dot(C, x).item(0))
        time.sleep(0.05)
plt.plot(y, 'ro'), plt.ylabel('y'), plt.xlabel('i')
plt.show()
