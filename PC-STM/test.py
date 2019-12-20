import serial
import time
import numpy as np

A = [1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1]
B = [0, 0, 1, 0]
C = [1, 1, 0, 0]

dataToSend = 'A = ' + str(A) + '; B = ' + str(B) + '; C = ' + str(C) + ';\n\0'
print(dataToSend, len(dataToSend))
A = np.array(A).reshape(-1, int(np.sqrt(len(A))))
B = np.array(B).reshape(len(B), -1)
C = np.array(C)
x = np.zeros((4, 1))
y = []
v = 0  # stab for now
# y.append(C * x)
i = 0
# try:
with serial.Serial('COM3', 115200, timeout=1) as ser:
    # ser.write(chr(len(dataToSend)).encode())
    # ser.write(dataToSend.encode())
    while i < 100:
        xToSend = np.array2string(x.flatten(), formatter={'float_kind': lambda d: "%.4f" % d})
        xToSend = xToSend[1:-1] + '\0'
        ser.write(chr(len(xToSend)).encode())
        ser.write(xToSend.encode())
        v = float(ser.readline().decode('utf-8'))
        x = np.dot(A, x) + v * B
        print(x, v)
        y.append(np.dot(C, x).item(0))

        time.sleep(0.05)
        i += 1
print(y)
# except:
#     print('Wild error appears')

# Jak widzę działanie tego skryptu:
#
# STM
# STM chodzi sobie w pętli: jeden task sprawdza cyklicznie czy coś zostało wysłane po uarcie
# Jeśli zostało to przechodzimy do taska liczącego sterowanie, który na samym końcu wysyła po uarcie wynik
# Jeśli nic nie zostało wykryte to dalej czekawmy !!!Ważne: upewnić się, że nie czytamy wyniku STMa
#
# Python
# Inicjalizacja: podanie macierzy, ustabilizowaniee połączenia, chyba tyle na razie
# Pętla: czytanie kanału, walidacja czy jest to rzeczywiście sygnał od STMa
# Wystawienie zmodyfikowanej wartości po uarcie; czekanie na kolejną
