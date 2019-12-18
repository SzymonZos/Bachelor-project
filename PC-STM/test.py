import serial
import time
import numpy as np

A = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
B = [0, 0, 1, 0]
C = [1, 1, 0, 0]

dataToSend = 'A = ' + str(A) + '; B = ' + str(B) + '; C = ' + str(C) + ';'

A = np.array(A).reshape(-1, int(np.sqrt(len(A))))
B = np.array(B).reshape(len(B), -1)
C = np.array(C)
x = np.zeros((4, 1))
y = []
v = 0  # stab for now
x = np.dot(A, x) + v * B
y.append(np.dot(C, x).item(0))
print(y)
# y.append(C * x)

i = 0
try:
    with serial.Serial('COM3', 115200, timeout=1) as ser:
        ser.write(dataToSend)
        while i < 100:
            print(ser.read(10).decode('utf-8'))
            time.sleep(0.1)
            i += 1
except:
    print('Wild error appears')

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
