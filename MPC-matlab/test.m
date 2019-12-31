k = 4; T1 = 5; T2 = 10; T3 = 12;
intertia = k * tf(1, [T1 1]) * tf(1, [T2 1]) * tf(1, [T3 1]);
inertiaStateSpace = ss(intertia);
A = inertiaStateSpace.a;
B = inertiaStateSpace.b;
C = inertiaStateSpace.c;