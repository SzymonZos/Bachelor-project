k = 4; T1 = 5; T2 = 10; T3 = 12;
inertia = k * tf(1, [T1 1]) * tf(1, [T2 1]) * tf(1, [T3 1]);
inertiaStateSpace = ss(inertia);
ssD = ss(inertiaStateSpace.a, inertiaStateSpace.b, inertiaStateSpace.c, inertiaStateSpace.d, 0.1);
%{
A = inertiaStateSpace.a;
B = inertiaStateSpace.b;
C = inertiaStateSpace.c;
%}

A = [1, 0, 1, 0; 
    0, 1, 0, 1;
    0, 0, 1, 0;
    0, 0, 0, 1];

B = [0;
    0;
    1;
    0];

C = [1 1 0 0];

w = 10;

fi = [C*B 0 0; C*A*B C*B 0; C*A^2*B C*A*B C*B];
F = [C*A; C*A^2; C*A^3];

x = zeros(4,1);
J = 0;
H = fi' * fi + eye(3);
W = fi' * (-w * ones(3,1) + F * x);
y(1) = C*x;
for j = 1:100
    W = fi' * (-w * ones(3,1) + F * x);
    %v = quadprog(H, W, [], [], [], [], -15 * ones(3, 1), 15 * ones(3, 1));
    for i = 1:100
        W = fi' * (-w * ones(3,1) + F * x);
        v = v - 0.1 * (H * v + W);
        J_prev = J;
        J = 0.5 * (v' * H * v) + v' * W;
        if abs(J - J_prev) < 0.01
            break
        end
    end
    x = A * x + v(1) * B;
    y(j+1) = C*x;
end

figure
plot(1:101, y);