k = 4; T1 = 5; T2 = 10; T3 = 12;
intertia = k * tf(1, [T1 1]) * tf(1, [T2 1]) * tf(1, [T3 1]);
inertiaStateSpace = ss(intertia);
inertiaStateSpace.InputGroup.MV = 1;
inertiaStateSpace.OutputGroup.MO = 1;
inertiaStateSpace.InputName = {'u'};
inertiaStateSpace.OutputName = {'y'};
inertiaStateSpace.StateName = {'x1', 'x2', 'x3'};
step(inertiaStateSpace);
MV = struct('Min',-15,'Max',15);
MPCobj = mpc(inertiaStateSpace, 0.1, 2, 2, [], MV);
sim(MPCobj)

a = inertiaStateSpace.a;
b = inertiaStateSpace.b;
H = zeros(2,2);
which = 1;
H(1, 1) = (1 + b(which) * b(which) + (a(which,:) * b) * (a(which,:) * b));
H(1, 2) = a(which, :) * b * b(which);
H(2, 1) = a(which, :) * b * b(which);
H(2, 2) = (1 + b(which) * b(which));

F = zeros(2, 1);
w = 1;
x = zeros(3, 1);
v = zeros(2, 1);
J = 0;

for j = 1:100
    %{
    for i = 1:100
        F(1) = b(1) * (-w + a(1, :) * x) + (a(1, :) * b) * (-w + a(1, :) * a * x);
        F(2) = b(1) * (-w + a(1, :) * a * x);
        v = v - 0.1 * (H * v + F);
        J_prev = J;
        J = 0.5 * (v' * H * v) + v' * F;
        if abs(J - J_prev) < 0.01
            break
        end
    end
    %}
    F(1) = b(which) * (-w + a(which, :) * x) + (a(which, :) * b) * (-w + a(which, :) * a * x);
    F(2) = b(which) * (-w + a(which, :) * a * x);
    v = quadprog(H, F);
    x = a * x + v(1) * b;
    dupa(j) = x(1);
end

figure
plot(1:100, dupa);
