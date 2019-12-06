k = 4; T1 = 5; T2 = 10; T3 = 12;
intertia = k * tf(1, [T1 1]) * tf(1, [T2 1]) * tf(1, [T3 1]);
inertiaStateSpace = ss(intertia);
inertiaStateSpace.InputGroup.MV = 1;
inertiaStateSpace.OutputGroup.MO = 1;
inertiaStateSpace.InputName = {'u'};
inertiaStateSpace.OutputName = {'y'};
inertiaStateSpace.StateName = {'x1', 'x2', 'x3'};
step(inertiaStateSpace);
%MV = struct('Min',-1,'Max',1);
%MPCobj = mpc(inertiaStateSpace, 0.1, 20, 3, [], MV);
%MPCobj.sim()
%sim(MPCobj)

a = inertiaStateSpace.a;
b = inertiaStateSpace.b;
H = zeros(3,3);
H(1, 1) = 2 * (1 + b(1) * b(1) + a(1,:) * b);
H(1, 2) = 2 * a(1, :) * b * b(1);
H(2, 1) = 2 * a(1, :) * b * b(1);
H(2, 2) = 2 *(1 + b(1) * b(1));
H(3, 3) = 2;
F = zeros(3, 1);
w = 4;
x = ones(3, 1);
F(1) = 2 * (a(1, :) * x * b(1) + (a(1,:) * a * x) * (a(1,:) * b) - w * a(1,:) * b);
F(2) = 2 * (a(1, :) * a * x * b(1) - w * b(1));
