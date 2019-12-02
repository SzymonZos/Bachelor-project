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