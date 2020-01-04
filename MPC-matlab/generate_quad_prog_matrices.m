r = 10; % assume const for now

A = [1, 0, 1, 0; 
    0, 1, 0, 1;
    0, 0, 1, 0;
    0, 0, 0, 1];

B = [0;
    0;
    1;
    0];

C = [1 1 0 0];


prediction_horizon = 15; % Np
control_horizon = 3; % Nc

min_control = -5;
max_control = 5;

R1 = 1;
Rs = r * ones(prediction_horizon, 1);
Rw = R1 * eye(control_horizon);

%calculate fi
fi = zeros(prediction_horizon, control_horizon);
product_matrix = C * A^(prediction_horizon - control_horizon);
for i = prediction_horizon : -1 : 1
    for j = control_horizon : -1 : 1
        if i == prediction_horizon
            if j < control_horizon
                product_matrix = product_matrix * A;
            end
            fi(i, j) = product_matrix * B;
        else
            if i < j && j < control_horizon
                fi(i, j) = fi(i+1, j+1);
            end
        end
    end
end

%calculate F
F = zeros(prediction_horizon, length(C));
F(1, :) = C*A;
for i = 2 : prediction_horizon
    F(i, :) = F(i-1, :) * A;
end

x = 2 * zeros(length(C), 1);
H = fi' * fi + Rw;
step = 1/max(eig(H));
W = fi' * (F * x - Rs);
J = 0;
y(1) = C*x;
v = zeros(control_horizon, 1);
for j = 1:100
    W = fi' * (F * x - Rs);
    %v = quadprog(H, W, [], [], [], [], -15 * ones(3, 1), 15 * ones(3, 1));
    for i = 1:1000
        v = v - step * (H * v + W);
        v = max(min(v, max_control), min_control);
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