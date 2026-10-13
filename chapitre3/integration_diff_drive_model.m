%% Comparison of Euler Discretization and Exact Integration for Differential Drive Robot

% Parameters
v = 1.0;               % Linear velocity (m/s)
omega = 0.5;           % Angular velocity (rad/s)
x0 = 0;                % Initial x position
y0 = 0;                % Initial y position
theta0 = 0;            % Initial orientation (rad)
num_steps = 150;        % Number of steps

% Case 1: Delta_t = 0.1 (small time step)
Delta_t1 = 0.1;
x_euler1 = zeros(1, num_steps+1);
y_euler1 = zeros(1, num_steps+1);
theta_euler1 = zeros(1, num_steps+1);
x_exact1 = zeros(1, num_steps+1);
y_exact1 = zeros(1, num_steps+1);
theta_exact1 = zeros(1, num_steps+1);
x_euler1(1) = x0;
y_euler1(1) = y0;
theta_euler1(1) = theta0;
x_exact1(1) = x0;
y_exact1(1) = y0;
theta_exact1(1) = theta0;

for k = 1:num_steps
    % Euler discretization
    x_euler1(k+1) = x_euler1(k) + v * cos(theta_euler1(k)) * Delta_t1;
    y_euler1(k+1) = y_euler1(k) + v * sin(theta_euler1(k)) * Delta_t1;
    theta_euler1(k+1) = theta_euler1(k) + omega * Delta_t1;
    % Exact integration
    theta_next = theta_exact1(k) + omega * Delta_t1;
    if omega ~= 0
        x_exact1(k+1) = x_exact1(k) + (v / omega) * (sin(theta_next) - sin(theta_exact1(k)));
        y_exact1(k+1) = y_exact1(k) - (v / omega) * (cos(theta_next) - cos(theta_exact1(k)));
    else
        x_exact1(k+1) = x_exact1(k) + v * cos(theta_exact1(k)) * Delta_t1;
        y_exact1(k+1) = y_exact1(k) + v * sin(theta_exact1(k)) * Delta_t1;
    end
    theta_exact1(k+1) = theta_next;
end

% Case 2: Delta_t = 0.5 (large time step)
Delta_t2 = 0.5;
x_euler2 = zeros(1, num_steps+1);
y_euler2 = zeros(1, num_steps+1);
theta_euler2 = zeros(1, num_steps+1);
x_exact2 = zeros(1, num_steps+1);
y_exact2 = zeros(1, num_steps+1);
theta_exact2 = zeros(1, num_steps+1);
x_euler2(1) = x0;
y_euler2(1) = y0;
theta_euler2(1) = theta0;
x_exact2(1) = x0;
y_exact2(1) = y0;
theta_exact2(1) = theta0;

for k = 1:num_steps
    % Euler discretization
    x_euler2(k+1) = x_euler2(k) + v * cos(theta_euler2(k)) * Delta_t2;
    y_euler2(k+1) = y_euler2(k) + v * sin(theta_euler2(k)) * Delta_t2;
    theta_euler2(k+1) = theta_euler2(k) + omega * Delta_t2;
    % Exact integration
    theta_next = theta_exact2(k) + omega * Delta_t2;
    if omega ~= 0
        x_exact2(k+1) = x_exact2(k) + (v / omega) * (sin(theta_next) - sin(theta_exact2(k)));
        y_exact2(k+1) = y_exact2(k) - (v / omega) * (cos(theta_next) - cos(theta_exact2(k)));
    else
        x_exact2(k+1) = x_exact2(k) + v * cos(theta_exact2(k)) * Delta_t2;
        y_exact2(k+1) = y_exact2(k) + v * sin(theta_exact2(k)) * Delta_t2;
    end
    theta_exact2(k+1) = theta_next;
end

% Plot results
figure;
plot(x_euler1, y_euler1, 'r--', 'LineWidth', 1.5); hold on;
plot(x_euler2, y_euler2, 'g--', 'LineWidth', 1.5);
plot(x_exact1, y_exact1, 'b-', 'LineWidth', 1.5);
legend('Euler Discretization (\Delta t = 0.1)', 'Euler Discretization (\Delta t = 0.5)', 'Exact Integration');
title('Comparison of Euler vs Exact Integration for Differential Drive Robot');
xlabel('X position (m)');
ylabel('Y position (m)');
grid on;
axis equal;
