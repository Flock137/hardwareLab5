% Comparative Step Response Plot for PID Tuning Methods
% Manual, Ziegler-Nichols, and MATLAB PID Tuner
% This script generates a comparison plot for the lab report

% Parameters for the Brushless DC Motor
J = 0.01;  % Moment of inertia (kg*m^2)
b = 0.001; % Motor damping coefficient (N*m*s)
K = 0.01;  % Motor constant (N*m/A)
R = 1;     % Electric resistance (Ohms)
L = 0.5;   % Electric inductance (H)

% Transfer function of the BLDC motor (Plant)
num_motor = K;
den_motor = [L*J, (R*J + L*b), (R*b + K^2)];
motor_tf = tf(num_motor, den_motor);

% Target speed
target_speed = 210; % rad/s

% Time vector for simulation
t = 0:0.01:10;  % 10 seconds simulation

%% 1. Manual Tuning
disp('Manual Tuning PID Gains:');
Kp_manual = 2.5;
Ki_manual = 1;
Kd_manual = 0.5;

pid_manual = pid(Kp_manual, Ki_manual, Kd_manual);
closed_loop_manual = feedback(pid_manual * motor_tf, 1);

% Step response for Manual tuning
[y_manual, t_out] = step(target_speed * closed_loop_manual, t);

% Calculate step response characteristics for Manual tuning
info_manual = stepinfo(y_manual, t_out, 'SettlingTimeThreshold', 0.02);
steady_state_error_manual = abs(target_speed - y_manual(end));

disp(table(info_manual.RiseTime, info_manual.SettlingTime, info_manual.Overshoot, steady_state_error_manual, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot', 'SteadyStateError'}));

%% 2. Ziegler-Nichols Method
disp(' ');
disp('Ziegler-Nichols Tuning:');
Ku = 10;  % Ultimate gain
Tu = 1.6;  % Ultimate period

% Ziegler-Nichols tuning rules
Kp_ZN = 0.6 * Ku;
Ki_ZN = 2 * Kp_ZN / Tu;
Kd_ZN = Kp_ZN * Tu / 8;

pid_ZN = pid(Kp_ZN, Ki_ZN, Kd_ZN);
closed_loop_ZN = feedback(pid_ZN * motor_tf, 1);

% Step response for Ziegler-Nichols
[y_ZN, t_out] = step(target_speed * closed_loop_ZN, t);

% Calculate step response characteristics for Ziegler-Nichols
info_ZN = stepinfo(y_ZN, t_out, 'SettlingTimeThreshold', 0.02);
steady_state_error_ZN = abs(target_speed - y_ZN(end));

disp(table(info_ZN.RiseTime, info_ZN.SettlingTime, info_ZN.Overshoot, steady_state_error_ZN, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot', 'SteadyStateError'}));

%% 3. MATLAB PID Tuner
disp(' ');
disp('MATLAB PID Tuner:');
pid_tuner = pidtune(motor_tf, 'PD');
closed_loop_tuner = feedback(pid_tuner * motor_tf, 1);

% Step response for PID Tuner
[y_tuner, t_out] = step(target_speed * closed_loop_tuner, t);

% Calculate step response characteristics for PID Tuner
info_tuner = stepinfo(y_tuner, t_out, 'SettlingTimeThreshold', 0.02);
steady_state_error_tuner = abs(target_speed - y_tuner(end));

disp(table(info_tuner.RiseTime, info_tuner.SettlingTime, info_tuner.Overshoot, steady_state_error_tuner, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot', 'SteadyStateError'}));

%% Comparative Plot
figure;
set(gcf, 'WindowState', 'maximized'); % Set figure to full screen
hold on;

% Plot all three responses
plot(t_out, y_manual, 'b-', 'LineWidth', 2, 'DisplayName', 'Manual Tuning');
plot(t_out, y_ZN, 'g--', 'LineWidth', 2, 'DisplayName', 'Ziegler-Nichols');
plot(t_out, y_tuner, 'm-.', 'LineWidth', 2, 'DisplayName', 'MATLAB PID Tuner');
yline(target_speed, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target Speed');

% Set axis limits
xlim([0 10]);
ylim([0 350]);

% Labels and title
title('Comparative Step Response: Manual vs Ziegler-Nichols vs MATLAB PID Tuner', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Rotational Speed (rad/s)', 'FontSize', 12);
legend('Location', 'southeast', 'FontSize', 11);
grid on;

% Add text boxes with performance metrics for each method
% Manual Tuning metrics
text_manual = sprintf('Manual Tuning:\nKp=%.1f, Ki=%.1f, Kd=%.1f\nRise Time: %.2f s\nSettling Time: %.2f s\nOvershoot: %.2f%%\nSS Error: %.2f', ...
    Kp_manual, Ki_manual, Kd_manual, info_manual.RiseTime, info_manual.SettlingTime, info_manual.Overshoot, steady_state_error_manual);

% Ziegler-Nichols metrics
text_ZN = sprintf('Ziegler-Nichols:\nKp=%.1f, Ki=%.2f, Kd=%.2f\nRise Time: %.2f s\nSettling Time: %.2f s\nOvershoot: %.2f%%\nSS Error: %.4f', ...
    Kp_ZN, Ki_ZN, Kd_ZN, info_ZN.RiseTime, info_ZN.SettlingTime, info_ZN.Overshoot, steady_state_error_ZN);

% PID Tuner metrics
text_tuner = sprintf('MATLAB PID Tuner:\nKp=%.2f, Ki=%.2f, Kd=%.2f\nRise Time: %.2f s\nSettling Time: %.2f s\nOvershoot: %.2f%%\nSS Error: %.2f', ...
    pid_tuner.Kp, pid_tuner.Ki, pid_tuner.Kd, info_tuner.RiseTime, info_tuner.SettlingTime, info_tuner.Overshoot, steady_state_error_tuner);

% Display text boxes on the plot
annotation('textbox', [0.15, 0.65, 0.2, 0.25], 'String', text_manual, 'FitBoxToText', 'on', ...
    'BackgroundColor', [0.9 0.9 1], 'EdgeColor', 'blue', 'FontSize', 9, 'LineWidth', 1.5);

annotation('textbox', [0.15, 0.35, 0.2, 0.25], 'String', text_ZN, 'FitBoxToText', 'on', ...
    'BackgroundColor', [0.9 1 0.9], 'EdgeColor', 'green', 'FontSize', 9, 'LineWidth', 1.5);

annotation('textbox', [0.15, 0.05, 0.2, 0.25], 'String', text_tuner, 'FitBoxToText', 'on', ...
    'BackgroundColor', [1 0.9 1], 'EdgeColor', 'magenta', 'FontSize', 9, 'LineWidth', 1.5);

hold off;

%% Print Summary Table
disp(' ');
disp('====================================================================');
disp('COMPARATIVE SUMMARY OF ALL THREE TUNING METHODS');
disp('====================================================================');

% Create comparison table
Method = {'Manual'; 'Ziegler-Nichols'; 'MATLAB PID Tuner'};
Kp = [Kp_manual; Kp_ZN; pid_tuner.Kp];
Ki = [Ki_manual; Ki_ZN; pid_tuner.Ki];
Kd = [Kd_manual; Kd_ZN; pid_tuner.Kd];
RiseTime = [info_manual.RiseTime; info_ZN.RiseTime; info_tuner.RiseTime];
SettlingTime = [info_manual.SettlingTime; info_ZN.SettlingTime; info_tuner.SettlingTime];
Overshoot = [info_manual.Overshoot; info_ZN.Overshoot; info_tuner.Overshoot];
SteadyStateError = [steady_state_error_manual; steady_state_error_ZN; steady_state_error_tuner];

comparison_table = table(Method, Kp, Ki, Kd, RiseTime, SettlingTime, Overshoot, SteadyStateError);
disp(comparison_table);

disp(' ');
disp('Comparative PID Tuning Analysis Completed.');