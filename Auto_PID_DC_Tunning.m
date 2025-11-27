% MATLAB Script for Automatic PID Tuning for Brushless DC Motor Speed Control
% This code is developed by Dr Nidal Kamel as a demonstration of Lecture 5

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

% Target speed in radians/sec (2000 rotations/sec)
target_speed = 2000 * 2 * pi / 60;  %#ok<*NASGU,*NASGU> % Convert to rad/s;
target_speed = 210; % Set to 210 rad/sec

% Time vector for simulation
t = 0:0.01:10;  % 10 seconds simulation

%% 1. Ziegler-Nichols Method for PID Tuning
disp('Ziegler-Nichols Tuning...');
Ku = 10;  % Ultimate gain (estimated)
Tu = 1.6;  % Ultimate period (estimated)

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

%% 2. MATLAB PID Tuner for PID Tuning
% The MATLAB function pidtune is based on an algorithm known as "Iterative Loop Shapingz", 
% which is a frequency-domain technique designed to achieve desired characteristics in the closed-loop system
disp('MATLAB PID Tuner...');
pid_tuner = pidtune(motor_tf, 'PD');
closed_loop_tuner = feedback(pid_tuner * motor_tf, 1);

% Step response for PID Tuner
[y_tuner, t_out] = step(target_speed * closed_loop_tuner, t);

% Calculate step response characteristics for PID Tuner
info_tuner = stepinfo(y_tuner, t_out, 'SettlingTimeThreshold', 0.02);
steady_state_error_tuner = abs(target_speed - y_tuner(end));

% Plotting the results
figure;
hold on;
set(gcf, 'WindowState', 'maximized'); % Set figure to full screen
plot(t_out, y_ZN, 'b', 'LineWidth', 1.5); % Ziegler-Nichols response
plot(t_out, y_tuner, 'g', 'LineWidth', 1.5); % PID Tuner response
yline(target_speed, '--r', 'LineWidth', 1.5); % Target speed line
ylim([0 350]); % Set y-axis limits from 0 to 350
title('Automatic PID Tuning for BLDC Motor Speed Control');
xlabel('Time (seconds)');
ylabel('Rotational Speed (rad/s)');
legend('Ziegler-Nichols', 'PID Tuner', 'Target Speed');
grid on;
hold off;

% Display step response characteristics in command window
disp(' ');
disp('Step Response Characteristics for Ziegler-Nichols Tuning:');
disp(table(info_ZN.RiseTime, info_ZN.SettlingTime, info_ZN.Overshoot, steady_state_error_ZN, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot', 'SteadyStateError'}));

disp(' ');
disp('Step Response Characteristics for PID Tuner:');
disp(table(info_tuner.RiseTime, info_tuner.SettlingTime, info_tuner.Overshoot, steady_state_error_tuner, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot', 'SteadyStateError'}));

% Convert Ziegler-Nichols table to text for display on plot
table_text_ZN = sprintf('Ziegler-Nichols:\nRise Time: %.2f s\nSettling Time: %.2f s\nOvershoot: %.2f %%\nSteady-State Error: %.2f rad/s', ...
    info_ZN.RiseTime, info_ZN.SettlingTime, info_ZN.Overshoot, steady_state_error_ZN);

% Convert PID Tuner table to text for display on plot
table_text_tuner = sprintf('PID Tuner:\nRise Time: %.2f s\nSettling Time: %.2f s\nOvershoot: %.2f %%\nSteady-State Error: %.2f rad/s', ...
    info_tuner.RiseTime, info_tuner.SettlingTime, info_tuner.Overshoot, steady_state_error_tuner);

% Display table-like text on the plot
annotation('textbox', [0.2, 0.3, 0.2, 0.2], 'String', table_text_ZN, 'FitBoxToText', 'on', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);

annotation('textbox', [0.6, 0.3, 0.2, 0.2], 'String', table_text_tuner, 'FitBoxToText', 'on', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);

disp(' ');
disp('Automatic PID Tuning Completed.');
disp(' ');
disp(['Ziegler-Nichols PID Gains: Kp = ' num2str(Kp_ZN) ', Ki = ' num2str(Ki_ZN) ', Kd = ' num2str(Kd_ZN)]);
disp(' ');
disp(['PID Tuner Gains: Kp = ' num2str(pid_tuner.Kp) ', Ki = ' num2str(pid_tuner.Ki) ', Kd = ' num2str(pid_tuner.Kd)]);

%% How to Use the Code:
% Run the script in MATLAB.
% Observe the response of the BLDC motor speed control with each tuning technique.
% The plot will show the comparison between the Ziegler-Nichols and PID Tuner methods.
