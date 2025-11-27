% MATLAB Script for PID Tuning for Brushless DC Motor Speed Control
% with target speed of 2000 rotations/sec
% This code is developed by Dr Nidal Kamel as a demonstration of Lecture 5.

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

% Initial PID Gains
Kp = 1;  % Proportional gain
Ki = 0;  % Integral gain
Kd = 0;  % Derivative gain

% Target speed in radians/sec (2000 rotations/sec)
target_speed = 2000 * 2 * pi / 60;  %#ok<*NASGU> % Convert to rad/s
target_speed = 210; % Set to 210 rad/s

% Create the PID controller
pid_controller = pid(Kp, Ki, Kd);

% Closed-loop system
closed_loop_sys = feedback(pid_controller * motor_tf, 1);

% Time vector for simulation
t = 0:0.01:10;  % 10 seconds simulation

% Step response of the closed-loop system with target speed
[y, t_out] = step(target_speed * closed_loop_sys, t);

% Calculate step response characteristics
info = stepinfo(y, t_out, 'SettlingTimeThreshold', 0.02);

% Calculate steady-state error
steady_state_error = abs(target_speed - y(end));

% Plot the step response
figure;
set(gcf, 'WindowState', 'maximized'); % Set figure to full screen
plot(t_out, y, 'b', 'LineWidth', 1.5); % Plot the speed response
hold on;
yline(target_speed, '--r', 'LineWidth', 1.5); % Dashed red line at target speed
ylim([0 350]); % Set y-axis limits from 0 to 350
title(['Step Response for Target Speed of 210 rad/sec (2000 Rotations/sec) with Initial PID: Kp = ' num2str(Kp) ', Ki = ' num2str(Ki) ', Kd = ' num2str(Kd)]);
xlabel('Time (seconds)');
ylabel('Rotational Speed (rad/s)');
legend('Speed Response', 'Target Speed');
grid on;
shg

% Display PID values on the graph
text(0.5, 0.8, ['Kp = ' num2str(Kp)], 'FontSize', 12, 'Units', 'normalized');
text(0.5, 0.7, ['Ki = ' num2str(Ki)], 'FontSize', 12, 'Units', 'normalized');
text(0.5, 0.6, ['Kd = ' num2str(Kd)], 'FontSize', 12, 'Units', 'normalized');

% Create a table for step response characteristics
step_response_table = table(info.RiseTime, info.SettlingTime, info.Overshoot, steady_state_error, ...
    'VariableNames', {'RiseTime', 'SettlingTime', 'Overshoot', 'SteadyStateError'});

% Convert table to text for display on plot
table_text = sprintf('Rise Time: %.2f s\nSettling Time: %.2f s\nOvershoot: %.2f %%\nSteady-State Error: %.2f rad/s', ...
    info.RiseTime, info.SettlingTime, info.Overshoot, steady_state_error);

% Display table-like text on the plot
annotation('textbox', [0.6, 0.2, 0.2, 0.2], 'String', table_text, 'FitBoxToText', 'on', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);

% Display step response characteristics in command window
disp('Step Response Characteristics:');
disp(step_response_table);

% User input for PID tuning
disp('PID Tuning for BLDC Motor Speed Control (Target: 2000 rotations/sec)');
disp('Press Ctrl+C to stop the tuning process.');

while true
    % Prompt user to enter new PID gains
    Kp = input('Enter Proportional gain (Kp): ');
    Ki = input('Enter Integral gain (Ki): ');
    Kd = input('Enter Derivative gain (Kd): ');

    % Update the PID controller with new gains
    pid_controller = pid(Kp, Ki, Kd);

    % Update the closed-loop system
    closed_loop_sys = feedback(pid_controller * motor_tf, 1);

    % Step response of the updated closed-loop system
    [y, t_out] = step(target_speed * closed_loop_sys, t);

    % Calculate step response characteristics
    info = stepinfo(y, t_out, 'SettlingTimeThreshold', 0.02);
    steady_state_error = abs(target_speed - y(end));

    % Plot the step response
    close all
    figure;
    set(gcf, 'WindowState', 'maximized'); % Set figure to full screen
    plot(t_out, y, 'b', 'LineWidth', 1.5); % Plot the speed response
    hold on;
    yline(target_speed, '--r', 'LineWidth', 1.5); % Dashed red line at target speed
    ylim([0 350]); % Set y-axis limits from 0 to 350
    title('Step Response for Target Speed of 210 rad/sec (2000 Rotations/sec) ');
    xlabel('Time (seconds)');
    ylabel('Rotational Speed (rad/s)');
    legend('Speed Response', 'Target Speed');
    grid on;

    % Convert table to text for display on plot
    table_text = sprintf('Rise Time: %.2f s\nSettling Time: %.2f s\nOvershoot: %.2f %%\nSteady-State Error: %.2f rad/s', ...
        info.RiseTime, info.SettlingTime, info.Overshoot, steady_state_error);

    % Display table-like text on the plot
    annotation('textbox', [0.3, 0.2, 0.2, 0.2], 'String', table_text, 'FitBoxToText', 'on', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);

    % Display step response characteristics in command window
    % disp('Step Response Characteristics:');
    % disp(step_response_table);

    % Ask user if they want to continue tuning
    cont = input('Do you want to continue tuning? (y/n): ', 's');
    if cont == 'n'
        break;
    end
    close all
end

disp('PID Tuning completed.');