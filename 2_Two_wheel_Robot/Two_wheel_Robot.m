clc;   
clear;
close all;

%%====================================== Main ======================================%% 
% 초기값
params = Params_init();
r_w = params.r_w;

% 시간
dt = 0.001;                                      % 시간차 
T = 0:dt:6;                                      % 총 시간

% 초기 조건
x0 = 0;                                          % 카트 위치 (m)
x_dot0 = 0;                                      % 카트 속도 (m/s)
theta0 = deg2rad(10);                            % 진자 각도 (rad)
theta_dot0 = 0;                                  % 진자 각속도 (rad/s)

X_init = [x0; x_dot0; theta0; theta_dot0];       % State vector

a_PD = X_init;
a_PID = X_init;

% 제어값
x_d = 0;                                         % 제어 카트 위치
x_dot_d = 0;                                     % 제어 카트 속도
theta_d = deg2rad(0);                            % 제어 진자 각도
theta_dot_d = 0;                                 % 제어 진자 각속도
 

X_target = [x_d, x_dot_d, theta_d, theta_dot_d];

% Preallocation
state_history_U_PD = zeros(length(T), 4); 
state_history_U_PID = zeros(length(T), 4);

target = zeros(length(T), 2);                    % 목표 제어값 표시

% 초기값
state_history_U_PD(1 , :) = X_init;
state_history_U_PID(1 , :) = X_init;

target(1 , :) = [X_target(1);X_target(3)];                    % 목표 제어값 표시

for i = 1 : length(T) - 1


    % %목표값
    target(i+ 1,1) = x_d;
    target(i+ 1,2) = theta_d;
    
    % 제어 입력
    U_theta_PD = theta_Controller_PD(a_PD, X_target, params); 

    U_pos = a_Controller_PID(a_PID, X_target, params);
    U_theta = theta_Controller_PID(a_PID, X_target, params);

    U = [U_pos; U_theta];


    % Rk4
    a_PD = Rk4(@inverted_pendulum_Robot_dynamics_PD, a_PD, U_theta_PD, dt, params);
    a_PID = Rk4(@inverted_pendulum_Robot_dynamics, a_PID, U, dt, params);

    % 변수 저장
    state_history_U_PD(i + 1, :) = a_PD'; 
    state_history_U_PID(i + 1, :) = a_PID';    
 
end

%%====================================== Plot ======================================%%
% Plot results for Linear PID control

Yp_pls = target(1) + 2;
Yp_neg = target(1) - 2;
Yt_pls = 25;
Yt_neg = -25;

figure(1);
subplot(2, 1, 1);
plot(T, state_history_U_PD(:, 1)); % Plotting Position
hold on
plot(T, target(:,1),"red--"); % Plotting target Position
grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot Position (PD Control)');
legend('Robot Position', 'Target Position'); ylim([Yp_neg Yp_pls]);
hold on

subplot(2, 1, 2);
plot(T, rad2deg(state_history_U_PD(:, 3))); % Plotting theta in degrees
hold on
plot(T, rad2deg(target(:,2)),"red--"); % Plotting target theta in degrees
grid on; xlabel('Time [s]'); ylabel('Robot Angle [deg]'); title('Robot body Angle (PD Control)');
legend('Robot body Angle','Target Angle'); ylim([Yt_neg Yt_pls]);
hold on

figure(2);
subplot(2, 1, 1);
plot(T, state_history_U_PID(:, 1)); % Plotting Position
hold on
plot(T, target(:,1),"red--"); % Plotting target Position
grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot Position (PID Control)');
legend('Robot Position', 'Target Position'); ylim([Yp_neg Yp_pls]);
hold on

subplot(2, 1, 2);
plot(T, rad2deg(state_history_U_PID(:, 3))); % Plotting theta in degrees
hold on
plot(T, rad2deg(target(:,2)),"red--"); % Plotting target theta in degrees
grid on; xlabel('Time [s]'); ylabel('Robot Angle [deg]'); title('Robot body Angle (PID Control)');
legend('Robot body Angle','Target Angle'); ylim([Yt_neg Yt_pls]);
hold on



%% Parameters
%%====================================== Parameters ======================================%%
function params = Params_init() 
    % % PID 게인값 
  
    params.a_Kp = 10;  
    params.a_Ki = 6.838;
    params.a_Kd = 2.5;

    params.Theta_Kp = 2;  
    params.Theta_Ki = 5.084;
    params.Theta_Kd = 0.071;

    % 초기값
    params.g = 9.81;                                       % 중력 (m/s^2)       
    params.M = 2;                                          % 로봇 질량 (kg)    
    params.m = 0.5;                                        % 바퀴 질량 (kg)
    params.L = 0.3;                                        % 로봇 길이 (m)
    params.r_w = 0.03;                                     % 막대 반지름
    params.r_b = 0.03;                                     % 바퀴 반지름    
    params.C_a = 0.00055;                                  % 외력

    params.dt = 1/1000;
end

%% Dynamics
%%====================================== Dynamics ======================================%%
function Robot_xdot = inverted_pendulum_Robot_dynamics_PD(X, U_theta_PD, params)

    % 초기값 Unpack 
    g = params.g;    
    M = params.M;
    m = params.m;
    L = params.L;
    r_w = params.r_w;
    r_b = params.r_b;
    I_b = (M * r_b ^2) / 2 ;
    I_w = (m * r_w ^2) / 2;
    C_a = params.C_a;


    U_theta = U_theta_PD;

    %%=============== 상태변수 ===============%%
    
    % Position
    x = X(1);
    alpha = x / r_w;                                     % 바퀴 이동 거리 (rad)

    % Position V                       
    x_dot = X(2);
    alpha_dot = x_dot / r_w;                             % 바퀴 이동 속도 (rad/s)

    % theta
    theta = X(3);

    % theta V
    theta_dot = X(4);


    % 선형 방정식 치환
    A = ((M + m) * r_w^2 + I_w);
    B = (I_b + M * L^2);
    denominator = A * B - (r_w * M * L)^2;
    numerator_a = r_w * g * theta * (M * L)^2 - B * C_a * alpha_dot + B * U_theta;
    numerator_theta = A * M * L * g * theta - r_w * M * L * C_a * alpha_dot + r_w * M * L * U_theta;
         
    % Position A
    a_2dot = numerator_a / denominator;
    x_2dot = a_2dot * r_w;
  
    % theta A
    theta_2dot = numerator_theta / denominator;    
 
    % State derivatives
    Robot_xdot = [x_dot; x_2dot; theta_dot; theta_2dot;];
end

function Robot_xdot = inverted_pendulum_Robot_dynamics(X, U, params)

    % 초기값 Unpack 
    g = params.g;    
    M = params.M;
    m = params.m;
    L = params.L;
    r_w = params.r_w;
    r_b = params.r_b;
    I_b = (M * r_b ^2) / 2 ;
    I_w = (m * r_w ^2) / 2;
    C_a = params.C_a;


    U_pos = U(1);
    U_theta = U(2);

    %%=============== 상태변수 ===============%%
    
    % Position
    x = X(1);
    alpha = x / r_w;                                     % 바퀴 이동 거리 (rad)

    % Position V                       
    x_dot = X(2);
    alpha_dot = x_dot / r_w;                             % 바퀴 이동 속도 (rad/s)

    % theta
    theta = X(3);

    % theta V
    theta_dot = X(4);


    % 선형 방정식 치환
    A = ((M + m) * r_w^2 + I_w);
    B = (I_b + M * L^2);
    denominator = A * B - (r_w * M * L)^2;
    numerator_a = r_w * g * theta * (M * L)^2 - B * C_a * alpha_dot + B * U_theta;
    numerator_theta = A * M * L * g * theta - r_w * M * L * C_a * alpha_dot + r_w * M * L * U_theta;
         
    % Position A
    a_2dot = numerator_a / denominator;
    x_2dot = a_2dot * r_w;
  
    % theta A
    theta_2dot = numerator_theta / denominator;    
 
    % State derivatives
    Robot_xdot = [x_dot; x_2dot; theta_dot; theta_2dot;];
end

%% PD Integration Function
%%====================================== PD Controller Function ======================================%%
function U_theta_PD = theta_Controller_PD(X, X_target, params)
     persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end

    % Robot PID 게인값 
    Kp = params.Theta_Kp; 
    Ki = 0; 
    Kd = params.Theta_Kd;  

    % PD 제어    
    theta0 = X(3);
    e_theta = X_target(3) - theta0;
    
    % PID control law
    integral_theta = integral_theta + e_theta * params.dt;
    derivative_theta = (e_theta - error_previous) / params.dt;
    U_theta_PD = (Kp * e_theta + Ki * integral_theta + Kd * derivative_theta);

    % Update previous error
    error_previous = e_theta;        
end

%% PID Controller
%%====================================== PID Controller Function ======================================%%
function U_pos = a_Controller_PID(X, X_target, params)
    persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end

    % Robot PID 게인값 
    Kp = params.a_Kp; 
    Ki = params.a_Ki; 
    Kd = params.a_Kd;     
    
    % PD 제어    
    a0 = X(1);
    e_a = X_target(1) - a0;
    
    % PID control law
    integral_theta = integral_theta + e_a * params.dt;
    derivative_theta = (e_a - error_previous) / params.dt;
    U_pos = (Kp * e_a + Ki * integral_theta + Kd * derivative_theta);

    % Update previous error
    error_previous = e_a;    
end

function U_theta = theta_Controller_PID(X, X_target, params)
    persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end

    % Robot PID 게인값 
    Kp = params.Theta_Kp; 
    Ki = params.Theta_Ki; 
    Kd = params.Theta_Kd;  

    % PD 제어    
    theta0 = X(3);
    e_theta = X_target(3) - theta0;
    
    % PID control law
    integral_theta = integral_theta + e_theta * params.dt;
    derivative_theta = (e_theta - error_previous) / params.dt;
    U_theta = (Kp * e_theta + Ki * integral_theta + Kd * derivative_theta);

    % Update previous error
    error_previous = e_theta;    
end

%% RK4 Integration Function
%%====================================== Rk4 ======================================%%

function X = Rk4 (Func, X, u, dt, params)
a= 0.5;
b= 0.5;

k1 = Func(X, u, params) * dt;
k2 = Func(X + a*k1, u, params) * dt;
k3 = Func(X + + b*k2, u, params) * dt;
k4 = Func(X + + b*k3, u, params) * dt;
X = X + (k1  + 2*(k2 + k3) + k4) / 6;

end

