clc;   
clear;
close all;
%%====================================== Main ======================================%% 
% 초기값
params = Params_init();

% 시간
dt = 0.001;                                      % 시간차 
T = 0:dt:1;                                     % 총 시간

% 초기 조건
x0 = 0;                                          % 카트 위치 (m)
x_dot0 = 0;                                      % 카트 속도 (m/s)
theta0 = deg2rad(10);                            % 진자 각도 (rad)
theta_dot0 = 0;                                  % 진자 각속도 (rad/s)

X_init = [x0; x_dot0; theta0; theta_dot0];       % State vector
 
X0 = X_init; a0 = X_init;
X_PD = X_init; X_PID = X_init;
a_PD = X_init; a_PID = X_init;

% 제어값
x_d = 0;                                         % 제어 카트 위치
x_dot_d = 0;                                     % 제어 카트 속도
theta_d = deg2rad(0);                            % 제어 진자 각도
theta_dot_d = 0;                                 % 제어 진자 각속도
 
X_target = [x_d, x_dot_d, theta_d, theta_dot_d];

% Preallocation
state_history_u_x_0 = zeros(length(T), 4); state_history_u_a_0 = zeros(length(T), 4);

state_history_F_ext0_PD = zeros(length(T), 4); state_history_F_ext0_PID = zeros(length(T), 4);

state_history_u_PD = zeros(length(T), 4); state_history_u_PID = zeros(length(T), 4);

target = zeros(length(T), 4);

for i = 1 : length(T) - 1

    % 초기값
    state_history_u_x_0(1 , :) = [0,0,theta0,0];
    state_history_F_ext0_PD(1 , :) = [0,0,theta0,0];
    state_history_F_ext0_PID(1 , :) = [0,0,theta0,0];
    state_history_u_PD(1 , :) = [0,0,theta0,0];
    state_history_u_PID(1 , :) = [0,0,theta0,0];

    %목표값
    target(i) = theta_d;
    
    % 제어 입력
    u_non = 0;

    u_F_ext0_PD = Robot_Controller_F_ext0_PD(X_PD, X_target, params); 
    u_F_ext0_PID = Robot_Controller_F_ext0_PID(X_PID, X_target, params);
    
    u_PD = Robot_Controller_PD(a_PD, X_target, params);    
    u_PID = Robot_Controller_PID(a_PID, X_target, params);  

    % Rk4
    X0 = Rk4(@inverted_pendulum_Robot_dynamics_F_ext0, X0, u_non, dt, params);

    a0 = Rk4(@inverted_pendulum_Robot_dynamics, a0, u_non, dt, params);
    X_PD = Rk4(@inverted_pendulum_Robot_dynamics_F_ext0, X_PD, u_F_ext0_PD, dt, params);      
    X_PID = Rk4(@inverted_pendulum_Robot_dynamics_F_ext0, X_PID, u_F_ext0_PID, dt, params);   

    a_PD = Rk4(@inverted_pendulum_Robot_dynamics, a_PD, u_PD, dt, params);
    a_PID = Rk4(@inverted_pendulum_Robot_dynamics, a_PID, u_PID, dt, params);

    % 변수 저장
    state_history_u_x_0(i + 1, :) = X0'; state_history_u_a_0(i + 1, :) = a0';    
    state_history_F_ext0_PD(i + 1, :) = X_PD'; state_history_F_ext0_PID(i + 1, :) = X_PID';  
    state_history_u_PD(i + 1, :) = a_PD'; state_history_u_PID(i + 1, :) = a_PID';    
 
end

%%====================================== Plot ======================================%%
% Plot results for u = 0
figure(1);
subplot(2, 1, 1);
plot(T, 100*state_history_u_x_0(:, 1)); % Plotting theta in degrees
grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot Position (u = 0)');
hold on
% plot(T, rad2deg(state_history_u_a_0(:, 1))); % Plotting theta in degrees
% grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot Position (u = 0)');

 
subplot(2, 1, 2);
plot(T, rad2deg(state_history_u_x_0(:, 3))); % Plotting theta in degrees
grid on; xlabel('Time [s]'); ylabel('Robot Angle [cm]'); title('Robot Position (u = 0)');
hold on
plot(T, rad2deg(target),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Robot Angle [deg]'); title('Robot Angle (u = 0)');
hold on
% plot(T, 100*state_history_u_a_0(:, 3)); % Plotting x
% grid on; xlabel('Time [s]'); ylabel('Robot body Angle [deg]'); title('Robot body Angle (u = 0)');


% % Plot results for Linear PD control
figure(2);
subplot(2, 1, 1);
plot(T, 100*state_history_F_ext0_PD(:, 1)); % Plotting theta in degrees
hold on
plot(T, 100*state_history_u_PD(:, 1)); % Plotting theta in degrees
grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot Position (PD Control)');
legend('external force=0','external force','target Angle');


subplot(2, 1, 2);
plot(T, rad2deg(state_history_F_ext0_PD(:, 3))); % Plotting x
hold on
plot(T, rad2deg(state_history_u_PD(:, 3))); % Plotting x
hold on
plot(T, rad2deg(target),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Robot Angle [deg]'); title('Robot body Angle (PD Control)');
legend('external force=0','external force','target Angle'); ylim([-10 10])
hold on

% Plot results for Linear PID control
figure(3);
subplot(2, 1, 1);
plot(T, 100*state_history_F_ext0_PID(:, 1)); % Plotting theta in degrees
hold on
plot(T, 100*state_history_u_PID(:, 1)); % Plotting theta in degrees
grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot Position (PID Control)');
legend('external force=0', 'external force');
hold on

subplot(2, 1, 2);
plot(T, rad2deg(state_history_F_ext0_PID(:, 3))); % Plotting theta in degrees
hold on
plot(T, rad2deg(state_history_u_PID(:, 3))); % Plotting x
hold on
plot(T, rad2deg(target),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Robot Angle [deg]'); title('Robot body Angle (PID Control)');
legend('external force=0','external force','target Angle'); ylim([-10 10]);
hold on



%% Parameters
%%====================================== Parameters ======================================%%
function params = Params_init() 
    % % PID 게인값 
  
    params.Ka_p = 15;  
    params.Ka_i = 1;
    params.Ka_d = 2; 

    % 초기값
    params.g = 9.81;                                       % 중력 (m/s^2)   
    params.m = 0.5;                                        % 바퀴 질량 (kg)
    params.M = 2;                                          % 로봇 질량 (kg)    
    params.L = 0.3;                                        % 로봇 길이 (m)
    params.r_w = 0.03;                                     % 막대 반지름
    params.r_b = 0.03;                                     % 바퀴 반지름    
    params.C_a = 0.00055;                                  % 외력

    params.dt = 1/1000;
end

%% Dynamics
%%====================================== Dynamics ======================================%%
function Robot_xdot_F_ext0 = inverted_pendulum_Robot_dynamics_F_ext0(X, u, params)

   % 초기값 Unpack 
    g = params.g;    
    M = params.M;
    m = params.m;
    L = params.L;
    r_w = params.r_w;
    r_b = params.r_b;
    I_b = (M * r_b ^2) / 2;

    %%=============== 상태변수 ===============%%
     
    % Position
    x = X(1);
    a = x / r_w;                                     % 바퀴 이동 거리 (rad)

    % Position V
    x_dot = X(2);
    a_dot = x_dot / r_w;                             % 바퀴 이동 속도 (rad/s)

    % theta
    theta = X(3);

    % theta V
    theta_dot = X(4);

    % 선형 방정식 치환
    A = (M + m);
    B = I_b + M * (L ^ 2);
    denominator = A * B * (r_w ^ 2) - (r_w * M * L)^2;
    numerator_a = A * u + r_w * ((M * L)^2) * g * theta;
    numerator_theta = r_w * M * L * u + B * (r_w ^ 2) * M * L * g * theta;
         
    % Position A
    a_2dot = numerator_a / denominator;
    x_2dot = a_2dot * r_w;
  
    % theta A
    theta_2dot = numerator_theta / denominator;    
 
    % State derivatives
    Robot_xdot_F_ext0 = [x_dot; x_2dot; theta_dot; theta_2dot;];
end

function Robot_xdot = inverted_pendulum_Robot_dynamics(X, u, params)

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

    %%=============== 상태변수 ===============%%
    
    % Position
    x = X(1);
    a = x / r_w;                                     % 바퀴 이동 거리 (rad)

    % Position V
    x_dot = X(2);
    a_dot = x_dot / r_w;                             % 바퀴 이동 속도 (rad/s)

    % theta
    theta = X(3);

    % theta V
    theta_dot = X(4);


    % 선형 방정식 치환
    A = ((M + m) * r_w^2 + I_w);
    B = (I_b + M * L^2);
    denominator = A * B - (r_w * M * L)^2;
    numerator_a = r_w * g * theta * (M * L)^2 - B * C_a * a_dot + B * u;
    numerator_theta = A * M * L * g * theta - r_w * M * L * C_a * a_dot + r_w * M * L * u;
         
    % Position A
    a_2dot = numerator_a / denominator;
    x_2dot = a_2dot * r_w;
  
    % theta A
    theta_2dot = numerator_theta / denominator;    
 
    % State derivatives
    Robot_xdot = [x_dot; x_2dot; theta_dot; theta_2dot;];
end

%% State-space matrices
%%====================================== State space matrices ======================================%%
function [A, B, C, D] = state_space_matrices(params)
    g = params.g;
    m = params.m;
    M = params.M;
    L = params.L;
    r = params.r;
    I = params.I;
    
    % 선형 방정식 치환
    a = I + m * (L / 2)^2;
    alpha = a * (M + m) - (L * m / 2)^2;
    a_2_1 = (g / alpha) * (L * m / 2)^2;
    a_4_1 = (m * g * L ) * (M + m) / 2 * alpha;
    b_2_1 = a / alpha;
    b_4_1 = (m * L) / 2 * alpha;

    A = [ 0,        1,       0,        0 ;
          a_2_1,    0,       0,        0 ;
          0,        0,       0,        1 ;
          a_4_1,    0,       0,        0];

    B = [ 0; b_2_1; 0; b_4_1 ];

    C = [1, 0, 0, 0;
         0, 0, 1, 0];

    D = [0; 0];
end
%% PD Integration Function
%%====================================== PD Controller Function ======================================%%
function u_PD = Robot_Controller_F_ext0_PD(X, X_target, params)
    % % Robot PID 게인값 
    Kp = params.Ka_p;    
    Kd = params.Ka_d;    
    
    % PD 제어    
    theta0 = X(3);
    theta_dot0 = X(4);
    e_theta = X_target(3) - theta0;                
    e_theta_v = X_target(4) - theta_dot0;    
    u_PD = (Kp * e_theta + Kd * e_theta_v);    
    
end

function u_PD = Robot_Controller_PD(X, X_target, params)
    % % Robot PID 게인값 
    Kp = params.Ka_p;    
    Kd = params.Ka_d;    
    
    % PD 제어    
    theta0 = X(3);
    theta_dot0 = X(4);
    e_theta = X_target(3) - theta0;                
    e_theta_v = X_target(4) - theta_dot0;    
    u_PD = (Kp * e_theta + Kd * e_theta_v);    
    
end

%% PID Controller
%%====================================== PID Controller Function ======================================%%
function u_PID = Robot_Controller_F_ext0_PID(X, X_target, params) 
    persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end

    % Robot PID 게인값 
    Kp = params.Ka_p; 
    Ki = params.Ka_i; 
    Kd = params.Ka_d;  
    
    % PD 제어    
    theta0 = X(3);  
    theta_dot0 = X(4);

    e_theta = X_target(3) - theta0;
    
    % PID control law
    integral_theta = integral_theta + e_theta * params.dt;
    derivative_theta = (e_theta - error_previous) / params.dt;
    u_PID = (Kp * e_theta + Ki * integral_theta + Kd * derivative_theta);

    % Update previous error
    error_previous = e_theta;    
end

function u_PID = Robot_Controller_PID(X, X_target, params)
    persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end

    % Robot PID 게인값 
    Kp = params.Ka_p; 
    Ki = params.Ka_i; 
    Kd = params.Ka_d;     
    
    % PD 제어    
    theta0 = X(3);
    theta_dot0 = X(4);

    e_theta = X_target(3) - theta0;
    
    % PID control law
    integral_theta = integral_theta + e_theta * params.dt;
    derivative_theta = (e_theta - error_previous) / params.dt;
    u_PID = (Kp * e_theta + Ki * integral_theta + Kd * derivative_theta);

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

