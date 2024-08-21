clc;   
clear;   
close all;

%%====================================== Main ======================================%% 
% 초기값
params = Params_init();
r_w = params.r_w;
 
% 시간
dt = 0.01;                                                     % 시간차 
T = 0:dt:10;                                                    % 총 시간

% 초기 조건
pos_I = 0;                                                     % 로봇 위치 (m)
Vel_B = 0;                                                     % 로봇 속도 (m/s)
ang_I = deg2rad(40);                                           % 로봇 몸체 각도 (rad)
ang_V_B = 0;                                                   % 로봇 몸체 각속도 (rad/s)

Robot_init = [pos_I; Vel_B; ang_I; ang_V_B];                   % State vector

Robot_PID = Robot_init;

% 다중 웨이포인트
pos_d_1 = 0;
pos_d_2 = 3;
pos_d_3 = 8;
pos_d_4 = 0;

pos_d_all = [pos_d_1]; %; pos_d_2; pos_d_3; pos_d_4
pos_d_size = length(pos_d_all(:,1));

save_cnt = 1;

% Preallocation
state_history_Robot = zeros(2, length(T));
state_history_PID = zeros(4, length(T));
state_history_target = zeros(2, length(T));

% 초기값
state_history_Robot = [pos_I; ang_I];                          % 출력값 배열 생성
state_history_PID(: , 1) = Robot_PID;                          % 초기값 배열 생성
state_history_target(: , 1) = [pos_I; ang_I];                  % 목표 제어값 배열 생성


for waypoint = 1 : pos_d_size

    %%=============== 제어값 ===============%%
    pos_d = pos_d_all(waypoint,1);                             % 로봇 위치 제어 (m)
    Vel_d = 0;                                                 % 로봇 속도 제어 (m/s)
    ang_I_d = deg2rad(0);                                      % 로봇 몸체 각도 제어 (rad)
    ang_V_B_d = 0;                                             % 로봇 몸체 각속도 제어 (rad/s)

    target = [pos_d; ang_I_d];                                 % 목표 제어값 
    state_history_target(: , 1) = target;

    for i = 1 : length(T) - 1

        % 제어 입력            
        U_pos = Position_PID(Robot_PID, target, params);
        U_angle = Angle_PID(Robot_PID, target, params);

        U = U_pos + U_angle;
        
        % Rk4        
        Robot_PID = Rk4(@inverted_pendulum_Robot_dynamics, Robot_PID, U, dt, params );       

        % 변수 저장
        state_history_Robot(1, save_cnt + 1) = Robot_PID(1, :);
        state_history_Robot(2, save_cnt + 1) = Robot_PID(3, :);

        state_history_target(1, save_cnt + 1) = target(1);
        state_history_target(2, save_cnt + 1) = target(2);
     
        save_cnt = save_cnt + 1;
    end        

end

Position(1,:) = state_history_Robot(1, :);
Angle(1, :) = state_history_Robot(2, :);

pos_target(1,:) = state_history_target(1, :);
angle_target(1, :) = state_history_target(2, :);

T = 1 : save_cnt;
T = T*dt;

%%====================================== Plot ======================================%%
% Plot results for Linear PID control
Yt_pls = 25;
Yt_neg = -25;

figure(1);
subplot(2, 1, 1);
plot(T, Position(1,:)); % Plotting Position
hold on
plot(T, pos_target(1,:),"red--"); % Plotting target Position
grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot Position (PID Control)');
legend('Robot Position', 'Target Position'); 
% ylim([0 Yp_pls]);
hold on

subplot(2, 1, 2);
plot(T, rad2deg(Angle(1, :))); % Plotting theta in degrees
hold on
plot(T, rad2deg(angle_target(1, :)),"red--"); % Plotting target theta in degrees
grid on; xlabel('Time [s]'); ylabel('Robot Angle [deg]'); title('Robot body Angle (PID Control)');
legend('Robot body Angle','Target Angle');
% ylim([Yt_neg Yt_pls]);
hold on



%% Parameters
%%====================================== Parameters ======================================%%
function params = Params_init() 
    % % PID 게인값 
    params.pos_Kp = 0;  
    params.pos_Ki = 0;
    params.pos_Kd = 0;

    params.Theta_Kp = 0;  
    params.Theta_Ki = 0;
    params.Theta_Kd = 0;

    % params.pos_Kp = 0;  
    % params.pos_Ki = 0;
    % params.pos_Kd = 0;
    % 
    % params.Theta_Kp = 0;  
    % params.Theta_Ki = 0;
    % params.Theta_Kd = 0;

    

    % 초기값
    params.g = 9.81;                                           % 중력 (m/s^2)    
    params.m_w = 0.5;                                          % 바퀴 질량 (kg)
    params.m_b = 2;                                            % 로봇 질량 (kg)
    params.r_w = 0.03;                                         % 바퀴 반지름 (m)
    params.r_b = 0.03;                                         % 막대 반지름 (m)
    params.L = 0.3;                                            % 로봇 길이 (m)
    params.I_w = (params.m_w*params.r_w^2)/2;                  % 바퀴 관성모멘트 (kg m^2) 
    params.I_b = (params.m_b*params.r_b^2)/2;                  % 막대 관성모멘트 (kg m^2)
    
    params.f_w = 0.003;                                        % 바퀴에 작용하는 외력
    params.f_b = 0;                                            % 로봇에 작용하는 외력

    params.dt = 1/1000;
end

%% Dynamics
%%====================================== Dynamics ======================================%%
function Robot_xdot = inverted_pendulum_Robot_dynamics(X, U, params)

    % 초기값 Unpack 
    g = params.g;
    m_w = params.m_w;
    m_b = params.m_b;
    r_w = params.r_w;
    L = params.L;
    I_w = params.I_w;
    I_b = params.I_b;
    f_w = params.f_w;

    %%=============== 상태변수 ===============%%
    % Position
    x = X(1);

    % Position V 
    x_dot = X(2);

    % theta
    theta = X(3);

    % theta V
    theta_dot = X(4);

 
    % 비선형 방정식 치환
    W = (m_w + m_b) + I_w / (r_w ^ 2);
    P = I_b + m_b * L ^ 2;
    Q = m_b * L;
    denominator = (Q * cos(theta)) ^ 2 - W * P;
    numerator_X = - P * Q * sin(theta) * (theta_dot ^ 2) + (Q ^ 2) * sin(theta) * cos(theta) * g ...
                  + P * f_w + (Q * P * cos(theta) / r_w) * U;

    numerator_theta = (Q ^ 2) * sin(theta) * cos(theta) * (theta_dot ^ 2) - W * Q * sin(theta) * g ...
                  + Q * cos(theta) * f_w + (Q * cos(theta) / r_w + W) * U;
         
    % Position A
    x_2dot = numerator_X / denominator;
  
    % theta A
    theta_2dot = numerator_theta / denominator;    
 
    % State derivatives
    Robot_xdot = [x_dot; x_2dot; theta_dot; theta_2dot];
end

%%====================================== State space matrices ======================================%%
%% State-space matrices
function [A, B, C, D] = state_space_matrices(params)
    g = params.g;
    m_w = params.m_w;
    m_b = params.m_b;
    r_w = params.r_w;
    L = params.L;
    I_w = params.I_w;
    I_b = params.I_b;
    
    % 선형 방정식 치환
    W = (m_w + m_b) + I_w / (r_w ^ 2);
    P = I_b + m_b * L ^ 2;
    Q = m_b * L;

    a_23 = (Q ^ 2) * g / H;
    a_43 = - W * Q * g / H;
    b_21 = (Q * P) / (H * r_w);
    b_41 = Q / (H * r_w) + W;

    A = [ 0,        1,       0,        0 ;
          0,        0,       a_23,     0 ;
          0,        0,       0,        1 ;
          0,        0,       a_43,     0 ];

    B = [ 0; b_21; 0; b_41 ];

    C = [1, 0, 0, 0;
         0, 0, 1, 0];

    D = [0; 0];
end
%% PID Controller
%%====================================== PID Controller Function ======================================%%
function U_pos = Position_PID(X, target, params)
    persistent integral_pos error_previous;
    if isempty(integral_pos)
        integral_pos = 0;
        error_previous = 0;
    end

    % Robot PID 게인값 
    Kp = params.pos_Kp; 
    Ki = params.pos_Ki; 
    Kd = params.pos_Kd;     
    
    % X_target = [x_d, x_dot_d, theta_d, theta_dot_d];
    x0 = X(1);
    pos_er = target(1) - x0;
    
    % PID control law
    integral_pos = integral_pos + pos_er * params.dt;
    derivative_pos = (pos_er - error_previous) / params.dt;
    U_pos = -(Kp * pos_er + Ki * integral_pos + Kd * derivative_pos);

    % Update previous error
    error_previous = pos_er;    
end

function U_angle = Angle_PID(X, target, params)
    persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end

    % Robot PID 게인값 
    Kp = params.Theta_Kp; 
    Ki = params.Theta_Ki; 
    Kd = params.Theta_Kd;  
    
    % X_target = [x_d, x_dot_d, theta_d, theta_dot_d];  
    theta0 = X(3);
    angle_er = target(2) - theta0;
    
    % PID control law
    integral_theta = integral_theta + angle_er * params.dt;
    derivative_theta = (angle_er - error_previous) / params.dt;
    U_angle = -(Kp * angle_er + Ki * integral_theta + Kd * derivative_theta);

    % Update previous error
    error_previous = angle_er;    
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

