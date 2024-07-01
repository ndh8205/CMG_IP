clc;
clear;
close all;
%%====================================== Main ======================================%%
% 초기값
params = Params_init();
r = params.r;

% 시간
dt = 0.001;                                      % 시간차 
T = 0:dt:5;                                     % 총 시간

% 초기 조건

x0 = 0;                                          % 카트 위치 (m)
x_dot0 = 0;                                      % 카트 속도 (m/s)
theta0 = deg2rad(20);                            % 진자 각도 (rad)
theta_dot0 = 0;                                  % 진자 각속도 (rad/s)

X_init = [x0; x_dot0; theta0; theta_dot0];       % State vector

X0 = X_init;
Linear_X_PD = X_init;
X_PD = X_init;
X_PID = X_init;

% 제어값
x_d = 0;                                         % 제어 카트 위치
x_dot_d = 0;                                     % 제어 카트 속도
theta_d = deg2rad(0);                            % 제어 진자 각도
theta_dot_d = 0;                                 % 제어 진자 각속도

X_target  = [x_d, x_dot_d, theta_d, theta_dot_d];

% Preallocation
state_history_u0 = zeros(length(T), 4);
state_history_Linear_PD = zeros(length(T), 4);
state_history_PD = zeros(length(T), 4);
state_history_PID = zeros(length(T), 4);
target = zeros(length(T), 4);
  
for i = 1 : length(T) - 1
 
    % 초기값
    state_history_u0(1 , :) = [0,0,theta0,0];
    state_history_Linear_PD(1 , :) = [0,0,theta0,0];
    state_history_PD(1 , :) = [0,0,theta0,0];
    state_history_PID(1 , :) = [0,0,theta0,0];

    %목표값
    target(i) = theta_d;

    % 제어 입력
    u_non = 0;
    Linear_u_PID = Controller_PD(Linear_X_PD, X_target, params);
    u_PD = Controller_PD(X_PD, X_target, params);
    u_PID = Controller_PID(X_PID, X_target, params);

    % Rk4
    X0 = Rk4 (@inverted_pendulum_dynamics, X0, u_non, dt, params);    
    Linear_X_PD = Rk4(@inverted_pendulum_Linear_dynamics, Linear_X_PD, Linear_u_PID, dt, params);
    X_PD = Rk4(@inverted_pendulum_dynamics, X_PD, u_PD, dt, params);    
    X_PID = Rk4(@inverted_pendulum_dynamics, X_PID, u_PID, dt, params);    

    % 변수 저장
    state_history_u0(i + 1, :) = X0';
    state_history_Linear_PD(i + 1, :) = Linear_X_PD';
    state_history_PD(i + 1, :) = X_PD';  
    state_history_PID(i + 1, :) = X_PID';  

   
end 

%%====================================== Plot ======================================%%
% Plot results for u = 0
figure(1)
subplot(2,1,1)
plot(T, 100*state_history_u0(:, 1))
grid on; xlabel('Time [s]'); ylabel('Cart Position [cm]');
title('Cart Position (u = 0)');


subplot(2,1,2)
plot(T, rad2deg(state_history_u0(:, 3)))
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]');
hold on
plot(T, rad2deg(target),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]'); 
title('Pendulum Angle (u = 0)'); legend('Angle','target Angle');
ylim([-342 342]);     
hold on

% Plot results for Linear PD control
figure(2);
subplot(2, 1, 1); 
plot(T, 100*state_history_Linear_PD(:, 1)); % Plotting theta in degrees
grid on; xlabel('Time [s]'); ylabel('Cart Position [cm]'); 
title('Linear Cart Position (PD Control)');
hold on

subplot(2, 1, 2);
plot(T, rad2deg(state_history_Linear_PD(:, 3))); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]'); 
hold on
plot(T, rad2deg(target),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]'); 
title('Linear Pendulum Angle (PD Control)'); legend('Pendulum Angle','target Angle');
hold on

figure(3);
subplot(2, 1, 1);
plot(T, 100*state_history_PD(:, 1)); % Plotting theta in degrees
grid on; xlabel('Time [s]'); ylabel('Cart Position [cm]'); 
title('Cart Position (PD Control)');
hold on

subplot(2, 1, 2);
plot(T, rad2deg(state_history_PD(:, 3))); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]');
hold on
plot(T, rad2deg(target),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]'); 
title('Pendulum Angle (PD Control)'); legend('Pendulum Angle','target Angle');
ylim([-20 20]); 
hold on
 
figure(4);
subplot(2, 1, 1);
plot(T, 100*state_history_PID(:, 1)); % Plotting theta in degrees
grid on; xlabel('Time [s]'); ylabel('Cart Position [cm]'); 
title('Cart Position (PID Control)');

hold on

subplot(2, 1, 2);
plot(T, rad2deg(state_history_PID(:, 3))); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]');
hold on
plot(T, rad2deg(target),"red--"); % Plotting x 
grid on; xlabel('Time [s]'); ylabel('Pendulum Angle [deg]');
title('Pendulum Angle (PID Control)'); legend('Pendulum Angle','target Angle'); 
ylim([-20 20]);  
hold on  

%%====================================== Parameters ======================================%%
function params = Params_init() 
    % % PID 게인값 
    params.Ka_p = 50;
    params.Ka_i = 21; %5
    params.Ka_d = 10; %10 

    % 초기값
    params.g = 9.81;                                       % 중력 (m/s^2)    
    params.M = 2;                                          % 카트 질량 (kg)
    params.m = 0.5;                                        % 막대 질량 (kg)
    params.L = 0.3;                                        % 막대 길이 (m)
    params.r = 0.05;                                       % 막대 반지름
    params.I = (params.m*params.r^2)/2;                    % 막대 관성모멘트

    params.dt = 1/1000;
end
%%====================================== Dynamics ======================================%%
function Linear_xdot = inverted_pendulum_Linear_dynamics(X, u, params)

    % 초기값 Unpack 
    g = params.g;
    m = params.m;
    M = params.M;
    L = params.L;
    I = params.I;

    % 선형 방정식 치환
    a = I + m * ((L / 2)^2);
    alpha = a * (M + m) - (L * m / 2)^2;
    a_2_3 = (g / alpha) * ((L * m / 2)^2);
    a_4_3 = ((m * g * L ) * (M + m)) / (2 * alpha);
    b_2_1 = a / alpha;
    b_4_1 = (m * L) / (2 * alpha);

    %%=============== 상태변수 ===============%%
    % Position
    x = X(1);

    % Position V
    x_dot = X(2);

    % theta
    theta = X(3);

    % theta V
    theta_dot = X(4);
     
    % Position A
    x_2dot = a_2_3 * theta + b_2_1 * u;

    % theta A
    theta_2dot = a_4_3 * theta + b_4_1 * u;
 
    % State derivatives
    Linear_xdot = [x_dot; x_2dot; theta_dot; theta_2dot;];
end

function xdot = inverted_pendulum_dynamics(X, u, params)

    % 초기값 Unpack 
    g = params.g;
    m = params.m;
    M = params.M;
    L = params.L;
    I = params.I;
    

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
    beta = (I + m * (L/2)^2);
    denominator = beta * (M + m) - (L * m / 2)^2 * cos(theta)^2;
    numerator_X = beta * (u - (L/2) * m * sin(theta) * theta_dot^2) + (L * m / 2)^2 * g * sin(theta) * cos(theta);
    numerator_theta = u * (L/2) * m * cos(theta) - (L* m/2)^2  * sin(theta) * cos(theta) * theta_dot^2 ...
                      + (M + m) * (L/2) * m * g * sin(theta);
         
    % Position A
    x_2dot = numerator_X / denominator;
  
    % theta A
    theta_2dot = numerator_theta / denominator;    
 
    % State derivatives
    xdot = [x_dot; x_2dot; theta_dot; theta_2dot;];
end
%%====================================== State space matrices ======================================%%
%% State-space matrices
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
%%====================================== PD Controller Function ======================================%%
function u = Controller_PD(X, X_target, params)

    % PID gains
    Kp = params.Ka_p;
    Kd = params.Ka_d;

    % PD 제어    
    theta0 = X(3);
    theta_dot0 = X(4);
    e_theta = X_target(3) - theta0;                
    e_theta_v = X_target(4) - theta_dot0;   
    u = (Kp * e_theta + Kd * e_theta_v);    
    
end

%% PID Controller
%%====================================== PID Controller Function ======================================%%
function u_PID = Controller_PID(X, X_target, params)
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
    e_theta = X_target(3) - theta0;
    
    % PID control law
    integral_theta = integral_theta + e_theta * params.dt;
    derivative_theta = (e_theta - error_previous) / params.dt;
    u_PID = (Kp * e_theta + Ki * integral_theta + Kd * derivative_theta);

    % Update previous error
    error_previous = e_theta;    
end
%%====================================== Rk4 ======================================%%
%% RK4 Integration Function
function X = Rk4 (Func, X, u, dt, params)
a= 0.5;
b= 0.5;

k1 = Func(X, u, params) * dt;
k2 = Func(X + a*k1, u, params) * dt;
k3 = Func(X + + b*k2, u, params) * dt;
k4 = Func(X + + b*k3, u, params) * dt;
X = X + (k1  + 2*(k2 + k3) + k4) / 6;

end

