clc;   
clear;
close all;
%%====================================== Main ======================================%% 
% 초기값
params = Params_init();

% 시간
dt = 0.001;                                                         % 시간차 
T = 0:dt:5;                                                         % 총 시간

% 초기 조건
theta0 = deg2rad(10);                                               % 로봇 각도 (rad)
theta_dot0 = 0;                                                     % 로봇 각속도 (rad/s)
phi0 = deg2rad(0);                                                  % gimbal 각도 (rad)
phi_dot0  = 0;                                                      % gimbal 각속도 (rad/s)
x0 = 0;                                                             % 로봇의 위치 (m)
x_dot0 = 0;                                                         % 로봇의 속도 (m/s)

X_init = [theta0; theta_dot0; phi0; phi_dot0; x0; x_dot0];          % State vector
 
CMG_PID = X_init;

% 다중 웨이포인트
x_d1 = 0;
x_d2 = 3;
x_d3 = 8;
x_d4 = 0;

x_d_all = [x_d1]; %; x_d2; x_d3; x_d4
x_d_size = length(x_d_all(:,1));

save_cnt = 1;

% Preallocation
state_history_Robot = zeros(3, length(T));
state_history_CMG_PID = zeros(6, length(T));
state_history_target = zeros(3, length(T));

% 초기값
state_history_Robot(: , 1) = [theta0; phi0; x0];                % 출력값 배열 생성
state_history_CMG_PID(: , 1) = X_init;
state_history_target(: , 1) = [theta0; phi0; x0];               % 목표 제어값 배열 생성

for waypoint = 1 : x_d_size

    %%=============== 제어값 ===============%%
    theta_d = deg2rad(0);                                               % 로봇 몸체 각도 제어 (rad)
    theta_dot_d = 0;                                                    % 로봇 몸체 각속도 제어 (rad/s)
    phi_d = deg2rad(0);                                                 % gimbal 각도 (rad)
    phi_dot_d  = 0;                                                     % gimbal 각속도 (rad/s)
    x_d = x_d_all(waypoint,1);                                          % 로봇 위치 제어 (m)
    x_dot_d = 0;                                                        % 로봇 속도 제어 (m/s)
    
    target = [theta_d; phi_d; x_d];                                     % 목표 제어값 
   
    
    X_target = [theta_d; theta_dot_d; phi_d; phi_dot_d; x_d; x_dot_d];
    
    for i = 1 : length(T) - 1
        %목표값
        state_history_target(i) = theta_d;
        
        % 제어 입력 
        U_theta = -Wheel_PID(CMG_PID, X_target, params);
        U_phi = Gimbal_PID(CMG_PID, X_target, params);  
        
        U = [U_theta; U_phi]; 
        
        % Rk4
        CMG_PID = Rk4(@Two_wheel_Robot_with_CMG_dynamics, CMG_PID, U, dt, params);
        
        % 변수 저장
        state_history_Robot(1, save_cnt + 1) = CMG_PID(1, :);
        state_history_Robot(2, save_cnt + 1) = CMG_PID(3, :);
        state_history_Robot(3, save_cnt + 1) = CMG_PID(5, :);

        save_cnt = save_cnt + 1;    
    end

end

state_history_target(: , 1) = target;

T = 1 : save_cnt;
T = T*dt;

%%====================================== Plot ======================================%%
% Plot results for Linear PID control
figure(1);
subplot(3, 1, 1); 
plot(T, 100*state_history_Robot(3, :)); % Plotting x
hold on
plot(T, 100*state_history_target(3, :),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Robot Position [cm]'); title('Robot with CMG Position (PID Control)');
legend('Robot Position','Target Position');
hold on 
 
subplot(3, 1, 2); 
plot(T, rad2deg(state_history_Robot(2, :))); % Plotting theta in degrees   
hold on
plot(T, rad2deg(state_history_target(2, :)),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Robot Angle [deg]'); title('Robot with CMG body Angle (PID Control)');
legend('Robot Angle','Target Angle'); ylim([-10 10]);
hold on

subplot(3, 1, 3);  
plot(T, rad2deg(state_history_Robot(1,:))); % Plotting Gimbal Angle    
hold on
plot(T, rad2deg(state_history_target(1, :)),"red--"); % Plotting x
grid on; xlabel('Time [s]'); ylabel('Gimbal Angle [deg]'); title('Robot with CMG Gimbal Angle (PID Control)');
legend('Gimbal Angle','Target Angle'); ylim([-10 10]);
hold on

%% Parameters
%%====================================== Parameters ======================================%%
function params = Params_init() 
    % % PID 게인값 

    %---------------  wheel, gimbal gain값-2 ---------------%

    params.Ka_p_wheel = 70;   
    params.Ka_i_wheel = 350;
    params.Ka_d_wheel = 2; 


    params.Ka_p_gimbal = 50;   
    params.Ka_i_gimbal = 350;
    params.Ka_d_gimbal = 0.5; 
    %---------------------------------------------------------%
    
    
    % 초기값
    params.g = 9.81;                                       % 중력 (m/s^2)      
    
    params.m_g = 0.171;                                    % gombal 질량 (kg)
    params.M = 0.79;                                       % 전체 질량 (kg)        
    
    params.l_g = 0.09;                                     % gimbal 길이 (m)
    params.L = 0.07;                                       % 바퀴부터 로봇 무게중심 까지 길이 (m)
    
    params.I_fz = 0.000714;                                % 막대 반지름
    params.I_theta = 0.00872;                              % 막대 반지름
    params.I_phi = 0.000607;                               % 막대 반지름
    params.I_w = 0.000015;                                 % 막대 반지름

    params.omega = 193.73;                                 % 막대 반지름
    params.r_w = 0.040;                                    % 막대 반지름
    params.a = 42.79 * (-1);                               % 바퀴 반지름  
    params.b = 122.07;                                     % 바퀴 반지름       
    params.C_a = 0.00055;                                  % 외력

    params.dt = 1/1000;
end

%% Dynamics
%%====================================== Dynamics ======================================%%
function Robot_xdot = Two_wheel_Robot_with_CMG_dynamics(X, U, params)

    % 초기값 Unpack 
    g = params.g;   

    m_g = params.m_g;
    M = params.M;

    l_g = params.l_g;
    L = params.L;

    I_fz = params.I_fz;
    I_theta = params.I_theta;
    I_phi = params.I_phi;
    % I_w = params.I_w;
    
    omega = params.omega;
    r_w = params.r_w;
    a = params.a;
    b = params.b;
    C_a = params.C_a;

    U_alpha = U(1);
    U_phi = U(2);

    % U_alpha = U;
    % U_phi = U;

    %%=============== 상태변수 ===============%%
 
    % theta
    theta = X(1);

    % theta V
    theta_dot = X(2);

    % gimbal angle
    phi = X(3);
    
    % gimbal angle V
    phi_dot = X(4);
    
    % Position
    x = X(5);
    alpha = x / r_w;                                     % 바퀴 이동 거리 (rad)

    % Position V
    x_dot = X(6);
    alpha_dot = x_dot / r_w;                             % 바퀴 이동 속도 (rad/s)


    % 선형 방정식
    
    % theta A
    theta_2dot = (M * g * L / I_theta) * theta - (I_fz * omega / I_theta) * phi_dot ...
               - (M * r_w * L * a / I_theta) * alpha_dot - U_alpha * M * r_w * L * b / I_theta ;
    
    % gimbal angle A
    phi_2dot = (I_fz * omega / I_phi) * theta_dot - (m_g * g * l_g / I_phi) * phi ...
             - (C_a / I_phi) * phi_dot + U_phi / I_phi;
    
    % Position A
    alpha_2dot = a * alpha_dot + b * U_alpha;
    x_2dot = alpha_2dot * r_w;
 
    % State derivatives
    Robot_xdot = [theta_dot; theta_2dot; phi_dot; phi_2dot; x_dot; x_2dot];
end


%% PID Controller
%%====================================== PID Controller Function ======================================%%
function U_theta = Wheel_PID(X, X_target, params)
    persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end
    % X_target = [theta_d; theta_dot_d; phi_d; phi_dot_d; x_d; x_dot_d];
    % Robot PID 게인값 
    Kp_wheel = params.Ka_p_wheel; 
    Ki_wheel = params.Ka_i_wheel; 
    Kd_wheel = params.Ka_d_wheel;     
    
    % error    
    theta0 = X(1);
    theta_d = X_target(1);
    e_theta = theta_d - theta0;
    
    % PID control law
    integral_theta = integral_theta + e_theta * params.dt;
    derivative_theta = (e_theta - error_previous) / params.dt;
    U_theta = (Kp_wheel * e_theta + Ki_wheel * integral_theta + Kd_wheel * derivative_theta);

    % Update previous error
    error_previous = e_theta;    
end

function U_phi = Gimbal_PID(X, X_target, params)
    persistent integral_theta error_previous;
    if isempty(integral_theta)
        integral_theta = 0;
        error_previous = 0;
    end
    
    % Robot PID 게인값 
    Kp_gimbal = params.Ka_p_gimbal; 
    Ki_gimbal = params.Ka_i_gimbal; 
    Kd_gimbal = params.Ka_d_gimbal;     
    
    % error    
    phi0 = X(3);
    phi_d = X_target(3);
    e_phi = phi_d - phi0; 

    % PID control law
    integral_theta = integral_theta + e_phi * params.dt;
    derivative_phi = (e_phi - error_previous) / params.dt;
    U_phi = (Kp_gimbal * e_phi + Ki_gimbal * integral_theta + Kd_gimbal * derivative_phi);

    % Update previous error
    error_previous = e_phi;    
end

%% RK4 Integration Function
%%====================================== Rk4 ======================================%%

function X = Rk4 (Func, X, u, dt, params)

k1 = Func(X, u, params) * dt;
k2 = Func(X + 0.5*k1, u, params) * dt;
k3 = Func(X + + 0.5*k2, u, params) * dt;
k4 = Func(X + + 0.5*k3, u, params) * dt;
X = X + (k1  + 2*(k2 + k3) + k4) / 6;

end

