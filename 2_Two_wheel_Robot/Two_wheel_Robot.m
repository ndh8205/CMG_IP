clc;   
clear all;   
% close all;

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

Robot_init = [ pos_I; Vel_B; ang_I; ang_V_B ];                   % State vector

X = Robot_init;

% 다중 웨이포인트
pos_d_1 = 0;
pos_d_2 = 3;
pos_d_3 = 8;
pos_d_4 = 0;

% pos_d_all = [ pos_d_1; pos_d_2; pos_d_3; pos_d_4 ]
pos_d_all = pos_d_1;

sim_time = length( pos_d_all( :,1 ) );

save_cnt = 1;

% Preallocation
state_history_Robot = zeros( 2, length(T) );
state_history_X = zeros( 4, length(T) );
state_history_target = zeros( 2, length(T) );

% 초기값
state_history_initial = [ pos_I; ang_I ];                        % 출력값 배열 생성
state_history_Robot( :, 1 ) = state_history_initial;
state_history_X( : , 1 ) = X;                          % 초기값 배열 생성
state_history_target( : , 1 ) = [ pos_I; ang_I ];                  % 목표 제어값 배열 생성


for waypoint = 1 : sim_time

    %%=============== 제어값 ===============%%
    pos_d = pos_d_all(waypoint,1);                             % 로봇 위치 제어 (m)
    Vel_d = 0;                                                 % 로봇 속도 제어 (m/s)
    ang_I_d = deg2rad( 0 );                                      % 로봇 몸체 각도 제어 (rad)
    ang_V_B_d = 0;                                             % 로봇 몸체 각속도 제어 (rad/s)

    target = [ pos_d; ang_I_d ];                                 % 목표 제어값 
    state_history_target( : , 1 ) = target;

    for i = 1 : length( T ) - 1

        % 제어 입력            
        U = PID_C( X, target, params );
        U = Control_Allocator( U, params );
        
        % Rk4        
        X = Rk4( @inverted_pendulum_Robot_dynamics, X, U, dt, params );       

        % 변수 저장
        state_history_Robot( 1, save_cnt + 1 ) = X( 1, : );
        state_history_Robot( 2, save_cnt + 1 ) = X( 3, : );

        state_history_target( 1, save_cnt + 1 ) = target( 1 );
        state_history_target( 2, save_cnt + 1 ) = target( 2 );
     
        save_cnt = save_cnt + 1;
    end        

end

Position(1,:) = state_history_Robot(1, :);
Angle(1, :) = state_history_Robot(2, :);

pos_target(1,:) = state_history_target(1, :);
angle_target(1, :) = state_history_target(2, :);


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










