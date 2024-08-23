%% Parameters
%%====================================== Parameters ======================================%%
function params = Params_init() 
    % % PID 게인값 
    params.pos_Kp = 1;  
    params.pos_Ki = 0;
    params.pos_Kd = 0;
 
    params.Theta_Kp = 1.5;  
    params.Theta_Ki = 1;
    params.Theta_Kd = 0.01;

    % params.pos_Kp = 0;  
    % params.pos_Ki = 0;11111111
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