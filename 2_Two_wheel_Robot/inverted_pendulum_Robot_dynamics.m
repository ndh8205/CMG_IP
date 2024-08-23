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
    Robot_xdot = [ x_dot; x_2dot; theta_dot; theta_2dot ];
end

%%====================================== State space matrices ======================================%%