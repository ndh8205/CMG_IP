%% PID Controller
%%====================================== PID Controller Function ======================================%%
function U = PID_C(X, target, params)

    persistent integral_pos error_previous_p integral_theta error_previous_a;

    if isempty(integral_pos)
        integral_pos = 0;
        error_previous_p = 0;
        integral_theta = 0;
        error_previous_a = 0;
    end

    % Robot PID 게인값 
    Kp_p = params.pos_Kp; 
    Ki_p = params.pos_Ki; 
    Kd_p = params.pos_Kd;     

    % Robot PID 게인값 
    Kp_a = params.Theta_Kp; 
    Ki_a = params.Theta_Ki; 
    Kd_a = params.Theta_Kd;
    
    % X_target = [x_d, x_dot_d, theta_d, theta_dot_d];
    x0 = X(1);
    pos_er = target(1) - x0;

    theta0 = X(3);
    angle_er = target(2) - theta0;
    
    % PID control law
    integral_pos = integral_pos + pos_er * params.dt;
    derivative_pos = (pos_er - error_previous_p) / params.dt;
    U_pos = -(Kp_p * pos_er + Ki_p * integral_pos + Kd_p * derivative_pos);

    integral_theta = integral_theta + angle_er * params.dt;
    derivative_theta = ( angle_er - error_previous_a ) / params.dt;
    U_angle = -( Kp_a * angle_er + Ki_a * integral_theta + Kd_a * derivative_theta );

    U = [ U_pos; U_angle ];

    % Update previous error
    error_previous_p = pos_er;   
    error_previous_a = angle_er;    

end