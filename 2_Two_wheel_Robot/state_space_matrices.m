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