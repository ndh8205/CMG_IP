function [ U ]  = Control_Allocator( U, params )

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
    H = 1;

    b_21 = (Q * P) / (H * r_w);
    b_41 = Q / (H * r_w) + W;

    Ad = [ 0; b_21; 0; b_41 ];


    U = pinv( Ad ) * U; % A * ( A * A^T )^-1 moore penrose inverse matrix
      
end