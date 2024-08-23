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
