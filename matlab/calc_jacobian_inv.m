function J_inv = calc_jacobian_inv(q)
% Calcula J^-1(q) para velocidades cartesianas do LIMO e Crazyflie.

rho = q(4);
alpha = q(5);
beta = q(6);

S = [cos(alpha) * cos(beta), -rho * sin(alpha) * cos(beta), -rho * cos(alpha) * sin(beta); ...
     sin(alpha) * cos(beta),  rho * cos(alpha) * cos(beta), -rho * sin(alpha) * sin(beta); ...
     sin(beta),                0,                              rho * cos(beta)];

J_inv = [eye(3), zeros(3); ...
         eye(3), S];
end
