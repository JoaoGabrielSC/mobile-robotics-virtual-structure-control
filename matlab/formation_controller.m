function q_dot_ref = formation_controller(q, qd, qd_dot, Kq, Lq)
% Controlador cinemático saturado no espaço do cluster.

error_q = qd - q;
error_q(5:6) = atan2(sin(error_q(5:6)), cos(error_q(5:6)));
q_dot_ref = qd_dot + Lq * tanh(Lq \ (Kq * error_q));
end
