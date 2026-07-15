function u = dynamic_compensator(robot_type, v_des, v_meas, v_des_prev, cfg)
% Compensadores dinâmicos dos robôs no laço interno.

v_dot_des = (v_des - v_des_prev) / cfg.T;

if strcmp(robot_type, 'limo')
    linear_velocity = v_meas(1);
    angular_velocity = v_meas(2);
    Y = [linear_velocity, 0, angular_velocity^2, 0, 0, 0; ...
         0, angular_velocity, 0, linear_velocity, ...
         linear_velocity * angular_velocity, angular_velocity];
    u_control = Y * cfg.theta_limo + cfg.KD_limo * (v_des - v_meas);
    M = diag(cfg.theta_limo(1:2));
    C = [cfg.theta_limo(4) * linear_velocity, cfg.theta_limo(3) * angular_velocity; ...
        cfg.theta_limo(5) * linear_velocity + cfg.theta_limo(6) * angular_velocity, 0];
    v_dot = M \ (u_control - C * v_meas);
    u = v_meas + cfg.T * v_dot;
    u(1) = clamp_scalar(u(1), cfg.v_limo_max);
    u(2) = clamp_scalar(u(2), cfg.w_limo_max);
    return;
end

if strcmp(robot_type, 'crazyflie')
    raw_u = cfg.f1_cf \ (v_dot_des + cfg.KD_cf * (v_des - v_meas) + cfg.f2_cf * v_meas);
    theta = clamp_scalar(cfg.pitch_sign * raw_u(1), cfg.theta_max);
    phi = clamp_scalar(cfg.roll_sign * raw_u(2), cfg.phi_max);
    zdot = clamp_scalar(raw_u(3), cfg.zdot_max);
    psidot = clamp_scalar(raw_u(4), cfg.psidot_max);
    u = [phi; theta; zdot; psidot];
    return;
end

error('Tipo de robô inválido: %s', robot_type);
end

function value = clamp_scalar(value, limit)
value = min(max(value, -limit), limit);
end
