% Simulador offline de matlab/formacao_2.m — sem ROS, OptiTrack ou joystick.
% Reproduz a mesma lei de controle (cluster + NSB + compensadores dinâmicos)
% sobre plantas virtuais do LIMO (unicycle cinemático) e do Bebop
% (v_dot = f1*u - f2*v), para validar o comportamento antes de ir ao
% laboratório. Convenção de ganhos e valores de cfg idênticos aos de
% matlab/formacao_2.m.
clear; clc; close all;

%% Configuração (idêntica a matlab/formacao_2.m)
cfg.T = 1 / 30;
cfg.Tsim = 120;
cfg.preparation_time_s = 10;
cfg.v_max = 0.30;
cfg.w_max = 1.20;
cfg.a1 = 0.10;
cfg.kq = 0.8;
cfg.lq = 0.30;
cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];
cfg.kd_limo = 4.0;

cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence_radius = 0.50;
cfg.use_obstacle_avoidance = true;
cfg.obstacle_potential_gain = 0.50;
cfg.obstacle_potential_exponent = 4;
cfg.obstacle_potential_shape_a = 0.12; % fixo: zona de repulsão forte não escala com influence_radius
cfg.obstacle_potential_shape_b = 0.12;
cfg.obstacle_potential_vmax = 0.25;    % <= cfg.v_max, senão o LIMO nunca "vence" a repulsão
cfg.crossing_center = [0.0; 0.0];
cfg.crossing_zone_radius = 0.01;
cfg.crossing_feedback_min = 0.20;
cfg.crossing_cross_track_gain = 0.35;

TRAJ = 1; % 0: cluster para na origem, 1: lemniscata
rho_f = 1.5;
alpha_f = 0;
beta_f = pi / 3;
K_shape_diag = [1.2; 1.0; 1.2];
L_shape_diag = [0.3; 0.4; 0.4];

Kp_B = diag([1.0, 1.0, 1.2]);
Ls_B = diag([0.6, 0.6, 0.6]);
KD_B = diag([2.5, 2.5, 2.0, 2.5]);
f1 = diag([0.8417, 0.8354, 3.966, 9.8524]);
f2 = diag([0.18227, 0.17095, 4.001, 4.7295]);
cmdB_max = [0.5; 0.5; 0.3; 0.5];
vd_B_max = [0.5; 0.5; 0.3];

cfg.soft_start_time_s = 8.0;
cfg.soft_start_gamma_min = 0.3;
cfg.dvd_B_filter_alpha = 0.3;
cfg.dvd_B_max = [1.0; 1.0; 0.6; 1.0];
cfg.cmdB_rate_max = [1.2; 1.2; 0.8; 1.2];

N = round((cfg.Tsim + cfg.preparation_time_s) / cfg.T);

%% Estado inicial (condições do SPEC)
x1 = 0.40; y1 = -0.25; psi1 = 0;
v_limo_state = [0; 0];

p2 = [x1; y1 + 0.30; 0]; % Bebop pousado ~30 cm ao lado do LIMO
psi2 = 0;
v_body_B = zeros(4, 1);

vd_B_ant = zeros(4, 1);
dvd_B_filt = zeros(4, 1);
cmdB_prev = zeros(4, 1);
em_preparacao_ant = true;

H.t = zeros(1, N);
H.poi = zeros(2, N);
H.ref = zeros(2, N);
H.p2 = zeros(3, N);
H.p2d = zeros(3, N);
H.erroB = zeros(3, N);
H.cmdL = zeros(2, N);
H.cmdB = zeros(4, N);
H.dobs = zeros(1, N);
H.satB = false(1, N);
H.gamma = zeros(1, N);
H.satRate = false(1, N);

%% Loop de simulação
for k = 1:N
    t = (k - 1) * cfg.T;
    dt = cfg.T;

    poi = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];
    em_preparacao = t < cfg.preparation_time_s;
    t_traj = max(0, t - cfg.preparation_time_s);
    gamma = min(max(t / cfg.soft_start_time_s, cfg.soft_start_gamma_min), 1);

    if em_preparacao
        ref_xy = poi;
        cmdL = [0; 0];
        p2d = [poi; rho_f];
        Kp_B_eff = gamma * Kp_B;
        dx2 = Ls_B * tanh(Ls_B \ (Kp_B_eff * (p2d - p2)));
    else
        if TRAJ == 1
            [ref_xy, ref_xy_dot] = lemniscata_reference(t_traj);
        else
            ref_xy = [0; 0]; ref_xy_dot = [0; 0];
        end

        q = cluster_state([poi; 0], p2);
        qd = [ref_xy; 0; rho_f; alpha_f; beta_f];
        qd_dot = [ref_xy_dot; 0; 0; 0; 0];

        err_xy = attenuate_crossing_error(ref_xy - poi, ref_xy_dot, poi, cfg);
        [kq_eff, lq_eff] = crossing_gain_scale(poi, cfg);
        K = diag([kq_eff; kq_eff; 1; gamma * K_shape_diag]);
        L = diag([lq_eff; lq_eff; 1; L_shape_diag]);

        q_tilde = qd - q;
        q_tilde(1:2) = err_xy;
        q_tilde(5:6) = wrap_pi(q_tilde(5:6));
        q_dot_r = qd_dot + L * tanh(L \ (K * q_tilde));

        if cfg.use_obstacle_avoidance
            q_dot_r = cluster_obstacle_nsb(q_dot_r, poi, cfg);
        end

        x_dot = cluster_jacobian_inv(q) * q_dot_r;
        x_dot(3) = 0;

        A1inv = [cos(psi1), sin(psi1); -sin(psi1) / cfg.a1, cos(psi1) / cfg.a1];
        u = A1inv * x_dot(1:2);
        vd_L = [saturar(u(1), cfg.v_max); saturar(u(2), cfg.w_max)];
        v_limo_state = limo_inner_loop(vd_L, v_limo_state, cfg);
        cmdL = v_limo_state;

        dx2 = x_dot(4:6);
        p2d = [poi; 0] + cluster_offset(rho_f, alpha_f, beta_f);
    end

    dx2 = [saturar(dx2(1), vd_B_max(1));
           saturar(dx2(2), vd_B_max(2));
           saturar(dx2(3), vd_B_max(3))];

    A2inv = [cos(psi2), sin(psi2), 0; -sin(psi2), cos(psi2), 0; 0, 0, 1];
    vB_meas = v_body_B; % planta virtual: estado interno real, sem ruído de pose
    vd_B = [A2inv * dx2; 0];
    KD_B_eff = gamma * KD_B;

    transicao_prep_formacao = em_preparacao_ant && ~em_preparacao;
    if k == 1 || transicao_prep_formacao
        dvd_B_filt = zeros(4, 1);
    else
        dvd_B_raw = (vd_B - vd_B_ant) / dt;
        dvd_B_filt = (1 - cfg.dvd_B_filter_alpha) * dvd_B_filt + cfg.dvd_B_filter_alpha * dvd_B_raw;
    end
    dvd_B_filt = max(min(dvd_B_filt, cfg.dvd_B_max), -cfg.dvd_B_max);
    dvd_B = dvd_B_filt;

    cmdB_raw = f1 \ (dvd_B + KD_B_eff * (vd_B - vB_meas) + f2 * vB_meas);
    cmdB_satN2 = max(min(cmdB_raw, cmdB_max), -cmdB_max);
    delta_max = cfg.cmdB_rate_max * dt;
    delta_cmd = max(min(cmdB_satN2 - cmdB_prev, delta_max), -delta_max);
    cmdB = cmdB_prev + delta_cmd;
    satB_rate = any(abs(cmdB - cmdB_satN2) > 1e-9);

    % Avanço das plantas virtuais
    psi1 = wrap_pi(psi1 + cmdL(2) * dt);
    x1 = x1 + cmdL(1) * cos(psi1) * dt;
    y1 = y1 + cmdL(1) * sin(psi1) * dt;

    R = [cos(psi2), -sin(psi2), 0; sin(psi2), cos(psi2), 0; 0, 0, 1];
    p2 = p2 + dt * R * v_body_B(1:3);
    psi2 = wrap_pi(psi2 + dt * v_body_B(4));
    v_body_B = v_body_B + dt * (f1 * cmdB - f2 * v_body_B);

    H.t(k) = t;
    H.poi(:, k) = poi;
    H.ref(:, k) = ref_xy;
    H.p2(:, k) = p2;
    H.p2d(:, k) = p2d;
    H.erroB(:, k) = p2d - p2;
    H.cmdL(:, k) = cmdL;
    H.cmdB(:, k) = cmdB;
    H.dobs(k) = norm(poi - cfg.obstacle_center);
    H.satB(k) = any(abs(cmdB_raw - cmdB) > 1e-9);
    H.gamma(k) = gamma;
    H.satRate(k) = satB_rate;

    vd_B_ant = vd_B;
    cmdB_prev = cmdB;
    em_preparacao_ant = em_preparacao;
end

%% Resultados
erro_norma = vecnorm(H.erroB, 2, 1);
fprintf('Erro RMS do Bebop: %.4f m\n', sqrt(mean(erro_norma.^2)));
fprintf('Erro máximo do Bebop: %.4f m\n', max(erro_norma));
fprintf('Distância mínima ao obstáculo: %.4f m\n', min(H.dobs));
fprintf('Amostras saturadas (nível 2): %d de %d\n', nnz(H.satB), N);
fprintf('Amostras com rate limiter ativo: %d de %d\n', nnz(H.satRate), N);

figure('Name', 'Simulação formacao_2: trajetórias XY', 'Color', 'w');
hold on; axis equal; grid on;
plot(H.ref(1, :), H.ref(2, :), 'k--', 'DisplayName', 'Lemniscata desejada');
plot(H.poi(1, :), H.poi(2, :), 'b', 'LineWidth', 1.5, 'DisplayName', 'PoI LIMO');
plot(H.p2(1, :), H.p2(2, :), 'r', 'LineWidth', 1.2, 'DisplayName', 'Bebop');
desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_radius, 'k-');
desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_influence_radius, 'k:');
xlabel('x [m]'); ylabel('y [m]');
legend('Location', 'bestoutside');

figure('Name', 'Simulação formacao_2: sinais no tempo', 'Color', 'w');
subplot(2, 2, 1);
plot(H.t, H.p2(3, :), 'r', H.t, H.p2d(3, :), 'k--'); grid on;
xlabel('t [s]'); ylabel('z Bebop [m]'); legend('real', 'desejado');
subplot(2, 2, 2);
plot(H.t, erro_norma); grid on;
xlabel('t [s]'); ylabel('||erro formação|| [m]');
subplot(2, 2, 3);
plot(H.t, H.gamma); grid on;
xlabel('t [s]'); ylabel('\gamma (soft start)');
subplot(2, 2, 4);
plot(H.t, double(H.satB), H.t, double(H.satRate)); grid on;
xlabel('t [s]'); ylabel('saturação'); legend('nível 2', 'rate limiter');

%% Funções locais (mesma lógica de matlab/formacao_2.m)

function q = cluster_state(p1, p2)
    d = p2 - p1;
    rho = norm(d);
    beta = asin(max(min(d(3) / max(rho, 1e-6), 1), -1));
    alpha = atan2(d(2), d(1));
    q = [p1; rho; alpha; beta];
end

function offset = cluster_offset(rho, alpha, beta)
    offset = rho * [cos(alpha) * cos(beta); sin(alpha) * cos(beta); sin(beta)];
end

function J_inv = cluster_jacobian_inv(q)
    rho = q(4); alpha = q(5); beta = q(6);
    ca = cos(alpha); sa = sin(alpha);
    cb = cos(beta);  sb = sin(beta);
    S = [ca * cb, -rho * sa * cb, -rho * ca * sb;
         sa * cb,  rho * ca * cb, -rho * sa * sb;
         sb,       0,              rho * cb];
    J_inv = [eye(3), zeros(3); eye(3), S];
end

function q_dot_safe = cluster_obstacle_nsb(q_dot_formation, poi_xy, cfg)
    % J1 é a Jacobiana da tarefa escalar (direção do gradiente repulsivo),
    % não um seletor de xf,yf — remove só a componente radial do erro,
    % preserva a tangencial (permite contornar o obstáculo).
    offset = poi_xy - cfg.obstacle_center;
    distance = norm(offset);
    q_dot_safe = q_dot_formation;
    if distance >= cfg.obstacle_influence_radius || distance <= 1e-6
        return;
    end
    grad = obstacle_repulsive_gradient(offset, cfg);
    grad_mag = norm(grad);
    if grad_mag <= 1e-9
        return;
    end
    task_dir = grad / grad_mag;
    J1 = [task_dir.', 0, 0, 0, 0];
    J1_pinv = J1.';
    null_proj = eye(6) - J1_pinv * J1;
    q_dot_safe = J1_pinv * grad_mag + null_proj * q_dot_formation;
end

function grad = obstacle_repulsive_gradient(offset, cfg)
    distance = norm(offset);
    direction = offset / distance;
    clearance = distance - cfg.obstacle_radius;
    if clearance <= 0
        grad = direction * cfg.obstacle_potential_vmax;
        return;
    end
    a = cfg.obstacle_influence_radius - cfg.obstacle_radius;
    b = a;
    if ~isempty(cfg.obstacle_potential_shape_a), a = cfg.obstacle_potential_shape_a; end
    if ~isempty(cfg.obstacle_potential_shape_b), b = cfg.obstacle_potential_shape_b; end
    n = cfg.obstacle_potential_exponent;
    dx = offset(1); dy = offset(2);
    scale = cfg.obstacle_potential_gain * exp(-((dx / a)^n + (dy / b)^n)) * n;
    grad = [scale * sign(dx) * abs(dx)^(n - 1) / a^n;
            scale * sign(dy) * abs(dy)^(n - 1) / b^n];
    grad_norm = norm(grad);
    if grad_norm > cfg.obstacle_potential_vmax
        grad = grad * (cfg.obstacle_potential_vmax / grad_norm);
    end
end

function err_xy = attenuate_crossing_error(err_xy, ref_xy_dot, poi, cfg)
    dist_cross = norm(poi - cfg.crossing_center);
    if dist_cross >= cfg.crossing_zone_radius || norm(ref_xy_dot) < 1e-4
        return;
    end
    tangent = ref_xy_dot / norm(ref_xy_dot);
    err_along = dot(err_xy, tangent) * tangent;
    err_cross = err_xy - err_along;
    blend = (cfg.crossing_zone_radius - dist_cross) / cfg.crossing_zone_radius;
    cross_gain = cfg.crossing_cross_track_gain + (1 - cfg.crossing_cross_track_gain) * (1 - blend);
    err_xy = err_along + cross_gain * err_cross;
end

function [kq_eff, lq_eff] = crossing_gain_scale(poi, cfg)
    dist_cross = norm(poi - cfg.crossing_center);
    if dist_cross >= cfg.crossing_zone_radius
        kq_eff = cfg.kq;
        lq_eff = cfg.lq;
        return;
    end
    scale = cfg.crossing_feedback_min + (1 - cfg.crossing_feedback_min) * (dist_cross / cfg.crossing_zone_radius)^2;
    kq_eff = cfg.kq * scale;
    lq_eff = cfg.lq * scale;
end

function [ref_xy, ref_xy_dot] = lemniscata_reference(t)
    phase_x = 2 * pi * t / 40;
    phase_y = 4 * pi * t / 40;
    ref_xy = [0.75 * sin(phase_x); 0.75 * sin(phase_y)];
    ref_xy_dot = [0.75 * (2 * pi / 40) * cos(phase_x);
                  0.75 * (4 * pi / 40) * cos(phase_y)];
end

function v_state = limo_inner_loop(v_d, v_state, cfg)
    u_real = v_state(1);
    w_real = v_state(2);
    Y1 = [u_real, 0, w_real^2, 0, 0, 0;
          0, w_real, 0, u_real, u_real * w_real, w_real];
    KD = diag([cfg.kd_limo, cfg.kd_limo]);
    u_control = Y1 * cfg.theta_limo + KD * (v_d - v_state);
    M1 = [cfg.theta_limo(1), 0; 0, cfg.theta_limo(2)];
    C1 = [cfg.theta_limo(4) * u_real, cfg.theta_limo(3) * w_real;
          cfg.theta_limo(5) * u_real + cfg.theta_limo(6) * w_real, 0];
    v_dot = M1 \ (u_control - C1 * v_state);
    v_state = v_state + cfg.T * v_dot;
    v_state = [saturar(v_state(1), cfg.v_max); saturar(v_state(2), cfg.w_max)];
end

function y = saturar(u, umax)
    y = max(min(u, umax), -umax);
end

function ang = wrap_pi(ang)
    ang = atan2(sin(ang), cos(ang));
end

function desenhar_circulo(xc, yc, r, estilo)
    th = linspace(0, 2 * pi, 100);
    plot(xc + r * cos(th), yc + r * sin(th), estilo, 'HandleVisibility', 'off');
end
