% Teste: LIMO real segue a lemniscata (mesma lógica de formacao_2.m —
% preparação, cluster, NSB, compensador dinâmico), com cmd_vel REAL
% enviado ao LIMO. O Bebop só tem a pose lida via OptiTrack; o comando
% dele é calculado e logado, mas NUNCA enviado (sem publisher de
% /B1/cmd_vel neste arquivo).
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
cfg.obstacle_potential_shape_a = 0.12;
cfg.obstacle_potential_shape_b = 0.12;
cfg.obstacle_potential_vmax = 0.25;
cfg.crossing_center = [0.0; 0.0];
cfg.crossing_zone_radius = 0.01;
cfg.crossing_feedback_min = 0.20;
cfg.crossing_cross_track_gain = 0.35;

TRAJ = 1; % 0: LIMO para na origem, 1: lemniscata
rho_f = 1.5;
alpha_f = 0;
beta_f = pi / 3;
K_shape_active = [1.2; 1.0; 1.2];
L_shape_active = [0.3; 0.4; 0.4];

Kp_B = diag([1.0, 1.0, 1.2]);
Ls_B = diag([0.6, 0.6, 0.6]);
KD_B = diag([2.5, 2.5, 2.0, 2.5]);
f1 = diag([0.8417, 0.8354, 3.966, 9.8524]);
f2 = diag([0.18227, 0.17095, 4.001, 4.7295]);
cmdB_max = [0.5; 0.5; 0.3; 0.5];
vd_B_max = [0.5; 0.5; 0.3];

cfg.soft_start_time_s = 8.0;
cfg.soft_start_gamma_min = 0.3;

limo_pose_topic = '/natnet_ros/L1/pose';
bebop_pose_topic = '/natnet_ros/B1/pose'; % confirme o nome do corpo rígido no Motive
BTN_STOP = 1;

cfg.optitrack_timeout_s = 0.5;
N = round((cfg.Tsim + cfg.preparation_time_s) / cfg.T);

%% ROS e OptiTrack (LIMO envia cmd_vel real; Bebop só é lido)
rosshutdown;
rosinit('http://192.168.0.100:11311');
pub_L = rospublisher('/L1/cmd_vel', 'geometry_msgs/Twist');
msg_L = rosmessage(pub_L);
pose_L = rossubscriber(limo_pose_topic, 'geometry_msgs/PoseStamped');
pose_B = rossubscriber(bebop_pose_topic, 'geometry_msgs/PoseStamped');
J = vrjoystick(1);

fprintf('Aguardando poses do LIMO e do Bebop...\n');
receive(pose_L, 10);
receive(pose_B, 10);

%% Estado inicial
[x1, y1, ~, psi1, ts1] = ler_pose(pose_L);
[x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
p2 = [x2; y2; z2];
poseB_ant = p2;
poseB_psi_ant = psi2;

last_ts_L = ts1; last_update_L = tic;
last_ts_B = ts2; last_update_B = tic;

v_limo_state = [0; 0];
em_preparacao_ant = true;
t_ant = 0;

H.t = zeros(1, N);
H.poi_limo = zeros(2, N);
H.ref = zeros(2, N);
H.cmdL = zeros(2, N);
H.p2 = zeros(3, N);
H.p2d = zeros(3, N);
H.erroB = zeros(1, N);
H.cmdB = zeros(4, N);

%% Loop de controle (LIMO real; Bebop só logado)
fprintf('Iniciando teste: LIMO real na lemniscata, Bebop só logado. Botão %d para parar.\n', BTN_STOP);
t0 = tic;
kf = 0;
try
    for k = 1:N
        tloop = tic;
        t = toc(t0);
        dt = cfg.T;
        if k > 1, dt = max(t - t_ant, 1e-3); end

        btns = button(J);
        if numel(btns) >= BTN_STOP && btns(BTN_STOP)
            fprintf('Parada solicitada pelo joystick.\n');
            break;
        end

        [x1, y1, ~, psi1, ts1] = ler_pose(pose_L);
        [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);

        if ts1 > last_ts_L, last_ts_L = ts1; last_update_L = tic; end
        if ts2 > last_ts_B, last_ts_B = ts2; last_update_B = tic; end
        if toc(last_update_L) > cfg.optitrack_timeout_s || toc(last_update_B) > cfg.optitrack_timeout_s
            fprintf('OptiTrack perdido por mais de %.1f s. Abortando.\n', cfg.optitrack_timeout_s);
            break;
        end

        poi_limo = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];
        p2 = [x2; y2; z2];
        em_preparacao = t < cfg.preparation_time_s;
        t_traj = max(0, t - cfg.preparation_time_s);
        gamma = min(max(t / cfg.soft_start_time_s, cfg.soft_start_gamma_min), 1);

        if em_preparacao
            ref_xy = poi_limo;
            v_limo_state = [0; 0];
            cmdL = [0; 0];
            p2d = [poi_limo; rho_f];
            Kp_B_eff = gamma * Kp_B;
            vd_B_world = Ls_B * tanh(Ls_B \ (Kp_B_eff * (p2d - p2)));
        else
            if TRAJ == 1
                [ref_xy, ref_xy_dot] = lemniscata_reference(t_traj);
            else
                ref_xy = [0; 0]; ref_xy_dot = [0; 0];
            end

            q = cluster_state([poi_limo; 0], p2);
            qd = [ref_xy; 0; rho_f; alpha_f; beta_f];
            qd_dot = [ref_xy_dot; 0; 0; 0; 0];

            err_xy = attenuate_crossing_error(ref_xy - poi_limo, ref_xy_dot, poi_limo, cfg);
            [kq_eff, lq_eff] = crossing_gain_scale(poi_limo, cfg);
            K = diag([kq_eff; kq_eff; 1; gamma * K_shape_active]);
            L = diag([lq_eff; lq_eff; 1; L_shape_active]);

            q_tilde = qd - q;
            q_tilde(1:2) = err_xy;
            q_tilde(5:6) = wrap_pi(q_tilde(5:6));
            q_dot_r = qd_dot + L * tanh(L \ (K * q_tilde));

            if cfg.use_obstacle_avoidance
                q_dot_r = cluster_obstacle_nsb(q_dot_r, poi_limo, cfg);
            end

            x_dot_cart = cluster_jacobian_inv(q) * q_dot_r;
            x_dot_cart(3) = 0;

            A1inv = [cos(psi1), sin(psi1); -sin(psi1) / cfg.a1, cos(psi1) / cfg.a1];
            u = A1inv * x_dot_cart(1:2);
            vd_L = [saturar(u(1), cfg.v_max); saturar(u(2), cfg.w_max)];
            v_limo_state = limo_inner_loop(vd_L, v_limo_state, cfg);
            cmdL = v_limo_state;

            vd_B_world = x_dot_cart(4:6);
            p2d = [poi_limo; 0] + cluster_offset(rho_f, alpha_f, beta_f);
        end

        vd_B_world = [saturar(vd_B_world(1), vd_B_max(1));
                      saturar(vd_B_world(2), vd_B_max(2));
                      saturar(vd_B_world(3), vd_B_max(3))];

        A2inv = [cos(psi2), sin(psi2), 0; -sin(psi2), cos(psi2), 0; 0, 0, 1];
        velWB = (p2 - poseB_ant) / dt;
        psidot2 = wrap_pi(psi2 - poseB_psi_ant) / dt;
        vB_meas = [A2inv * velWB; psidot2];
        vd_B = [A2inv * vd_B_world; 0];
        KD_B_eff = gamma * KD_B;

        cmdB_raw = f1 \ (KD_B_eff * (vd_B - vB_meas) + f2 * vB_meas);
        cmdB = max(min(cmdB_raw, cmdB_max), -cmdB_max); % calculado, NÃO enviado

        % Envio REAL do cmd_vel ao LIMO (Bebop não tem publisher neste arquivo)
        msg_L.Linear.X = cmdL(1);
        msg_L.Linear.Y = 0;
        msg_L.Linear.Z = 0;
        msg_L.Angular.Z = cmdL(2);
        send(pub_L, msg_L);

        H.t(k) = t;
        H.poi_limo(:, k) = poi_limo;
        H.ref(:, k) = ref_xy;
        H.cmdL(:, k) = cmdL;
        H.p2(:, k) = p2;
        H.p2d(:, k) = p2d;
        H.erroB(k) = norm(p2d - p2);
        H.cmdB(:, k) = cmdB;
        kf = k;

        poseB_ant = p2;
        poseB_psi_ant = psi2;
        t_ant = t;
        em_preparacao_ant = em_preparacao;

        if mod(k, 30) == 0
            fprintf('t=%5.1fs | poi=(%+.2f,%+.2f) | erroBebop=%.3f m | alvoB=(%+.2f,%+.2f,%+.2f) | cmdB=(%+.2f,%+.2f,%+.2f,%+.2f)\n', ...
                t, poi_limo(1), poi_limo(2), H.erroB(k), p2d(1), p2d(2), p2d(3), cmdB(1), cmdB(2), cmdB(3), cmdB(4));
        end
        pause(max(0, cfg.T - toc(tloop)));
    end
catch ME
    fprintf(2, 'ERRO no loop: %s\n', ME.message);
end

%% Encerramento
msg_L.Linear.X = 0; msg_L.Linear.Y = 0; msg_L.Linear.Z = 0; msg_L.Angular.Z = 0;
send(pub_L, msg_L);
rosshutdown;

%% Gráfico
if kf > 1
    idx = 1:kf;
    figure('Name', 'Teste: LIMO na lemniscata (Bebop só logado)', 'Color', 'w');
    subplot(1, 2, 1);
    hold on; axis equal; grid on;
    plot(H.ref(1, idx), H.ref(2, idx), 'k--', 'DisplayName', 'Lemniscata desejada');
    plot(H.poi_limo(1, idx), H.poi_limo(2, idx), 'b', 'LineWidth', 1.5, 'DisplayName', 'PoI LIMO');
    desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_radius, 'k-');
    desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_influence_radius, 'k:');
    xlabel('x [m]'); ylabel('y [m]'); legend('Location', 'bestoutside');

    subplot(1, 2, 2);
    plot(H.t(idx), H.erroB(idx), 'LineWidth', 1.5); grid on;
    xlabel('t [s]'); ylabel('||p2d - p2|| [m]'); title('Erro do Bebop (calculado, não enviado)');
end

%% Funções locais

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

function [x, y, z, psi, tstamp] = ler_pose(sub)
    p = sub.LatestMessage;
    quat = [p.Pose.Orientation.W, p.Pose.Orientation.X, p.Pose.Orientation.Y, p.Pose.Orientation.Z];
    eul = quat2eul(quat);
    x = p.Pose.Position.X;
    y = p.Pose.Position.Y;
    z = p.Pose.Position.Z;
    psi = eul(1);
    tstamp = double(p.Header.Stamp.Sec) + double(p.Header.Stamp.Nsec) * 1e-9;
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
