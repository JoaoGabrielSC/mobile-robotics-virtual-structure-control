% limo_bebop.m — Formação LIMO + Bebop 2.
% LIMO rastreia a lemniscata pelo seu PoI (laço externo + compensador
% dinâmico). Bebop mantém a formação num offset fixo [rho_f, alpha_f, beta_f]
% acima/ao lado do LIMO (laço externo tanh + compensador dinâmico + controle
% cinemático de guinada). Parede virtual e watchdog do OptiTrack protegem o
% Bebop independente do controlador. Versão enxuta de formacao_2.m — mesma
% lógica de controle, sem o histórico de alterações e com auditoria reduzida
% ao essencial (alvo, erro, comando).
clear; clc; close all;

%% Configuração — tempo e limites
cfg.T = 1 / 30;
cfg.Tsim = 120;
cfg.takeoff_wait_s = 5;
cfg.preparation_time_s = 10;
cfg.v_max = 0.30;
cfg.w_max = 1.20;
cfg.a1 = 0.10;

%% Configuração — LIMO (laço externo + compensador dinâmico)
cfg.kq = 0.8;
cfg.lq = 0.30;
cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];
cfg.kd_limo = 4.0;

%% Configuração — formação (Bebop em relação ao LIMO)
% beta_f = elevação, alpha_f = azimute, a partir do eixo X global:
%   x2 = xf + rho_f*cos(beta_f)*cos(alpha_f)
%   y2 = yf + rho_f*cos(beta_f)*sin(alpha_f)
%   z2 = zf + rho_f*sin(beta_f)
TRAJ = 1; % 0: alvo fixo cfg.p2d_teste, 1: lemniscata
cfg.p2d_teste = [0.75; 0.00; 1.00]; % alvo fixo do teste TRAJ=0 [m]
rho_f = 1.5;
alpha_f = 0;
beta_f = pi / 3; % 60°; com isso a altura fica ~1.30 m (rho_f*sin(60°))
offset_f = [rho_f * cos(beta_f) * cos(alpha_f);
            rho_f * cos(beta_f) * sin(alpha_f);
            rho_f * sin(beta_f)];

%% Configuração — Bebop (laço externo, guinada, compensador dinâmico)
Kp_B = diag([1.0, 1.0, 1.2]);
Ls_B = diag([0.6, 0.6, 0.6]);
KD_B = diag([2.5, 2.5, 2.0, 5.0]);
cfg.yaw_d_B = 0.0;      % guinada desejada [rad], 0 = alinhado ao eixo X global
cfg.k_yaw_B = 1.0;      % ganho do controle cinemático de guinada [1/s]
cfg.wd_B_max = 0.6;     % saturação da taxa de guinada desejada [rad/s]
f1 = diag([0.8417, 0.8354, 3.966, 9.8524]);
f2 = diag([0.18227, 0.17095, 4.001, 4.7295]);
cmdB_max = [0.5; 0.5; 0.3; 0.5];

%% Configuração — obstáculo (desvio em espaço nulo, prioridade sobre a lemniscata)
cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence_radius = 0.25;
cfg.use_obstacle_avoidance = true;
cfg.obstacle_potential_gain = 0.80;
cfg.obstacle_potential_exponent = 4;
cfg.obstacle_potential_shape_a = [];
cfg.obstacle_potential_shape_b = [];
cfg.obstacle_potential_vmax = 0.80;

%% Configuração — segurança e auditoria
cfg.bebop_limite_x_pos = 1.8;  % parede virtual: limite em +x [m]
cfg.bebop_limite_x_neg = -1.8; % parede virtual: limite em -x [m]
cfg.bebop_limite_y_pos = 1.8;  % parede virtual: limite em +y [m]
cfg.bebop_limite_y_neg = -1.8; % parede virtual: limite em -y [m]
cfg.bebop_limite_z_pos = 1.8;  % parede virtual: limite em +z [m]
cfg.optitrack_timeout_s = 0.5; % watchdog de pose parada
cfg.audit_enabled = true;
cfg.audit_period = 1.0;
cfg.audit_dir = fullfile('results', 'limo_bebop');

MODO_BEBOP = 'teste'; % 'off' (Bebop virtual), 'teste' (Bebop real, sem decolagem automática) ou 'voo'
BTN_STOP = 1;
use_preparation = strcmp(MODO_BEBOP, 'voo') && cfg.preparation_time_s > 0;
preparation_time_s = use_preparation * cfg.preparation_time_s;
N = round((cfg.Tsim + preparation_time_s) / cfg.T);

audit_fid = -1;
if cfg.audit_enabled
    if ~exist(cfg.audit_dir, 'dir'), mkdir(cfg.audit_dir); end
    audit_stamp = datestr(now, 'yyyymmdd_HHMMSS');
    audit_file = fullfile(cfg.audit_dir, ['audit_', audit_stamp, '.txt']);
    audit_fid = fopen(audit_file, 'w');
    if audit_fid < 0
        error('Não foi possível criar o arquivo de auditoria: %s', audit_file);
    end
    fprintf(audit_fid, '=== AUDITORIA LIMO-BEBOP ===\nInício: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(audit_fid, 'Modo Bebop: %s | Trajetória: %d\n', MODO_BEBOP, TRAJ);
    fprintf(audit_fid, 'Formação: rho_f=%.3f m, alpha_f=%.1f°, beta_f=%.1f°, offset=[%+.3f %+.3f %+.3f] m\n', ...
        rho_f, rad2deg(alpha_f), rad2deg(beta_f), offset_f);
    fprintf(audit_fid, 'Preparação: %.1f s | Amostragem: %.2f s\n\n', preparation_time_s, cfg.audit_period);
    fprintf('Auditoria: %s\n', audit_file);
end

%% ROS e OptiTrack
rosshutdown;
rosinit('http://192.168.0.100:11311');

pub_L = rospublisher('/L1/cmd_vel', 'geometry_msgs/Twist');
msg_L = rosmessage(pub_L);
pose_L = rossubscriber('/natnet_ros/L1/pose', 'geometry_msgs/PoseStamped');

pub_B = rospublisher('/B1/cmd_vel', 'geometry_msgs/Twist');
msg_B = rosmessage(pub_B);
pub_TO = rospublisher('/B1/takeoff', 'std_msgs/Empty');
msg_TO = rosmessage(pub_TO);
pub_LD = rospublisher('/B1/land', 'std_msgs/Empty');
msg_LD = rosmessage(pub_LD);
pose_B = rossubscriber('/natnet_ros/B1/pose', 'geometry_msgs/PoseStamped');

J = vrjoystick(1);
fprintf('Aguardando poses do OptiTrack...\n');
receive(pose_L, 10);
if ~strcmp(MODO_BEBOP, 'off'), receive(pose_B, 10); end

if strcmp(MODO_BEBOP, 'voo')
    send(pub_TO, msg_TO);
    fprintf('Takeoff enviado. Aguardando %.1f s.\n', cfg.takeoff_wait_s);
    pause(cfg.takeoff_wait_s);
end

%% Estado inicial
[x1, y1, z1, psi1, ts1] = ler_pose(pose_L);
if strcmp(MODO_BEBOP, 'off')
    virtual_B.p = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1); z1] + offset_f;
    virtual_B.psi = 0;
    virtual_B.v_body = zeros(4, 1);
    x2 = virtual_B.p(1); y2 = virtual_B.p(2); z2 = virtual_B.p(3);
    psi2 = virtual_B.psi;
    ts2 = ts1;
else
    [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
end

last_ts_L = ts1; last_update_L = tic;
last_ts_B = ts2; last_update_B = tic;

v_limo_state = [0; 0];
vd_B_ant = [0; 0; 0; 0];
poseB_ant = [x2; y2; z2];
poseB_psi_ant = psi2;
t_ant = 0;
em_preparacao_ant = true;
vel_poi_ff = [0; 0];

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

if use_preparation
    fprintf('Preparando o Bebop por %.1f s com LIMO parado. Botão %d para parar.\n', preparation_time_s, BTN_STOP);
else
    fprintf('Iniciando formação. Botão %d para parar.\n', BTN_STOP);
end

%% Loop de controle
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

        % --- Leitura de sensores + watchdog do OptiTrack ---
        [x1, y1, z1, psi1, ts1] = ler_pose(pose_L);
        if strcmp(MODO_BEBOP, 'off')
            x2 = virtual_B.p(1); y2 = virtual_B.p(2); z2 = virtual_B.p(3);
            psi2 = virtual_B.psi;
            ts2 = ts1;
        else
            [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
        end
        if ts1 > last_ts_L, last_ts_L = ts1; last_update_L = tic; end
        if ts2 > last_ts_B, last_ts_B = ts2; last_update_B = tic; end
        if toc(last_update_L) > cfg.optitrack_timeout_s || ...
                (~strcmp(MODO_BEBOP, 'off') && toc(last_update_B) > cfg.optitrack_timeout_s)
            fprintf('OptiTrack perdido por mais de %.1f s. Abortando.\n', cfg.optitrack_timeout_s);
            break;
        end

        poi = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];
        em_preparacao = t < preparation_time_s;
        t_traj = max(0, t - preparation_time_s);

        % --- Malha externa: LIMO ---
        if em_preparacao
            ref_xy = poi;
            vel_poi_ff = [0; 0];
            v_limo_state = [0; 0];
            cmdL = [0; 0];
        else
            [vd_L, ref_xy, vel_poi_ff] = limo_reference_controller(t_traj, poi, psi1, TRAJ, cfg);
            v_limo_state = limo_inner_loop(vd_L, v_limo_state, cfg);
            cmdL = v_limo_state;
        end

        % --- Malha externa: Bebop (alvo + lei cinemática tanh) ---
        if em_preparacao || TRAJ == 1
            p2d = [poi(1); poi(2); z1] + offset_f;
        else
            p2d = cfg.p2d_teste;
        end
        p2 = [x2; y2; z2];

        % Parede virtual: rede de segurança independente do controlador.
        if ~strcmp(MODO_BEBOP, 'off') && ...
                (p2(1) > cfg.bebop_limite_x_pos || p2(1) < cfg.bebop_limite_x_neg || ...
                 p2(2) > cfg.bebop_limite_y_pos || p2(2) < cfg.bebop_limite_y_neg || ...
                 p2(3) > cfg.bebop_limite_z_pos)
            fprintf('PAREDE VIRTUAL: Bebop fora dos limites (%.2f,%.2f,%.2f). Abortando.\n', p2(1), p2(2), p2(3));
            break;
        end

        vel_poi_world = vel_poi_ff;
        if TRAJ == 0, vel_poi_world = [0; 0]; end
        dx2 = [vel_poi_world; 0] + Ls_B * tanh(Ls_B \ (Kp_B * (p2d - p2)));

        A2inv = [cos(psi2), sin(psi2), 0; -sin(psi2), cos(psi2), 0; 0, 0, 1];
        if strcmp(MODO_BEBOP, 'off')
            vB_meas = virtual_B.v_body;
        else
            velWB = (p2 - poseB_ant) / dt;
            psidot2 = wrap_pi(psi2 - poseB_psi_ant) / dt;
            vB_meas = [A2inv * velWB; psidot2];
        end
        w_d_B = cfg.k_yaw_B * wrap_pi(cfg.yaw_d_B - psi2); % controle cinemático de guinada
        vd_B = [A2inv * dx2; saturar(w_d_B, cfg.wd_B_max)];

        % --- Malha interna: compensador dinâmico do Bebop ---
        transicao_prep_formacao = em_preparacao_ant && ~em_preparacao;
        if k == 1 || transicao_prep_formacao
            dvd_B = zeros(4, 1);
        else
            dvd_B = (vd_B - vd_B_ant) / dt;
        end
        cmdB_raw = f1 \ (dvd_B + KD_B * (vd_B - vB_meas) + f2 * vB_meas);
        cmdB = max(min(cmdB_raw, cmdB_max), -cmdB_max);

        % --- Auditoria e envio de comandos ---
        audit_step = max(1, round(cfg.audit_period / cfg.T));
        if audit_fid >= 0 && (k == 1 || mod(k - 1, audit_step) == 0)
            registrar_auditoria(audit_fid, t, t_traj, em_preparacao, MODO_BEBOP, ...
                poi, ref_xy, psi1, p2, p2d, psi2, vd_B, vB_meas, cmdB_raw, cmdB);
        end

        msg_L.Linear.X = cmdL(1);
        msg_L.Linear.Y = 0;
        msg_L.Linear.Z = 0;
        msg_L.Angular.Z = cmdL(2);
        send(pub_L, msg_L);

        if ~strcmp(MODO_BEBOP, 'off')
            msg_B.Linear.X = cmdB(1);
            msg_B.Linear.Y = cmdB(2);
            msg_B.Linear.Z = cmdB(3);
            msg_B.Angular.Z = cmdB(4);
            send(pub_B, msg_B);
        else
            virtual_B = avancar_bebop_virtual(virtual_B, cmdB, dt, f1, f2);
        end

        % --- Histórico e log no console ---
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
        kf = k;
        poseB_ant = p2;
        poseB_psi_ant = psi2;
        vd_B_ant = vd_B;
        t_ant = t;
        em_preparacao_ant = em_preparacao;

        if mod(k, 30) == 0
            erro_xyz = p2d - p2;
            if em_preparacao
                fprintf(['Preparação t=%4.1fs | alvo=(%+.2f,%+.2f,%+.2f) erro=(%+.2f,%+.2f,%+.2f) ', ...
                    '|%.3fm| cmdB=(%+.2f,%+.2f,%+.2f,%+.2f)\n'], t, p2d(1), p2d(2), p2d(3), ...
                    erro_xyz(1), erro_xyz(2), erro_xyz(3), norm(erro_xyz), cmdB(1), cmdB(2), cmdB(3), cmdB(4));
            else
                fprintf('t=%5.1fs | PoI=(%+.2f,%+.2f) ref=(%+.2f,%+.2f) | v=%+.2f w=%+.2f\n', ...
                    t_traj, poi(1), poi(2), ref_xy(1), ref_xy(2), cmdL(1), cmdL(2));
                fprintf(['           Bebop alvo=(%+.2f,%+.2f,%+.2f) erro=(%+.2f,%+.2f,%+.2f) ', ...
                    '|%.3fm| cmdB=(%+.2f,%+.2f,%+.2f,%+.2f)\n'], p2d(1), p2d(2), p2d(3), ...
                    erro_xyz(1), erro_xyz(2), erro_xyz(3), norm(erro_xyz), cmdB(1), cmdB(2), cmdB(3), cmdB(4));
            end
        end
        pause(max(0, cfg.T - toc(tloop)));
    end
catch ME
    fprintf(2, 'ERRO no loop: %s\n', ME.message);
end

%% Encerramento
msg_L.Linear.X = 0; msg_L.Linear.Y = 0; msg_L.Linear.Z = 0; msg_L.Angular.Z = 0;
send(pub_L, msg_L);
if ~strcmp(MODO_BEBOP, 'off')
    msg_B.Linear.X = 0; msg_B.Linear.Y = 0; msg_B.Linear.Z = 0; msg_B.Angular.Z = 0;
    send(pub_B, msg_B);
end
if strcmp(MODO_BEBOP, 'voo'), send(pub_LD, msg_LD); end
pause(0.5);
rosshutdown;

%% Resultados
if audit_fid >= 0
    if kf > 0
        idx = 1:kf;
        erro_norma = vecnorm(H.erroB(:, idx), 2, 1);
        fprintf(audit_fid, '=== RESUMO ===\nAmostras: %d\n', kf);
        fprintf(audit_fid, 'Erro RMS/máximo/final do Bebop [m]: %.4f / %.4f / %.4f\n', ...
            sqrt(mean(erro_norma.^2)), max(erro_norma), erro_norma(end));
        fprintf(audit_fid, 'Distância mínima LIMO-obstáculo [m]: %.4f\n', min(H.dobs(idx)));
        fprintf(audit_fid, 'Amostras com saturação do Bebop: %d de %d\n', nnz(H.satB(idx)), kf);
    end
    fprintf(audit_fid, 'Fim: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fclose(audit_fid);
    fprintf('Auditoria salva em %s\n', audit_file);
end

if kf > 1
    idx = 1:kf;
    figure('Name', 'LIMO-Bebop: trajetórias XY', 'Color', 'w');
    hold on; axis equal; grid on;
    plot(H.ref(1, idx), H.ref(2, idx), 'k--', 'DisplayName', 'Lemniscata desejada');
    plot(H.poi(1, idx), H.poi(2, idx), 'b', 'LineWidth', 1.5, 'DisplayName', 'PoI LIMO');
    plot(H.p2(1, idx), H.p2(2, idx), 'r', 'LineWidth', 1.2, 'DisplayName', 'Bebop');
    desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_radius, 'k-');
    desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_influence_radius, 'k:');
    xlabel('x [m]'); ylabel('y [m]'); legend('Location', 'bestoutside');
end

%% Funções auxiliares — ROS / pose

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

%% Funções auxiliares — LIMO

function [ref_xy, ref_xy_dot] = lemniscata_reference(t)
    phase_x = 2 * pi * t / 40;
    phase_y = 4 * pi * t / 40;
    ref_xy = [0.75 * sin(phase_x); 0.75 * sin(phase_y)];
    ref_xy_dot = [0.75 * (2 * pi / 40) * cos(phase_x);
                  0.75 * (4 * pi / 40) * cos(phase_y)];
end

function [v_d, ref_xy, vel_poi] = limo_reference_controller(t, poi, psi, traj, cfg)
    % vel_poi é a velocidade DESEJADA do PoI no mundo (antes de A1inv) — é o
    % sinal correto para o feedforward do Bebop, sem saturação de v_max/w_max
    % (esses limites são do robô, não da referência da formação).
    if traj == 1
        [ref_xy, ref_xy_dot] = lemniscata_reference(t);
    else
        ref_xy = [0; 0];
        ref_xy_dot = [0; 0];
    end
    err_xy = ref_xy - poi;
    vel_poi = ref_xy_dot + cfg.lq * tanh((cfg.kq / cfg.lq) * err_xy);
    if cfg.use_obstacle_avoidance
        vel_poi = apply_obstacle_null_space_xy(vel_poi, poi, cfg);
    end
    A1inv = [cos(psi), sin(psi); -sin(psi) / cfg.a1, cos(psi) / cfg.a1];
    u = A1inv * vel_poi;
    v_d = [saturar(u(1), cfg.v_max); saturar(u(2), cfg.w_max)];
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

%% Funções auxiliares — Bebop / obstáculo

function vel_xy = apply_obstacle_null_space_xy(vel_xy, poi, cfg)
    offset = poi - cfg.obstacle_center;
    distance = norm(offset);
    if distance >= cfg.obstacle_influence_radius || distance <= 1e-6
        return;
    end
    grad = obstacle_repulsive_gradient(offset, cfg);
    grad_mag = norm(grad);
    if grad_mag <= 1e-9
        return;
    end
    task_dir = grad / grad_mag;
    null_projector = eye(2) - task_dir * task_dir.';
    vel_xy = grad + null_projector * vel_xy;
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

function estado = avancar_bebop_virtual(estado, u, dt, f1, f2)
    % Planta usada apenas em MODO_BEBOP='off'.
    R = [cos(estado.psi), -sin(estado.psi), 0; sin(estado.psi), cos(estado.psi), 0; 0, 0, 1];
    estado.p = estado.p + dt * R * estado.v_body(1:3);
    estado.psi = wrap_pi(estado.psi + dt * estado.v_body(4));
    estado.v_body = estado.v_body + dt * (f1 * u - f2 * estado.v_body);
end

function registrar_auditoria(fid, t, t_traj, em_preparacao, modo_bebop, poi, ref_xy, psi1, ...
        p2, p2d, psi2, vd_B, vB_meas, cmdB_raw, cmdB)
    fprintf(fid, '---- t=%.3fs (traj=%.3fs, prep=%d, modo=%s) ----\n', t, t_traj, em_preparacao, modo_bebop);
    fprintf(fid, 'LIMO: PoI=(%+.3f,%+.3f) ref=(%+.3f,%+.3f) yaw=%+.1f°\n', ...
        poi(1), poi(2), ref_xy(1), ref_xy(2), rad2deg(psi1));
    fprintf(fid, 'Bebop: p2=(%+.3f,%+.3f,%+.3f) alvo=(%+.3f,%+.3f,%+.3f) erro=(%+.3f,%+.3f,%+.3f) yaw=%+.1f°\n', ...
        p2, p2d, p2d - p2, rad2deg(psi2));
    fprintf(fid, 'vd_B=(%+.3f,%+.3f,%+.3f,%+.3f) vB_meas=(%+.3f,%+.3f,%+.3f,%+.3f)\n', vd_B, vB_meas);
    fprintf(fid, 'cmdB_raw=(%+.3f,%+.3f,%+.3f,%+.3f) cmdB=(%+.3f,%+.3f,%+.3f,%+.3f)\n\n', cmdB_raw, cmdB);
end

%% Funções auxiliares — utilitários

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
