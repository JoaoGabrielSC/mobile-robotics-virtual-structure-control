%% main.m - Controlador da formação virtual LIMO + Crazyflie
% Robótica Móvel 2026/1 - LAB-AIR
%
% Implementa o paradigma de estrutura virtual (Sarcinelli-Filho & Carelli,
% 2023, "Control of Ground and Aerial Robots", Cap. 5.4-5.6): a formação
% é tratada como um único robô virtual, descrito pelo vetor de variáveis
% de cluster q=[xf;yf;zf;ρf;αf;βf], e uma lei de controle única no espaço
% de cluster gera, via Jacobiano inverso, as referências de velocidade
% para o LIMO e o Crazyflie SIMULTANEAMENTE. Cada robô mantém seu próprio
% laço interno-externo (inner-outer), incluindo compensador dinâmico
% individual, exatamente como validado isoladamente em test_limo.m e
% test_crazyflie.m — este script NÃO reimplementa esses controladores do
% zero, apenas os alimenta com a referência vinda da camada de formação.
%
% Rode test_limo.m e test_crazyflie.m ANTES deste script, para validar
% cada robô isoladamente.
%
% ===== Convenção geométrica da formação (enunciado do trabalho) =====
%  q = [xf; yf; zf; ρf; αf; βf], com:
%   xf,yf,zf = posição do ponto de interesse do LIMO (zf ≡ 0, robô no chão)
%   ρf   = distância 3D entre o PoI do LIMO e o Crazyflie
%   αf   = azimute (ângulo no plano XY do MUNDO, a partir do eixo Xw)
%   βf   = elevação (ângulo entre a reta LIMO-drone e o plano XY do mundo)
%  Transformação inversa (robô 2 = drone), conforme a figura do enunciado:
%   x2 = xf + ρf·cos(βf)·cos(αf)
%   y2 = yf + ρf·cos(βf)·sin(αf)
%   z2 = zf + ρf·sin(βf)
%  Esta é a mesma parametrização esférica de (5.5a)-(5.5b) do livro
%  (Cap. 5.4.2), apenas com os papéis de αf/βf trocados para casar com a
%  figura do enunciado (no livro α=elevação, β=azimute; aqui o inverso).
%  IMPORTANTE: as fórmulas de (5.5a)/(5.5b) do livro não dependem da
%  orientação ψ1 do robô 1 (são definidas no referencial do MUNDO, não do
%  corpo do robô) — replicado aqui deliberadamente.
%
% ===== Arquitetura do laço de controle (Fig. 5.12 do livro) =====
%  1) q(t) = f(x1,x2)                    — pose real dos dois robôs
%  2) q̃ = qd(t) - q(t)
%  3) q̇r = q̇d(t) + L·tanh(L⁻¹·K·q̃)      — eq. (5.7), espaço de cluster
%  4) [ẋ1r; ẋ2r] = J⁻¹(q)·q̇r             — eq. (5.6a), Jacobiano de movimento
%  5) Espaço nulo: desvio de obstáculo tem prioridade sobre ẋ1r (LIMO),
%     eq. (5.9): ẋ1r_final = ẋ1r_obstáculo + (I-J_o†J_o)·ẋ1r
%  6) Cada robô: cinemática inversa (A_i⁻¹) + compensador dinâmico próprio
%     (inner loop) → cmd_vel de cada robô.
%
% ===== Suposições assumidas (a confirmar com o professor) =====
%  - f1,f2 do Crazyflie: mesma ressalva de test_crazyflie.m (parâmetros
%    dados no enunciado, possivelmente identificados para outra
%    plataforma).
%  - Mapeamento atitude direta phi/theta do Crazyflie: mesma ressalva de
%    test_crazyflie.m — validar sinais em bancada antes de voar.
%  - zf do LIMO tratado como 0 por definição (conforme a própria figura
%    do enunciado), não lido do OptiTrack, para não introduzir um erro
%    de cluster impossível de corrigir (o LIMO não atua em z).

clear;
clc;
close all;

%% ========================================================================
%  CONFIGURAÇÃO — edite aqui antes de rodar
%  ========================================================================

cfg.t_final = 80;                % duração da missão (s); 80 = 2×40 s
cfg.T = 1 / 30;                  % período de amostragem (30 Hz, enunciado)
cfg.pose_timeout = 30;

% --- Rede ROS -------------------------------------------------------------
cfg.ros_master_host = '192.168.0.100';
cfg.ros_master_port = 11311;
cfg.limo_namespace = 'L1';
cfg.cf_namespace = 'cf7';
cfg.pose_topic_prefix = '/natnet_ros';

% --- LIMO: cinemática e limites --------------------------------------------
cfg.a1 = 0.10;                   % offset do PoI no eixo X do LIMO (m), enunciado
cfg.v_max = 0.30;
cfg.w_max = 1.20;
cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];  % enunciado
cfg.kd_limo = 4.0;

% --- Crazyflie: limites físicos (enunciado) --------------------------------
cfg.phi_max = deg2rad(5);
cfg.theta_max = deg2rad(5);
cfg.vz_max = 1.0;
cfg.psidot_max = 100.0;
cfg.roll_sign = 1;                % ver aviso de segurança no cabeçalho
cfg.pitch_sign = 1;
cfg.f1_cf = diag([0.8417, 0.8354, 3.9660, 9.8524]);   % enunciado
cfg.f2_cf = diag([0.18227, 0.17095, 4.0010, 4.7295]);
cfg.kd_cf = diag([2.0, 2.0, 1.8, 5.0]);

% --- Formação: forma desejada (constante, enunciado) -----------------------
cfg.rho_f_d = 1.5;                % m
cfg.alpha_f_d = 0.0;              % rad (azimute)
cfg.beta_f_d = deg2rad(90.0);     % rad (elevação; 90° = drone acima do LIMO)

% --- Formação: ganhos do laço no espaço de cluster (eq. 5.7) ---------------
% q = [xf;yf;zf;ρf;αf;βf]
cfg.K_q = [1.0; 1.0; 1.0;   1.0; 1.0; 1.0];   % ganho proporcional
cfg.L_q = [0.3; 0.3; 0.3;   0.3; 0.5; 0.5];   % saturação (m/s ou rad/s)

% --- Obstáculo e campo potencial (só afeta o LIMO — drone voa acima) -------
cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence_radius = 0.50;
cfg.use_obstacle_avoidance = true;
cfg.obstacle_potential_gain = 0.80;
cfg.obstacle_potential_exponent = 2;
cfg.obstacle_potential_vmax = 0.80;

% --- Sequenciamento de voo --------------------------------------------------
cfg.takeoff_settle_time = 3.0;
cfg.land_settle_time = 3.0;

% --- Joystick ---------------------------------------------------------------
cfg.joystick_stop_button = 1;     % botão -> pouso do drone + zero no LIMO
cfg.joystick_kill_button = 2;     % botão -> KILL do drone + zero no LIMO

% --- Resultados --------------------------------------------------------------
cfg.save_results = true;
cfg.results_dir = fullfile('results', 'main');
cfg.save_gif = true;
cfg.gif_fps = 10;
cfg.gif_frame_step = 3;

%% ========================================================================
%  Fim da configuração — não é necessário editar abaixo desta linha
%  ========================================================================

fprintf('=== Formação virtual LIMO + Crazyflie ===\n');
fprintf('Botão %d: pouso controlado. Botão %d: KILL (corte de motores).\n', ...
    cfg.joystick_stop_button, cfg.joystick_kill_button);
fprintf(['[AVISO] Este script faz o Crazyflie decolar próximo ao LIMO em ', ...
    'movimento. Confirme área livre, hélices e sinais phi/theta validados ', ...
    'em bancada (ver test_crazyflie.m).\n']);

%% ROS
rosshutdown;
master_uri = sprintf('http://%s:%d', cfg.ros_master_host, cfg.ros_master_port);
fprintf('Conectando ao ROS master: %s\n', master_uri);
rosinit(master_uri);

pub_cmdvel_limo = rospublisher(sprintf('/%s/cmd_vel', cfg.limo_namespace), 'geometry_msgs/Twist');
msg_cmdvel_limo = rosmessage(pub_cmdvel_limo);

pub_cmdvel_cf = rospublisher(sprintf('/%s/cmd_vel', cfg.cf_namespace), 'geometry_msgs/Twist');
msg_cmdvel_cf = rosmessage(pub_cmdvel_cf);

sub_pose_limo = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.limo_namespace), 'geometry_msgs/PoseStamped');
sub_pose_cf = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.cf_namespace), 'geometry_msgs/PoseStamped');

takeoff_client = rossvcclient(sprintf('/%s/takeoff', cfg.cf_namespace), 'std_srvs/Trigger');
land_client = rossvcclient(sprintf('/%s/land', cfg.cf_namespace), 'std_srvs/Trigger');
kill_client = rossvcclient(sprintf('/%s/kill', cfg.cf_namespace), 'std_srvs/Trigger');

%% Aguardar pose dos dois robôs
topic_limo = sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.limo_namespace);
topic_cf = sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.cf_namespace);

fprintf('Aguardando pose do LIMO em %s...\n', topic_limo);
[pos_limo, yaw_limo, ok_limo, info_limo] = wait_for_pose(sub_pose_limo, topic_limo, cfg.pose_timeout);
fprintf('Aguardando pose do Crazyflie em %s...\n', topic_cf);
[pos_cf, yaw_cf, ok_cf, info_cf] = wait_for_pose(sub_pose_cf, topic_cf, cfg.pose_timeout);

if ~ok_limo || ~ok_cf
    rosshutdown;
    error(['Falha ao obter pose inicial.\nLIMO ok=%d (%s)\nCrazyflie ok=%d (%s)'], ...
        ok_limo, info_limo, ok_cf, info_cf);
end
fprintf('LIMO: x=%.3f y=%.3f yaw=%.1f deg | Crazyflie: x=%.3f y=%.3f z=%.3f yaw=%.1f deg\n', ...
    pos_limo(1), pos_limo(2), rad2deg(yaw_limo), pos_cf(1), pos_cf(2), pos_cf(3), rad2deg(yaw_cf));

J = JoyControl;
fprintf('Joystick conectado.\n');
fprintf('Missão: lemniscata por %.0f s, formação ρf=%.2fm αf=%.0f° βf=%.0f°.\n', ...
    cfg.t_final, cfg.rho_f_d, rad2deg(cfg.alpha_f_d), rad2deg(cfg.beta_f_d));
input('Pressione Enter para decolar e iniciar (Ctrl+C para cancelar)...', 's');

%% Decolagem do Crazyflie (o LIMO não decola; permanece parado até então)
send_limo_cmd(msg_cmdvel_limo, pub_cmdvel_limo, 0.0, 0.0);
fprintf('Enviando takeoff ao Crazyflie...\n');
took_off = call_trigger_service(takeoff_client, 'takeoff');
if ~took_off
    rosshutdown;
    error('Falha no serviço de takeoff. Abortando.');
end
pause(cfg.takeoff_settle_time);

%% Loop principal
t0 = tic;
running = true;
emergency_kill = false;
log_counter = 0;

hist_t = [];
hist_limo = [];       % [x1;y1;psi1]
hist_cf = [];          % [x2;y2;z2;psi2]
hist_q = [];            % [xf;yf;zf;rho;alpha;beta]
hist_qd = [];
hist_err_q = [];

pose_prev_limo = [];
v_d_prev_limo = [0.0; 0.0];
pose_prev_cf = [];
v_d_prev_cf = [0.0; 0.0; 0.0; 0.0];

try
    while running
        loop_start = tic;
        t = toc(t0);

        mRead(J);
        Digital = J.pDigital;
        if is_stop_pressed(Digital, cfg.joystick_kill_button)
            fprintf('KILL solicitado pelo joystick.\n');
            emergency_kill = true;
            break;
        end
        if is_stop_pressed(Digital, cfg.joystick_stop_button)
            fprintf('Pouso solicitado pelo joystick.\n');
            break;
        end

        [pos_limo, yaw_limo, ok_limo] = read_pose(sub_pose_limo);
        [pos_cf, yaw_cf, ok_cf] = read_pose(sub_pose_cf);
        if ~ok_limo || ~ok_cf
            warning('Pose indisponível — mantendo comandos neutros.');
            send_limo_cmd(msg_cmdvel_limo, pub_cmdvel_limo, 0.0, 0.0);
            send_cf_attitude_cmd(msg_cmdvel_cf, pub_cmdvel_cf, 0.0, 0.0, 0.0, 0.0);
            pause(cfg.T);
            continue;
        end

        x1 = pos_limo(1); y1 = pos_limo(2); psi1 = yaw_limo;
        poi1 = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];   % PoI do LIMO (xf,yf)
        x2 = pos_cf(1); y2 = pos_cf(2); z2 = pos_cf(3); psi2 = yaw_cf;

        %% 1) Estado atual da formação: q = f(x)
        q = formation_forward_transform(poi1, [x2; y2; z2]);

        %% 2)-3) Referência e lei de controle no espaço de cluster (eq. 5.7)
        [qd, qd_dot] = formation_reference(t, cfg);
        q_tilde = qd - q;
        q_tilde(5) = wrapToPi(q_tilde(5));   % erro de azimute (αf) — ângulo
        q_dot_r = qd_dot + cfg.L_q .* tanh((cfg.K_q ./ cfg.L_q) .* q_tilde);

        %% 4) Jacobiano de movimento: [ẋ1r;ẋ2r] = J^-1(q)*q̇r (eq. 5.6a)
        x_dot_r = formation_jacobian_inv(q) * q_dot_r;
        x1r_xy = x_dot_r(1:2);     % velocidade desejada do PoI do LIMO (mundo)
        x2r = x_dot_r(4:6);        % velocidade desejada do Crazyflie (mundo)

        %% 5) Espaço nulo: obstáculo tem prioridade sobre a tarefa de formação (LIMO)
        if cfg.use_obstacle_avoidance
            x1r_xy = apply_obstacle_null_space_xy(x1r_xy, poi1, cfg);
        end

        %% 6a) LIMO: cinemática inversa + compensador dinâmico (reaproveitado)
        A1_inv = [cos(psi1), sin(psi1); -sin(psi1) / cfg.a1, cos(psi1) / cfg.a1];
        vd_limo = A1_inv * x1r_xy;
        vd_limo = [clamp_scalar(vd_limo(1), cfg.v_max); clamp_scalar(vd_limo(2), cfg.w_max)];

        [v_meas_limo, pose_prev_limo] = estimate_chassis_velocity([x1; y1; psi1], pose_prev_limo, cfg.T);
        v_dot_d_limo = (vd_limo - v_d_prev_limo) / cfg.T;
        v_r_limo = limo_inner_loop(vd_limo, v_dot_d_limo, v_meas_limo, cfg);
        v_d_prev_limo = vd_limo;
        send_limo_cmd(msg_cmdvel_limo, pub_cmdvel_limo, v_r_limo(1), v_r_limo(2));

        %% 6b) Crazyflie: cinemática inversa + compensador dinâmico (reaproveitado)
        % Orientação do drone não é controlada pela formação (robô
        % omnidirecional; mesma escolha do exemplo do livro, Cap. 5.6,
        % pág. 131: "select a null yaw velocity command").
        A_cf_inv = [cos(psi2), sin(psi2), 0, 0; -sin(psi2), cos(psi2), 0, 0; ...
                    0, 0, 1, 0; 0, 0, 0, 1];
        vd_cf = A_cf_inv * [x2r; 0.0];

        [v_meas_cf, pose_prev_cf] = estimate_body_velocity([x2; y2; z2; psi2], pose_prev_cf, cfg.T);
        v_dot_d_cf = (vd_cf - v_d_prev_cf) / cfg.T;
        v_r_cf = cf_inner_loop(vd_cf, v_dot_d_cf, v_meas_cf, cfg);
        v_d_prev_cf = vd_cf;
        send_cf_attitude_cmd(msg_cmdvel_cf, pub_cmdvel_cf, v_r_cf(1), v_r_cf(2), v_r_cf(3), v_r_cf(4));

        %% Log
        hist_t(end + 1, 1) = t; %#ok<AGROW>
        hist_limo(:, end + 1) = [x1; y1; psi1]; %#ok<AGROW>
        hist_cf(:, end + 1) = [x2; y2; z2; psi2]; %#ok<AGROW>
        hist_q(:, end + 1) = q; %#ok<AGROW>
        hist_qd(:, end + 1) = qd; %#ok<AGROW>
        hist_err_q(:, end + 1) = q_tilde; %#ok<AGROW>

        log_counter = log_counter + 1;
        if mod(log_counter, 30) == 0
            fprintf(['t=%5.1fs | LIMO=(%+.3f,%+.3f) | CF=(%+.3f,%+.3f,%+.3f) | ', ...
                'ρf=%.2f(d%.2f) αf=%+.1f°(d%+.1f°) βf=%+.1f°(d%+.1f°)\n'], ...
                t, x1, y1, x2, y2, z2, q(4), qd(4), rad2deg(q(5)), rad2deg(qd(5)), ...
                rad2deg(q(6)), rad2deg(qd(6)));
        end

        if t >= cfg.t_final
            fprintf('Tempo final (%.0f s) atingido.\n', cfg.t_final);
            running = false;
        end

        elapsed = toc(loop_start);
        pause(max(0.0, cfg.T - elapsed));
    end
catch ME
    fprintf('Erro: %s\n', ME.message);
end

%% Encerramento
send_limo_cmd(msg_cmdvel_limo, pub_cmdvel_limo, 0.0, 0.0);
if emergency_kill
    fprintf('Encerrando: KILL no Crazyflie (corte imediato de motores).\n');
    call_trigger_service(kill_client, 'kill');
else
    fprintf('Encerrando: land no Crazyflie.\n');
    send_cf_attitude_cmd(msg_cmdvel_cf, pub_cmdvel_cf, 0.0, 0.0, 0.0, 0.0);
    call_trigger_service(land_client, 'land');
    pause(cfg.land_settle_time);
end
rosshutdown;

if ~isempty(hist_t)
    save_formation_results(hist_t, hist_limo, hist_cf, hist_q, hist_qd, hist_err_q, cfg);
end

fprintf('Missão de formação finalizada.\n');

%% ========================================================================
%  Funções locais — camada de formação (estrutura virtual, Cap. 5.4-5.6)
%  ========================================================================

function q = formation_forward_transform(poi1, pos2)
    % q = f(x): xf,yf = PoI do LIMO; zf ≡ 0 (definição do enunciado, não
    % lido do OptiTrack — ver nota no cabeçalho); ρf,αf,βf = geometria
    % esférica do Crazyflie relativa ao PoI do LIMO, no referencial do
    % mundo (mesma convenção de (5.5a) do livro, com αf/βf trocados).
    xf = poi1(1);
    yf = poi1(2);
    zf = 0.0;

    dx = pos2(1) - xf;
    dy = pos2(2) - yf;
    dz = pos2(3) - zf;

    rho_f = sqrt(dx^2 + dy^2 + dz^2);
    alpha_f = atan2(dy, dx);
    beta_f = atan2(dz, sqrt(dx^2 + dy^2));

    q = [xf; yf; zf; rho_f; alpha_f; beta_f];
end

function J_inv = formation_jacobian_inv(q)
    % Jacobiano de movimento ∂g/∂q (eq. 5.6a-c do livro, rederivado para
    % a convenção do enunciado: αf=azimute, βf=elevação).
    rho_f = q(4);
    alpha_f = q(5);
    beta_f = q(6);

    ca = cos(alpha_f); sa = sin(alpha_f);
    cb = cos(beta_f); sb = sin(beta_f);

    J_inv = [1, 0, 0,          0,               0,                  0; ...
             0, 1, 0,          0,               0,                  0; ...
             0, 0, 1,          0,               0,                  0; ...
             1, 0, 0,      cb * ca,   -rho_f * cb * sa,   -rho_f * sb * ca; ...
             0, 1, 0,      cb * sa,    rho_f * cb * ca,   -rho_f * sb * sa; ...
             0, 0, 1,          sb,              0,           rho_f * cb];
end

function [qd, qd_dot] = formation_reference(t, cfg)
    % qd(t): posição do cluster segue a lemniscata do enunciado; forma
    % (ρf,αf,βf) constante durante toda a missão.
    [ref_xy, ref_xy_dot] = lemniscata_reference(t);

    qd = [ref_xy; 0.0; cfg.rho_f_d; cfg.alpha_f_d; cfg.beta_f_d];
    qd_dot = [ref_xy_dot; 0.0; 0.0; 0.0; 0.0];
end

function [ref_xy, ref_xy_dot] = lemniscata_reference(t)
    phase_x = 2.0 * pi * t / 40.0;
    phase_y = 4.0 * pi * t / 40.0;

    ref_xy = [0.75 * sin(phase_x); 0.75 * sin(phase_y)];
    ref_xy_dot = [0.75 * (2.0 * pi / 40.0) * cos(phase_x); ...
                  0.75 * (4.0 * pi / 40.0) * cos(phase_y)];
end

%% ========================================================================
%  Funções locais — espaço nulo do obstáculo (Cap. 5.8, eq. 5.9, 5.20-5.22)
%  Aplicado apenas ao LIMO (o Crazyflie voa acima do obstáculo físico).
%  Reaproveitado de test_limo.m.
%  ========================================================================

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
    % Gradiente de U=η·exp(-((dx/a)^n+(dy/b)^n)) — eq. (5.20) do livro.
    distance = norm(offset);
    a = cfg.obstacle_influence_radius - cfg.obstacle_radius;
    b = a;
    n = cfg.obstacle_potential_exponent;
    clearance = distance - cfg.obstacle_radius;

    if clearance <= 0.0
        grad = (offset / distance) * cfg.obstacle_potential_vmax;
        return;
    end

    dx = offset(1); dy = offset(2);
    u = (dx / a)^n + (dy / b)^n;
    scale = cfg.obstacle_potential_gain * exp(-u) * n;
    grad = [scale * dx / (a^2); scale * dy / (b^2)];

    grad_mag = norm(grad);
    if grad_mag > cfg.obstacle_potential_vmax
        grad = grad * (cfg.obstacle_potential_vmax / grad_mag);
    end
end

%% ========================================================================
%  Funções locais — LIMO (idênticas a test_limo.m, laço interno corrigido)
%  ========================================================================

function [v_meas, pose_state] = estimate_chassis_velocity(pose_now, pose_state, T)
    if isempty(pose_state)
        v_meas = [0.0; 0.0];
        pose_state = pose_now;
        return;
    end

    psi_prev = pose_state(3);
    dx = pose_now(1) - pose_state(1);
    dy = pose_now(2) - pose_state(2);
    dpsi = wrapToPi(pose_now(3) - psi_prev);

    u_meas = (dx * cos(psi_prev) + dy * sin(psi_prev)) / T;
    w_meas = dpsi / T;

    v_meas = [u_meas; w_meas];
    pose_state = pose_now;
end

function v_r = limo_inner_loop(v_d, v_dot_d, v_meas, cfg)
    % Compensação dinâmica em cascata (eq. 4.44), idêntica à correção
    % aplicada em test_limo.m.
    theta = cfg.theta_limo;
    w_meas = v_meas(2);

    H = diag([theta(1), theta(2)]);
    C = [theta(4), -theta(3) * w_meas; theta(5) * w_meas, theta(6)];
    KD = diag([cfg.kd_limo, cfg.kd_limo]);

    v_til = v_d - v_meas;
    v_r = H * (v_dot_d + KD * v_til) + C * v_meas;

    v_r(1) = clamp_scalar(v_r(1), cfg.v_max);
    v_r(2) = clamp_scalar(v_r(2), cfg.w_max);
end

function send_limo_cmd(msg, pub, v, w)
    msg.Linear.X = v;
    msg.Linear.Y = 0.0;
    msg.Linear.Z = 0.0;
    msg.Angular.X = 0.0;
    msg.Angular.Y = 0.0;
    msg.Angular.Z = w;
    send(pub, msg);
end

%% ========================================================================
%  Funções locais — Crazyflie (idênticas a test_crazyflie.m)
%  ========================================================================

function [v_meas, pose_state] = estimate_body_velocity(pose_now, pose_state, T)
    if isempty(pose_state)
        v_meas = [0.0; 0.0; 0.0; 0.0];
        pose_state = pose_now;
        return;
    end

    psi_prev = pose_state(4);
    dx = pose_now(1) - pose_state(1);
    dy = pose_now(2) - pose_state(2);
    dz = pose_now(3) - pose_state(3);
    dpsi = wrapToPi(pose_now(4) - psi_prev);

    vx_w = dx / T;
    vy_w = dy / T;

    vx_b = vx_w * cos(psi_prev) + vy_w * sin(psi_prev);
    vy_b = -vx_w * sin(psi_prev) + vy_w * cos(psi_prev);
    vz_b = dz / T;
    psidot = dpsi / T;

    v_meas = [vx_b; vy_b; vz_b; psidot];
    pose_state = pose_now;
end

function v_r = cf_inner_loop(v_d, v_dot_d, v_meas, cfg)
    % Compensação dinâmica em cascata do quadrimotor near-hover (eq. 4.47).
    v_til = v_d - v_meas;
    v_r = cfg.f1_cf \ (v_dot_d + cfg.kd_cf * v_til + cfg.f2_cf * v_meas);

    theta_cmd = cfg.pitch_sign * v_r(1);
    phi_cmd = cfg.roll_sign * v_r(2);
    zdot_cmd = v_r(3);
    psidot_cmd = v_r(4);

    phi_cmd = clamp_scalar(phi_cmd, cfg.phi_max);
    theta_cmd = clamp_scalar(theta_cmd, cfg.theta_max);
    zdot_cmd = clamp_scalar(zdot_cmd, cfg.vz_max);
    psidot_cmd = clamp_scalar(psidot_cmd, cfg.psidot_max);

    v_r = [phi_cmd; theta_cmd; zdot_cmd; psidot_cmd];
end

function send_cf_attitude_cmd(msg, pub, phi, theta, zdot, psidot)
    msg.Linear.X = 0.0;
    msg.Linear.Y = 0.0;
    msg.Linear.Z = zdot;
    msg.Angular.X = phi;
    msg.Angular.Y = theta;
    msg.Angular.Z = psidot;
    send(pub, msg);
end

function ok = call_trigger_service(client, label)
    ok = false;
    try
        request = rosmessage(client);
        response = call(client, request, 'Timeout', 5);
        ok = response.Success;
        if ~ok
            warning('Serviço %s retornou falha: %s', label, response.Message);
        end
    catch ME
        warning('Falha ao chamar serviço %s: %s', label, ME.message);
    end
end

%% ========================================================================
%  Funções locais — utilitários comuns (ROS, pose, joystick)
%  ========================================================================

function [position, yaw, ok] = read_pose(pose_subscriber)
    ok = false;
    position = [0.0; 0.0; 0.0];
    yaw = 0.0;

    latest = pose_subscriber.LatestMessage;
    if isempty(latest)
        return;
    end
    [position, yaw, ok] = parse_pose_stamped(latest);
end

function [position, yaw, ok] = parse_pose_stamped(msg)
    ok = false;
    position = [0.0; 0.0; 0.0];
    yaw = 0.0;

    if isempty(msg)
        return;
    end

    try
        pose_latest = msg.Pose;
        quat = [pose_latest.Orientation.W, pose_latest.Orientation.X, ...
                pose_latest.Orientation.Y, pose_latest.Orientation.Z];
        eul_zyx = quat2eul(quat);
        angles = [eul_zyx(3); eul_zyx(2); eul_zyx(1)];
        yaw = angles(3);

        position = [pose_latest.Position.X; pose_latest.Position.Y; pose_latest.Position.Z];
        ok = true;
    catch
        ok = false;
    end
end

function pressed = is_stop_pressed(Digital, button_index)
    pressed = false;
    if button_index >= 1 && numel(Digital) >= button_index
        pressed = logical(Digital(button_index));
    end
end

function [position, yaw, ok, info] = wait_for_pose(sub_pose, pose_topic, timeout_sec)
    ok = false;
    position = [0.0; 0.0; 0.0];
    yaw = 0.0;
    info = '';

    pause(2.0);

    topics = list_ros_topics();
    natnet_topics = topics(contains(topics, 'natnet_ros'));
    if isempty(natnet_topics)
        info = 'Nenhum tópico /natnet_ros/* visível no master.';
    elseif ~any(strcmp(topics, pose_topic))
        info = sprintf('Tópico %s ausente. natnet disponíveis: %s', pose_topic, strjoin(natnet_topics, ', '));
    else
        info = sprintf('Tópico %s listado no master.', pose_topic);
    end

    try
        [pose_msg, ~, ~] = receive(sub_pose, timeout_sec);
        [position, yaw, ok] = parse_pose_stamped(pose_msg);
        if ok
            info = [info, ' Mensagem recebida via receive().'];
            return;
        end
        info = [info, ' receive() retornou mensagem inválida (parse_pose_stamped).'];
    catch ME
        info = [info, ' receive() falhou: ', ME.message];
    end

    deadline = tic;
    while toc(deadline) < timeout_sec
        [position, yaw, ok] = read_pose(sub_pose);
        if ok
            info = [info, ' Mensagem recebida via LatestMessage.'];
            return;
        end
        pause(0.25);
    end

    info = [info, sprintf(' Sem dados por %.0f s.', timeout_sec)];
end

function topics = list_ros_topics()
    topics = {};
    try
        topic_table = rosTopicList;
        topics = topic_table.Name;
    catch
        try
            topics = rostopic('list');
        catch
            topics = {};
        end
    end
    if ischar(topics)
        topics = {topics};
    end
    topics = topics(:);
end

function y = clamp_scalar(x, limit)
    y = min(max(x, -limit), limit);
end

%% ========================================================================
%  Funções locais — resultados
%  ========================================================================

function save_formation_results(hist_t, hist_limo, hist_cf, hist_q, hist_qd, hist_err_q, cfg)
    if ~cfg.save_results
        return;
    end

    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    out_dir = cfg.results_dir;
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    prefix = sprintf('formation_%s', stamp);
    traj_png = fullfile(out_dir, [prefix, '_traj.png']);
    q_png = fullfile(out_dir, [prefix, '_q.png']);
    err_png = fullfile(out_dir, [prefix, '_error.png']);
    gif_file = fullfile(out_dir, [prefix, '_anim.gif']);
    mat_file = fullfile(out_dir, [prefix, '.mat']);

    rms_pos = sqrt(mean(sum(hist_err_q(1:2, :).^2, 1)));
    rms_shape = sqrt(mean(hist_err_q(4, :).^2));

    fig_traj = figure('Name', 'Formação LIMO+Crazyflie - trajetória', 'Visible', 'off');
    plot3(hist_qd(1, :), hist_qd(2, :), zeros(size(hist_qd, 2), 1), 'r--', 'LineWidth', 1.2, ...
        'DisplayName', 'Referência (chão)');
    hold on;
    plot3(hist_limo(1, :), hist_limo(2, :), zeros(size(hist_limo, 2), 1), 'k-', 'LineWidth', 1.5, ...
        'DisplayName', 'LIMO');
    plot3(hist_cf(1, :), hist_cf(2, :), hist_cf(3, :), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Crazyflie');
    grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Formação virtual — trajetória (%s)', stamp));
    legend('Location', 'best');
    view(45, 25);
    print(fig_traj, traj_png, '-dpng', '-r150');

    fig_q = figure('Name', 'Variáveis de formação', 'Visible', 'off');
    subplot(3, 1, 1);
    plot(hist_t, hist_q(4, :), 'b-', hist_t, hist_qd(4, :), 'r--');
    ylabel('\rho_f [m]'); grid on; legend('Real', 'Desejado', 'Location', 'best');
    subplot(3, 1, 2);
    plot(hist_t, rad2deg(hist_q(5, :)), 'b-', hist_t, rad2deg(hist_qd(5, :)), 'r--');
    ylabel('\alpha_f [°]'); grid on;
    subplot(3, 1, 3);
    plot(hist_t, rad2deg(hist_q(6, :)), 'b-', hist_t, rad2deg(hist_qd(6, :)), 'r--');
    ylabel('\beta_f [°]'); xlabel('Tempo (s)'); grid on;
    print(fig_q, q_png, '-dpng', '-r150');

    fig_err = figure('Name', 'Erros de formação', 'Visible', 'off');
    plot(hist_t, hist_err_q(1, :), 'DisplayName', 'Erro x_f');
    hold on;
    plot(hist_t, hist_err_q(2, :), 'DisplayName', 'Erro y_f');
    plot(hist_t, hist_err_q(4, :), 'DisplayName', 'Erro \rho_f');
    plot(hist_t, rad2deg(hist_err_q(5, :)), 'DisplayName', 'Erro \alpha_f [°]');
    plot(hist_t, rad2deg(hist_err_q(6, :)), 'DisplayName', 'Erro \beta_f [°]');
    grid on;
    xlabel('Tempo (s)'); ylabel('Erro');
    legend('Location', 'best');
    title(sprintf('Erros de formação (RMS pos=%.3f m, RMS \\rho_f=%.3f m)', rms_pos, rms_shape));
    print(fig_err, err_png, '-dpng', '-r150');

    if cfg.save_gif
        save_formation_gif(hist_t, hist_limo, hist_cf, hist_qd, gif_file, cfg);
    end

    results.meta.timestamp = stamp;
    results.meta.rms_pos = rms_pos;
    results.meta.rms_shape = rms_shape;
    results.hist_t = hist_t;
    results.hist_limo = hist_limo;
    results.hist_cf = hist_cf;
    results.hist_q = hist_q;
    results.hist_qd = hist_qd;
    results.hist_err_q = hist_err_q;
    results.cfg = cfg;
    save(mat_file, '-struct', 'results');

    close(fig_traj); close(fig_q); close(fig_err);

    fprintf('Erro RMS de posição do cluster: %.3f m | Erro RMS de forma (rho_f): %.3f m\n', ...
        rms_pos, rms_shape);
    fprintf('Resultados salvos em %s\n', out_dir);
end

function save_formation_gif(hist_t, hist_limo, hist_cf, hist_qd, gif_file, cfg)
    frame_step = max(1, round(cfg.gif_frame_step));
    delay = 1.0 / max(1, cfg.gif_fps);
    idx = 1:frame_step:numel(hist_t);
    n_frames = numel(idx);

    margin = 0.3;
    all_x = [hist_qd(1, :), hist_limo(1, :), hist_cf(1, :)];
    all_y = [hist_qd(2, :), hist_limo(2, :), hist_cf(2, :)];
    x_lim = [min(all_x) - margin, max(all_x) + margin];
    y_lim = [min(all_y) - margin, max(all_y) + margin];

    fig = figure('Name', 'Formação GIF', 'Visible', 'off', 'Position', [100, 100, 720, 640], 'Color', 'w');
    ax = axes('Parent', fig);
    hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
    xlim(ax, x_lim); ylim(ax, y_lim);

    plot(ax, hist_qd(1, :), hist_qd(2, :), 'Color', [1.0, 0.6, 0.6], 'LineStyle', '--', ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');

    h_trail_limo = plot(ax, nan, nan, 'k-', 'LineWidth', 1.2);
    h_trail_cf = plot(ax, nan, nan, 'b-', 'LineWidth', 1.2);
    h_limo_now = plot(ax, nan, nan, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    h_cf_now = plot(ax, nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    h_link = plot(ax, nan, nan, 'Color', [0.4, 0.4, 0.9], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    h_title = title(ax, '', 'FontSize', 11);

    legend(ax, [h_trail_limo, h_trail_cf, h_limo_now, h_cf_now], ...
        {'Trajetória LIMO', 'Trajetória Crazyflie (topo)', 'LIMO (t)', 'Crazyflie (t)'}, ...
        'Location', 'northeast');

    fprintf('Gerando GIF (%d frames)... ', n_frames);
    for fi = 1:n_frames
        k = idx(fi);
        set(h_trail_limo, 'XData', hist_limo(1, 1:k), 'YData', hist_limo(2, 1:k));
        set(h_trail_cf, 'XData', hist_cf(1, 1:k), 'YData', hist_cf(2, 1:k));
        set(h_limo_now, 'XData', hist_limo(1, k), 'YData', hist_limo(2, k));
        set(h_cf_now, 'XData', hist_cf(1, k), 'YData', hist_cf(2, k));
        set(h_link, 'XData', [hist_limo(1, k), hist_cf(1, k)], 'YData', [hist_limo(2, k), hist_cf(2, k)]);
        set(h_title, 'String', sprintf('t = %.1f s | z_{cf} = %.2f m | frame %d/%d', ...
            hist_t(k), hist_cf(3, k), fi, n_frames));

        drawnow limitrate;
        frame = getframe(fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if fi == 1
            imwrite(imind, cm, gif_file, 'gif', 'Loopcount', inf, 'DelayTime', delay);
        else
            imwrite(imind, cm, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
        end
    end
    close(fig);
    fprintf('OK\n');
end
