%% LIMO (L1) + BEBOP 2 (B1) — FORMAÇÃO FINAL
% Estrutura fixa: Bebop mantém um offset [rho_f, alpha_f, beta_f] em relação
% ao PoI do LIMO. LIMO rastreia a lemniscata (laço externo tanh + compensador
% dinâmico). Bebop segue o alvo da formação (laço externo tanh + controle
% cinemático de yaw + compensador dinâmico). Parede virtual e watchdog do
% OptiTrack protegem o Bebop, independente do controlador. Decolagem e pouso
% de emergência comandados pelo joystick (Botão A / Botão B).

clear; clc; close all;

% ========================================================================
% 1. PARÂMETROS DO SISTEMA E DA FORMAÇÃO
% ========================================================================
cfg.T = 1 / 30;                 % período do loop (30 Hz)
cfg.Tsim = 120;                  % duração da fase ativa, após a preparação [s]
cfg.takeoff_wait_s = 5;          % espera após o takeoff, antes de controlar [s]
cfg.preparation_time_s = 10;     % estabilização com o LIMO parado, após o takeoff [s]
cfg.v_max = 0.30;                % limite de velocidade linear do LIMO [m/s]
cfg.w_max = 1.20;                % limite de velocidade angular do LIMO [rad/s]
cfg.a1 = 0.10;                   % offset do PoI à frente do centro do LIMO [m]

% LIMO — laço externo (cinemático) + compensador dinâmico
cfg.kq = 0.8;                    % ganho proporcional da lei tanh
cfg.lq = 0.30;                   % saturação da lei tanh [m/s]
cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422]; % parâmetros identificados
cfg.kd_limo = 4.0;               % ganho de amortecimento do compensador dinâmico

% Formação — Bebop em relação ao PoI do LIMO
% beta_f = elevação, alpha_f = azimute, a partir do eixo X global:
%   offset_f = rho_f * [cos(beta_f)*cos(alpha_f); cos(beta_f)*sin(alpha_f); sin(beta_f)]
TRAJ = 1;                        % 0: alvo fixo cfg.p2d_teste, 1: lemniscata
cfg.p2d_teste = [0.75; 0.00; 1.00]; % alvo fixo do Bebop quando TRAJ=0 [m]
rho_f = 1.5;                     % distância LIMO-Bebop [m]
alpha_f = 0;                     % azimute [rad]
beta_f = pi / 3;                 % elevação [rad] (60°; 90° seria a singularidade)
offset_f = [rho_f * cos(beta_f) * cos(alpha_f);
            rho_f * cos(beta_f) * sin(alpha_f);
            rho_f * sin(beta_f)];

% Bebop — laço externo, yaw, compensador dinâmico
Kp_B = diag([1.0, 1.0, 1.2]);    % ganho de posição do laço externo (dentro do tanh)
Ls_B = diag([0.6, 0.6, 0.6]);    % saturação da correção de posição [m/s]
KD_B = diag([2.5, 2.5, 2.0, 5.0]); % ganho de amortecimento do compensador dinâmico
cfg.yaw_d_B = 0.0;               % yaw desejado [rad], 0 = alinhado ao eixo X global
cfg.k_yaw_B = 1.0;               % ganho do controle cinemático de yaw [1/s]
cfg.wd_B_max = 0.6;              % saturação da taxa de yaw desejada [rad/s]
f1 = diag([0.8417, 0.8354, 3.966, 9.8524]); % ganho de entrada do modelo dinâmico
f2 = diag([0.18227, 0.17095, 4.001, 4.7295]); % amortecimento/arrasto do modelo
cmdB_max = [0.5; 0.5; 0.3; 0.5]; % saturação final do comando [m/s, m/s, m/s, rad/s]

% Obstáculo — desvio em espaço nulo (prioridade sobre a lemniscata)
cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence_radius = 0.25;
cfg.use_obstacle_avoidance = true;
cfg.obstacle_potential_gain = 0.80;
cfg.obstacle_potential_exponent = 4;
cfg.obstacle_potential_shape_a = [];
cfg.obstacle_potential_shape_b = [];
cfg.obstacle_potential_vmax = 0.80;

% Segurança — parede virtual (por direção) e watchdog do OptiTrack
cfg.bebop_limite_x_pos = 1.8;
cfg.bebop_limite_x_neg = -1.8;
cfg.bebop_limite_y_pos = 1.8;
cfg.bebop_limite_y_neg = -1.8;
cfg.bebop_limite_z_pos = 1.8;
cfg.optitrack_timeout_s = 0.5;

% Auditoria
cfg.audit_enabled = true;
cfg.audit_period = 1.0;
cfg.audit_dir = fullfile('results', 'limo_bebop_final');

% Joystick (mapeamento padrão Xbox)
BOTAO_A = 1; % decolar e iniciar a formação
BOTAO_B = 2; % parar e pousar (emergência)

% ========================================================================
% 2. INICIALIZAÇÃO ROS E JOYSTICK
% ========================================================================
audit_fid = -1;
if cfg.audit_enabled
    if ~exist(cfg.audit_dir, 'dir'), mkdir(cfg.audit_dir); end
    audit_stamp = datestr(now, 'yyyymmdd_HHMMSS');
    audit_file = fullfile(cfg.audit_dir, ['audit_', audit_stamp, '.txt']);
    audit_fid = fopen(audit_file, 'w');
    if audit_fid < 0
        error('Não foi possível criar o arquivo de auditoria: %s', audit_file);
    end
    fprintf(audit_fid, '=== AUDITORIA LIMO-BEBOP (FINAL) ===\nInício: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(audit_fid, 'Trajetória: %d\n', TRAJ);
    fprintf(audit_fid, 'Formação: rho_f=%.3f m, alpha_f=%.1f°, beta_f=%.1f°, offset=[%+.3f %+.3f %+.3f] m\n', ...
        rho_f, rad2deg(alpha_f), rad2deg(beta_f), offset_f);
    fprintf('Auditoria: %s\n', audit_file);
end

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
receive(pose_B, 10);

fprintf('\n=== SISTEMA PRONTO ===\n');
fprintf('Botão %d: decolar e iniciar a formação\n', BOTAO_A);
fprintf('Botão %d: parar e pousar (emergência)\n\n', BOTAO_B);

% ========================================================================
% 3. VARIÁVEIS DE ESTADO E HISTÓRICO
% ========================================================================
voando = false;
emergencia = false;
primeiro_ciclo_voo = true;
t0 = tic;
t_ant = 0;

[x1, y1, z1, psi1, ts1] = ler_pose(pose_L); %#ok<ASGLU>
[x2, y2, z2, psi2, ts2] = ler_pose(pose_B);

last_ts_L = ts1; last_update_L = tic;
last_ts_B = ts2; last_update_B = tic;

v_limo_state = [0; 0];
vd_B_ant = [0; 0; 0; 0];
poseB_ant = [x2; y2; z2];
poseB_psi_ant = psi2;
em_preparacao_ant = true;
vel_poi_ff = [0; 0];

kf = 0;
H.t = []; H.poi = []; H.ref = []; H.p2 = []; H.p2d = [];
H.erroB = []; H.cmdL = []; H.cmdB = []; H.dobs = []; H.satB = [];

% ========================================================================
% 4. LOOP PRINCIPAL DE CONTROLE
% ========================================================================
try
    while true
        tloop = tic;

        % --------------------------------------------------------
        % 4.1. Máquina de estados (botões do joystick)
        % --------------------------------------------------------
        btns = button(J);
        if numel(btns) >= BOTAO_B && btns(BOTAO_B)
            fprintf('Parada solicitada: Botão %d. Pousando...\n', BOTAO_B);
            emergencia = true;
            break;
        end

        if numel(btns) >= BOTAO_A && btns(BOTAO_A) && ~voando
            fprintf('Decolando o Bebop...\n');
            send(pub_TO, msg_TO);
            fprintf('Takeoff enviado. Aguardando %.1f s.\n', cfg.takeoff_wait_s);
            pause(cfg.takeoff_wait_s);

            [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
            poseB_ant = [x2; y2; z2];
            poseB_psi_ant = psi2;
            last_ts_B = ts2; last_update_B = tic;
            [x1, y1, z1, psi1, ts1] = ler_pose(pose_L); %#ok<ASGLU>
            last_ts_L = ts1; last_update_L = tic;

            v_limo_state = [0; 0];
            vd_B_ant = [0; 0; 0; 0];
            em_preparacao_ant = true;
            primeiro_ciclo_voo = true;

            t0 = tic;
            t_ant = 0;
            voando = true;
            fprintf('Iniciando preparação (%.1f s com o LIMO parado)...\n', cfg.preparation_time_s);
        end

        if ~voando
            pause(cfg.T);
            continue;
        end

        t = toc(t0);
        if primeiro_ciclo_voo
            dt = cfg.T;
        else
            dt = max(t - t_ant, 1e-3);
        end

        % --------------------------------------------------------
        % 4.2. Leitura de sensores e watchdog do OptiTrack
        % --------------------------------------------------------
        [x1, y1, z1, psi1, ts1] = ler_pose(pose_L);
        [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
        if ts1 > last_ts_L, last_ts_L = ts1; last_update_L = tic; end
        if ts2 > last_ts_B, last_ts_B = ts2; last_update_B = tic; end
        if toc(last_update_L) > cfg.optitrack_timeout_s || toc(last_update_B) > cfg.optitrack_timeout_s
            fprintf('OptiTrack perdido por mais de %.1f s. Pousando...\n', cfg.optitrack_timeout_s);
            emergencia = true;
            break;
        end

        poi = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];
        em_preparacao = t < cfg.preparation_time_s;
        t_traj = max(0, t - cfg.preparation_time_s);

        % --------------------------------------------------------
        % 4.3. Parede virtual (geofencing) — independente do controlador
        % --------------------------------------------------------
        p2 = [x2; y2; z2];
        if p2(1) > cfg.bebop_limite_x_pos || p2(1) < cfg.bebop_limite_x_neg || ...
                p2(2) > cfg.bebop_limite_y_pos || p2(2) < cfg.bebop_limite_y_neg || ...
                p2(3) > cfg.bebop_limite_z_pos
            fprintf('PAREDE VIRTUAL: Bebop fora dos limites (%.2f,%.2f,%.2f). Pousando...\n', p2(1), p2(2), p2(3));
            emergencia = true;
            break;
        end

        % --------------------------------------------------------
        % 4.4. Referência: LIMO (lemniscata) e alvo de formação do Bebop
        % --------------------------------------------------------
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

        if em_preparacao || TRAJ == 1
            p2d = [poi(1); poi(2); z1] + offset_f;
        else
            p2d = cfg.p2d_teste;
        end

        % --------------------------------------------------------
        % 4.5. Cinemática e dinâmica do Bebop (tanh + yaw + compensador)
        % --------------------------------------------------------
        vel_poi_world = vel_poi_ff;
        if TRAJ == 0, vel_poi_world = [0; 0]; end
        dx2 = [vel_poi_world; 0] + Ls_B * tanh(Ls_B \ (Kp_B * (p2d - p2)));

        A2inv = [cos(psi2), sin(psi2), 0; -sin(psi2), cos(psi2), 0; 0, 0, 1];
        velWB = (p2 - poseB_ant) / dt;
        psidot2 = wrap_pi(psi2 - poseB_psi_ant) / dt;
        vB_meas = [A2inv * velWB; psidot2];

        w_d_B = cfg.k_yaw_B * wrap_pi(cfg.yaw_d_B - psi2); % controle cinemático de yaw
        vd_B = [A2inv * dx2; saturar(w_d_B, cfg.wd_B_max)];

        transicao_prep_formacao = em_preparacao_ant && ~em_preparacao;
        if primeiro_ciclo_voo || transicao_prep_formacao
            dvd_B = zeros(4, 1);
        else
            dvd_B = (vd_B - vd_B_ant) / dt;
        end
        cmdB_raw = f1 \ (dvd_B + KD_B * (vd_B - vB_meas) + f2 * vB_meas);
        cmdB = max(min(cmdB_raw, cmdB_max), -cmdB_max);

        % --------------------------------------------------------
        % 4.6. Envio de comandos
        % --------------------------------------------------------
        msg_L.Linear.X = cmdL(1);
        msg_L.Linear.Y = 0;
        msg_L.Linear.Z = 0;
        msg_L.Angular.Z = cmdL(2);
        send(pub_L, msg_L);

        msg_B.Linear.X = cmdB(1);
        msg_B.Linear.Y = cmdB(2);
        msg_B.Linear.Z = cmdB(3);
        msg_B.Angular.Z = cmdB(4);
        send(pub_B, msg_B);

        % --------------------------------------------------------
        % 4.7. Auditoria, histórico e log no console
        % --------------------------------------------------------
        audit_step = max(1, round(cfg.audit_period / cfg.T));
        if audit_fid >= 0 && (primeiro_ciclo_voo || mod(kf, audit_step) == 0)
            registrar_auditoria(audit_fid, t, t_traj, em_preparacao, ...
                poi, ref_xy, psi1, p2, p2d, psi2, vd_B, vB_meas, cmdB_raw, cmdB);
        end

        kf = kf + 1;
        H.t(end + 1) = t;
        H.poi(:, end + 1) = poi;
        H.ref(:, end + 1) = ref_xy;
        H.p2(:, end + 1) = p2;
        H.p2d(:, end + 1) = p2d;
        H.erroB(:, end + 1) = p2d - p2;
        H.cmdL(:, end + 1) = cmdL;
        H.cmdB(:, end + 1) = cmdB;
        H.dobs(end + 1) = norm(poi - cfg.obstacle_center);
        H.satB(end + 1) = any(abs(cmdB_raw - cmdB) > 1e-9);

        if mod(kf, 30) == 0
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

        poseB_ant = p2;
        poseB_psi_ant = psi2;
        vd_B_ant = vd_B;
        t_ant = t;
        em_preparacao_ant = em_preparacao;
        primeiro_ciclo_voo = false;

        if t >= cfg.Tsim + cfg.preparation_time_s
            fprintf('Tempo de missão concluído. Pousando...\n');
            emergencia = true;
            break;
        end

        pause(max(0, cfg.T - toc(tloop)));
    end
catch ME
    fprintf(2, 'ERRO no loop: %s\n', ME.message);
    emergencia = true;
end

% ========================================================================
% 5. PROTOCOLO DE POUSO
% ========================================================================
if emergencia
    fprintf('=== EXECUTANDO PROTOCOLO DE PARADA E POUSO ===\n');

    msg_L.Linear.X = 0; msg_L.Linear.Y = 0; msg_L.Linear.Z = 0; msg_L.Angular.Z = 0;
    send(pub_L, msg_L);

    msg_B.Linear.X = 0; msg_B.Linear.Y = 0; msg_B.Linear.Z = 0; msg_B.Angular.Z = 0;
    send(pub_B, msg_B);
    pause(0.5);

    fprintf('Pousando o Bebop...\n');
    for i = 1:3
        send(pub_LD, msg_LD);
        pause(0.5);
    end
    pause(2.0);
end

rosshutdown;
fprintf('Conexão ROS encerrada.\n');

% ========================================================================
% 6. RESULTADOS
% ========================================================================
if audit_fid >= 0
    if kf > 0
        erro_norma = vecnorm(H.erroB, 2, 1);
        fprintf(audit_fid, '=== RESUMO ===\nAmostras: %d\n', kf);
        fprintf(audit_fid, 'Erro RMS/máximo/final do Bebop [m]: %.4f / %.4f / %.4f\n', ...
            sqrt(mean(erro_norma.^2)), max(erro_norma), erro_norma(end));
        fprintf(audit_fid, 'Distância mínima LIMO-obstáculo [m]: %.4f\n', min(H.dobs));
        fprintf(audit_fid, 'Amostras com saturação do Bebop: %d de %d\n', nnz(H.satB), kf);
    end
    fprintf(audit_fid, 'Fim: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fclose(audit_fid);
    fprintf('Auditoria salva em %s\n', audit_file);
end

if kf > 1
    figure('Name', 'LIMO-Bebop (final): trajetórias XY', 'Color', 'w');
    hold on; axis equal; grid on;
    plot(H.ref(1, :), H.ref(2, :), 'k--', 'DisplayName', 'Lemniscata desejada');
    plot(H.poi(1, :), H.poi(2, :), 'b', 'LineWidth', 1.5, 'DisplayName', 'PoI LIMO');
    plot(H.p2(1, :), H.p2(2, :), 'r', 'LineWidth', 1.2, 'DisplayName', 'Bebop');
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
    % Lemniscata de Bernoulli (docs/equacoes_controle.md secao 3):
    %   xd(t) = 0.75*sin(2*pi*t/40),  yd(t) = 0.75*sin(4*pi*t/40)
    % ref_xy_dot e a derivada analitica (feedforward exato, sem diferenca finita).
    phase_x = 2 * pi * t / 40;
    phase_y = 4 * pi * t / 40;
    ref_xy = [0.75 * sin(phase_x); 0.75 * sin(phase_y)];
    ref_xy_dot = [0.75 * (2 * pi / 40) * cos(phase_x);
                  0.75 * (4 * pi / 40) * cos(phase_y)];
end

function [v_d, ref_xy, vel_poi] = limo_reference_controller(t, poi, psi, traj, cfg)
    % Laço externo do LIMO (docs/equacoes_controle.md secoes 1 e 2):
    %   err_xy = ref_xy - poi
    %   vel_poi = ref_xy_dot + lq*tanh((kq/lq)*err_xy)              (lei tanh, secao 2)
    %   A1inv = [cos(psi) sin(psi); -sin(psi)/a1 cos(psi)/a1]
    %   v_d = saturar(A1inv*vel_poi, v_max, w_max)                  (cinematica inversa, secao 1)
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
    % Compensador dinamico do LIMO (docs/equacoes_controle.md secao 8,
    % aplicacao especifica da Eq. 4.44 do livro-texto, secao 7):
    %   Y1 = [u 0 w^2 0 0 0; 0 w 0 u u*w w]
    %   u_control = Y1*theta_limo + kd_limo*(v_d - v)
    %   M1*v_dot = u_control - C1*v
    %   v <- saturar(v + T*v_dot, v_max, w_max)
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
    % Espaco nulo / NSB (docs/equacoes_controle.md secao 4, Eq. 5.13 do
    % livro-texto): a evasao do obstaculo tem prioridade maxima; o
    % rastreamento da trajetoria (vel_xy) so atua na direcao que NAO
    % interfere na evasao (so a componente radial e removida):
    %   task_dir = grad / norm(grad)
    %   vel_poi <- grad + (I - task_dir*task_dir') * vel_poi
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
    % Potencial repulsivo exponencial (docs/equacoes_controle.md secao 4):
    %   U(dx,dy) = eta * exp(-((dx/a)^n + (dy/b)^n))
    % grad = gradiente de U em relacao a offset=[dx;dy], saturado em
    % obstacle_potential_vmax. Perto do raio fisico (clearance<=0), o
    % gradiente satura na direcao radial em vez de explodir.
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

function registrar_auditoria(fid, t, t_traj, em_preparacao, poi, ref_xy, psi1, ...
        p2, p2d, psi2, vd_B, vB_meas, cmdB_raw, cmdB)
    fprintf(fid, '---- t=%.3fs (traj=%.3fs, prep=%d) ----\n', t, t_traj, em_preparacao);
    fprintf(fid, 'LIMO: PoI=(%+.3f,%+.3f) ref=(%+.3f,%+.3f) yaw=%+.1f°\n', ...
        poi(1), poi(2), ref_xy(1), ref_xy(2), rad2deg(psi1));
    fprintf(fid, 'Bebop: p2=(%+.3f,%+.3f,%+.3f) alvo=(%+.3f,%+.3f,%+.3f) erro=(%+.3f,%+.3f,%+.3f) yaw=%+.1f°\n', ...
        p2, p2d, p2d - p2, rad2deg(psi2));
    fprintf(fid, 'vd_B=(%+.3f,%+.3f,%+.3f,%+.3f) vB_meas=(%+.3f,%+.3f,%+.3f,%+.3f)\n', vd_B, vB_meas);
    fprintf(fid, 'cmdB_raw=(%+.3f,%+.3f,%+.3f,%+.3f) cmdB=(%+.3f,%+.3f,%+.3f,%+.3f)\n\n', cmdB_raw, cmdB);
end

%% Funções auxiliares — utilitários

function y = saturar(u, umax)
    % saturar(u,umax) = max(min(u,umax),-umax) — operador "saturar(.)" usado
    % em todas as secoes de docs/equacoes_controle.md.
    y = max(min(u, umax), -umax);
end

function ang = wrap_pi(ang)
    % wrap_pi(.) traz o angulo para [-pi,pi] — operador "wrap_pi(.)" usado no
    % erro/derivada de guinada (docs/equacoes_controle.md secao 6).
    ang = atan2(sin(ang), cos(ang));
end

function desenhar_circulo(xc, yc, r, estilo)
    th = linspace(0, 2 * pi, 100);
    plot(xc + r * cos(th), yc + r * sin(th), estilo, 'HandleVisibility', 'off');
end
