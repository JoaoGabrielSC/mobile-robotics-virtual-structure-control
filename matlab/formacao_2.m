% Formação LIMO-Bebop em espaço de cluster (Estrutura Virtual, Cap. 5 Sarcinelli).
% Convenção deste projeto: beta = elevação, alpha = azimute (trocada em
% relação à Eq. 5.5b do livro, mantida por compatibilidade com o restante
% do código já validado em voo). Ver decisão registrada na auditoria.
clear; clc; close all;

%% Configuração
cfg.T = 1 / 30;
cfg.Tsim = 120;
cfg.takeoff_wait_s = 5;
cfg.preparation_time_s = 10;
cfg.auto_takeoff = false;          % false: decolagem manual, confirmada por joystick
cfg.wait_for_start_signal = true;
cfg.btn_start = 2;
cfg.v_max = 0.30;
cfg.w_max = 1.20;
cfg.a1 = 0.10;
cfg.kq = 0.8;
cfg.lq = 0.30;
cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];
cfg.kd_limo = 4.0;

cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence_radius = 0.50; % exigido pelo SPEC (era 0.25)
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
cfg.audit_enabled = true;
cfg.audit_period = 1.0;
cfg.audit_dir = fullfile('results', 'formacao_2');

TRAJ = 1; % 0: cluster para na origem (teste), 1: lemniscata
rho_f = 1.5;
alpha_f = 0;
beta_f = pi / 3; % 60°, singularidade em beta=90° (drone exatamente acima)

% Ganhos da lei cinemática do cluster (Eq. 5.7), shape = [rho; alpha; beta]
K_shape_diag = [1.2; 1.0; 1.2];
L_shape_diag = [0.3; 0.4; 0.4];

% Ganhos usados só na fase de preparação (Bebop convergindo até p2d)
Kp_B = diag([1.0, 1.0, 1.2]);
Ls_B = diag([0.6, 0.6, 0.6]);

% KD_B reduzido de diag(4,4,4,4): amplificava ruído de vB_meas (diferença
% finita bruta da pose) e saturava com frequência.
KD_B = diag([2.5, 2.5, 2.0, 2.5]);
f1 = diag([0.8417, 0.8354, 3.966, 9.8524]);
f2 = diag([0.18227, 0.17095, 4.001, 4.7295]);

% Saturações do Bebop calibradas por eixo (driver interpreta cmd_vel como
% fração da vel./ângulo máximo por eixo; vz satura mais rápido que xy).
cmdB_max = [0.5; 0.5; 0.3; 0.5];      % nível 2: comando final
vd_B_max = [0.5; 0.5; 0.3];           % nível 1: velocidade mundo desejada

cfg.soft_start_time_s = 8.0;
cfg.soft_start_gamma_min = 0.3;
cfg.dvd_B_filter_alpha = 0.3;
cfg.dvd_B_max = [1.0; 1.0; 0.6; 1.0];
cfg.cmdB_rate_max = [1.2; 1.2; 0.8; 1.2];
cfg.bebop_limite_xy = 1.8;
cfg.bebop_limite_z = 1.8;

MODO_BEBOP = 'teste'; % 'off', 'teste' ou 'voo'
BTN_STOP = 1;
use_preparation = ~strcmp(MODO_BEBOP, 'off') && cfg.preparation_time_s > 0;
preparation_time_s = use_preparation * cfg.preparation_time_s;
N = round((cfg.Tsim + preparation_time_s) / cfg.T);

audit_fid = -1;
if cfg.audit_enabled
    if ~exist(cfg.audit_dir, 'dir')
        mkdir(cfg.audit_dir);
    end
    audit_stamp = datestr(now, 'yyyymmdd_HHMMSS');
    audit_file = fullfile(cfg.audit_dir, ['audit_formacao_', audit_stamp, '.txt']);
    audit_fid = fopen(audit_file, 'w');
    if audit_fid < 0
        error('Não foi possível criar o arquivo de auditoria: %s', audit_file);
    end
    fprintf(audit_fid, '=== AUDITORIA DE FORMAÇÃO LIMO-BEBOP (espaço de cluster) ===\n');
    fprintf(audit_fid, 'Início: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(audit_fid, 'Modo Bebop: %s | Trajetória: %d\n', MODO_BEBOP, TRAJ);
    fprintf(audit_fid, 'Decolagem automática: %d | Espera após takeoff: %.1f s\n', ...
        cfg.auto_takeoff, cfg.auto_takeoff * cfg.takeoff_wait_s);
    fprintf(audit_fid, 'Preparação antes da trajetória: %.1f s\n', preparation_time_s);
    fprintf(audit_fid, 'Amostragem de auditoria: %.2f s\n', cfg.audit_period);
    if strcmp(MODO_BEBOP, 'off')
        fprintf(audit_fid, 'AVISO: Bebop desligado; planta virtual v_dot=f1*u-f2*v.\n');
    end
    fprintf(audit_fid, '\n');
    fprintf('Auditoria da formação: %s\n', audit_file);
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
if ~strcmp(MODO_BEBOP, 'off')
    receive(pose_B, 10);
end

% Decolagem: automática (cfg.auto_takeoff) ou manual com confirmação por
% joystick — decolagem manual replica o grupo externo que voou com sucesso.
if ~strcmp(MODO_BEBOP, 'off')
    if cfg.auto_takeoff
        send(pub_TO, msg_TO);
        fprintf('Takeoff enviado. Aguardando %.1f s.\n', cfg.takeoff_wait_s);
        pause(cfg.takeoff_wait_s);
    elseif cfg.wait_for_start_signal
        fprintf(['Decolagem manual: decole o Bebop e estabilize o hover.\n', ...
            'Botão %d confirma início, botão %d cancela.\n'], cfg.btn_start, BTN_STOP);
        prosseguir = aguardar_sinal_inicio(J, cfg.btn_start, BTN_STOP);
        if ~prosseguir
            fprintf('Início cancelado pelo joystick.\n');
            if audit_fid >= 0, fclose(audit_fid); end
            rosshutdown;
            return;
        end
    end
end

[x1, y1, z1, psi1, ts1] = ler_pose(pose_L);
if strcmp(MODO_BEBOP, 'off')
    virtual_B.p = [x1; y1; z1 + rho_f];
    virtual_B.psi = 0;
    virtual_B.v_body = zeros(4, 1);
    x2 = virtual_B.p(1); y2 = virtual_B.p(2); z2 = virtual_B.p(3);
    psi2 = virtual_B.psi;
    ts2 = ts1;
else
    [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
end

% Watchdog do OptiTrack: aborta se a pose ficar parada por > 0.5 s, para
% que vB_meas/dvd_B nunca sejam calculados a partir de uma amostra atrasada.
cfg.optitrack_timeout_s = 0.5;
last_ts_L = ts1; last_update_L = tic;
last_ts_B = ts2; last_update_B = tic;

v_limo_state = [0; 0];
vd_B_ant = [0; 0; 0; 0];
poseB_ant = [x2; y2; z2];
poseB_psi_ant = psi2;
t_ant = 0;

% cmdB_prev guarda o último comando REALMENTE enviado (pós rate-limit), o
% que torna o limitador de taxa também um anti-windup.
cmdB_prev = zeros(4, 1);
dvd_B_filt = zeros(4, 1);
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

if use_preparation
    fprintf('Preparando o Bebop por %.1f s com LIMO parado. Botão %d para parar.\n', ...
        preparation_time_s, BTN_STOP);
else
    fprintf('Iniciando formação. Botão %d para parar.\n', BTN_STOP);
end
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
        p2 = [x2; y2; z2];
        em_preparacao = t < preparation_time_s;
        t_traj = max(0, t - preparation_time_s);
        gamma = min(max(t / cfg.soft_start_time_s, cfg.soft_start_gamma_min), 1);

        %% Laço externo: LIMO (cinemático) + Bebop (cluster space)
        if em_preparacao
            ref_xy = poi;
            v_limo_state = [0; 0];
            cmdL = [0; 0];
            p2d = [poi; z1 + rho_f];
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
            x_dot(3) = 0; % LIMO é terrestre: z1_dot sempre zero

            A1inv = [cos(psi1), sin(psi1); -sin(psi1) / cfg.a1, cos(psi1) / cfg.a1];
            u = A1inv * x_dot(1:2);
            vd_L = [saturar(u(1), cfg.v_max); saturar(u(2), cfg.w_max)];
            v_limo_state = limo_inner_loop(vd_L, v_limo_state, cfg);
            cmdL = v_limo_state;

            dx2 = x_dot(4:6);
            p2d = [poi; 0] + cluster_offset(rho_f, alpha_f, beta_f);
        end

        % Parede virtual: rede de segurança independente do controlador.
        if ~strcmp(MODO_BEBOP, 'off') && ...
                (abs(p2(1)) > cfg.bebop_limite_xy || abs(p2(2)) > cfg.bebop_limite_xy || ...
                 p2(3) > cfg.bebop_limite_z)
            fprintf('PAREDE VIRTUAL: Bebop fora dos limites (%.2f,%.2f,%.2f). Abortando.\n', ...
                p2(1), p2(2), p2(3));
            break;
        end

        %% Laço interno: Bebop (compensação dinâmica)
        dx2 = [saturar(dx2(1), vd_B_max(1));
               saturar(dx2(2), vd_B_max(2));
               saturar(dx2(3), vd_B_max(3))]; % nível 1

        A2inv = [cos(psi2), sin(psi2), 0; -sin(psi2), cos(psi2), 0; 0, 0, 1];
        if strcmp(MODO_BEBOP, 'off')
            vB_meas = virtual_B.v_body;
        else
            velWB = (p2 - poseB_ant) / dt;
            psidot2 = wrap_pi(psi2 - poseB_psi_ant) / dt;
            vB_meas = [A2inv * velWB; psidot2];
        end
        vd_B = [A2inv * dx2; 0];
        KD_B_eff = gamma * KD_B;

        % dvd_B: reset na transição preparação->formação (p2d muda de
        % fórmula, evita pico de derivada) + filtro passa-baixa + saturação.
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
        cmdB_satN2 = max(min(cmdB_raw, cmdB_max), -cmdB_max); % nível 2

        % Rate limiter sobre cmdB_prev (último comando enviado) = anti-windup.
        delta_max = cfg.cmdB_rate_max * dt;
        delta_cmd = max(min(cmdB_satN2 - cmdB_prev, delta_max), -delta_max);
        cmdB = cmdB_prev + delta_cmd;
        satB_rate = any(abs(cmdB - cmdB_satN2) > 1e-9);

        audit_step = max(1, round(cfg.audit_period / cfg.T));
        if audit_fid >= 0 && (k == 1 || mod(k - 1, audit_step) == 0)
            registrar_auditoria(audit_fid, t, t_traj, em_preparacao, dt, MODO_BEBOP, TRAJ, ...
                poi, ref_xy, psi1, p2, p2d, psi2, A2inv, dx2, vB_meas, vd_B, dvd_B, ...
                cmdB_raw, cmdB, f1, f2, KD_B_eff);
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
        kf = k;
        poseB_ant = p2;
        poseB_psi_ant = psi2;
        vd_B_ant = vd_B;
        t_ant = t;
        cmdB_prev = cmdB;
        em_preparacao_ant = em_preparacao;

        if mod(k, 30) == 0
            if em_preparacao
                fprintf('Preparação t=%4.1fs | alvo Bebop=(%+.2f,%+.2f,%+.2f)\n', t, p2d(1), p2d(2), p2d(3));
            else
                fprintf('t=%5.1fs | PoI=(%+.2f,%+.2f) ref=(%+.2f,%+.2f) | v=%+.2f w=%+.2f\n', ...
                    t_traj, poi(1), poi(2), ref_xy(1), ref_xy(2), cmdL(1), cmdL(2));
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
    send(pub_LD, msg_LD); % pouso sempre que há Bebop real, mesmo em decolagem manual
end
pause(0.5);
rosshutdown;

if audit_fid >= 0
    if kf > 0
        idx_audit = 1:kf;
        erro_norma = vecnorm(H.erroB(:, idx_audit), 2, 1);
        fprintf(audit_fid, '=== RESUMO DA EXECUÇÃO ===\n');
        fprintf(audit_fid, 'Amostras: %d\n', kf);
        fprintf(audit_fid, 'Erro RMS do Bebop [m]: %.6f\n', sqrt(mean(erro_norma.^2)));
        fprintf(audit_fid, 'Erro máximo do Bebop [m]: %.6f\n', max(erro_norma));
        fprintf(audit_fid, 'Erro final do Bebop [m]: %.6f\n', erro_norma(end));
        fprintf(audit_fid, 'Distância mínima LIMO-obstáculo [m]: %.6f\n', min(H.dobs(idx_audit)));
        fprintf(audit_fid, 'Amostras com saturação do Bebop: %d de %d\n', nnz(H.satB(idx_audit)), kf);
    end
    fprintf(audit_fid, 'Fim: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fclose(audit_fid);
    fprintf('Auditoria salva em %s\n', audit_file);
end

if kf > 1
    idx = 1:kf;
    figure('Name', 'Formação 2: trajetórias XY', 'Color', 'w');
    hold on; axis equal; grid on;
    plot(H.ref(1, idx), H.ref(2, idx), 'k--', 'DisplayName', 'Lemniscata desejada');
    plot(H.poi(1, idx), H.poi(2, idx), 'b', 'LineWidth', 1.5, 'DisplayName', 'PoI LIMO');
    plot(H.p2(1, idx), H.p2(2, idx), 'r', 'LineWidth', 1.2, 'DisplayName', 'Bebop');
    desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_radius, 'k-');
    desenhar_circulo(cfg.obstacle_center(1), cfg.obstacle_center(2), cfg.obstacle_influence_radius, 'k:');
    xlabel('x [m]'); ylabel('y [m]');
    legend('Location', 'bestoutside');
end

%% Funções locais — cluster (Estrutura Virtual)

function q = cluster_state(p1, p2)
    % q = [xf;yf;zf;rho;alpha;beta]; offset = p2-p1 = rho*[ca*cb; sa*cb; sb]
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
    % NSB (Eq. 5.13): evasão com prioridade máxima, formação projetada no
    % espaço nulo da tarefa de evasão. J1 é a Jacobiana da tarefa ESCALAR
    % (direção do gradiente repulsivo), não um seletor de xf,yf — assim o
    % espaço nulo remove só a componente radial e preserva a tangencial,
    % permitindo contornar o obstáculo em vez de zerar toda a formação.
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

%% Funções locais — ROS/IO e utilitários

function prosseguir = aguardar_sinal_inicio(J, btn_start, btn_stop)
    prosseguir = true;
    while true
        btns = button(J);
        if numel(btns) >= btn_stop && btns(btn_stop)
            prosseguir = false;
            return;
        end
        if numel(btns) >= btn_start && btns(btn_start)
            return;
        end
        pause(0.05);
    end
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

function estado = avancar_bebop_virtual(estado, u, dt, f1, f2)
    % Planta usada apenas em MODO_BEBOP='off'.
    R = [cos(estado.psi), -sin(estado.psi), 0; sin(estado.psi), cos(estado.psi), 0; 0, 0, 1];
    estado.p = estado.p + dt * R * estado.v_body(1:3);
    estado.psi = wrap_pi(estado.psi + dt * estado.v_body(4));
    estado.v_body = estado.v_body + dt * (f1 * u - f2 * estado.v_body);
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

function imprimir_matriz(fid, M)
    for i = 1:size(M, 1)
        fprintf(fid, '  ');
        fprintf(fid, '%+12.6f ', M(i, :));
        fprintf(fid, '\n');
    end
end

function registrar_auditoria(fid, t, t_traj, em_preparacao, dt, modo_bebop, traj, poi, ref_xy, psi1, ...
        p2, p2d, psi2, A2inv, dx2, vB_meas, vd_B, dvd_B, cmdB_raw, cmdB, f1, f2, KD_B)
    A2 = A2inv.';
    dx2_reconstruido = A2 * vd_B(1:3);
    residuo_cinematico = dx2_reconstruido - dx2;
    aceleracao_requerida = dvd_B + KD_B * (vd_B - vB_meas);
    aceleracao_modelo_bruta = f1 * cmdB_raw - f2 * vB_meas;
    aceleracao_modelo_saturada = f1 * cmdB - f2 * vB_meas;
    erro_saturacao = cmdB - cmdB_raw;

    fprintf(fid, '================================================================\n');
    fprintf(fid, 't = %.3f s | modo Bebop = %s\n', t, modo_bebop);
    fprintf(fid, 't da trajetória = %.3f s | preparação = %d\n', t_traj, em_preparacao);
    fprintf(fid, 'TRAJ = %d | dt efetivo = %.6f s\n', traj, dt);
    fprintf(fid, '--- LIMO E REFERÊNCIA ---\n');
    fprintf(fid, 'PoI LIMO [m]:\n'); fprintf(fid, '  %+.6f\n', poi);
    fprintf(fid, 'Referência lemniscata [m]:\n'); fprintf(fid, '  %+.6f\n', ref_xy);
    fprintf(fid, 'Yaw LIMO: %+.6f rad (%+.3f graus)\n', psi1, rad2deg(psi1));

    fprintf(fid, '--- ESTADO E ALVO DO BEBOP ---\n');
    fprintf(fid, 'p2 medido ou virtual [m]:\n'); fprintf(fid, '  %+.6f\n', p2);
    fprintf(fid, 'p2d de formação [m]:\n'); fprintf(fid, '  %+.6f\n', p2d);
    fprintf(fid, 'Erro p2d - p2 [m]:\n'); fprintf(fid, '  %+.6f\n', p2d - p2);
    fprintf(fid, 'Yaw Bebop: %+.6f rad (%+.3f graus)\n', psi2, rad2deg(psi2));

    fprintf(fid, '--- CINEMÁTICA DO BEBOP ---\n');
    imprimir_matriz(fid, A2);
    imprimir_matriz(fid, A2inv);
    fprintf(fid, 'dx2 desejada no mundo [m/s]:\n'); fprintf(fid, '  %+.6f\n', dx2);
    fprintf(fid, 'vB desejada no corpo [vx; vy; vz; psidot]:\n'); fprintf(fid, '  %+.6f\n', vd_B);
    fprintf(fid, 'Resíduo cinemático:\n'); fprintf(fid, '  %+.6e\n', residuo_cinematico);

    fprintf(fid, '--- COMPENSADOR DINÂMICO DO BEBOP ---\n');
    fprintf(fid, 'vB medida [vx; vy; vz; psidot]:\n'); fprintf(fid, '  %+.6f\n', vB_meas);
    fprintf(fid, 'dvd_B [m/s²; rad/s²]:\n'); fprintf(fid, '  %+.6f\n', dvd_B);
    fprintf(fid, 'Aceleração requerida:\n'); fprintf(fid, '  %+.6f\n', aceleracao_requerida);
    fprintf(fid, 'Comando bruto f1\\(...):\n'); fprintf(fid, '  %+.6f\n', cmdB_raw);
    fprintf(fid, 'Comando após saturação:\n'); fprintf(fid, '  %+.6f\n', cmdB);
    fprintf(fid, 'Erro de saturação:\n'); fprintf(fid, '  %+.6f\n', erro_saturacao);
    fprintf(fid, 'Resíduo dinâmico bruto:\n'); fprintf(fid, '  %+.6e\n', aceleracao_modelo_bruta - aceleracao_requerida);
    fprintf(fid, 'f1*cmd_saturado - f2*vB_medida:\n'); fprintf(fid, '  %+.6f\n', aceleracao_modelo_saturada);
    fprintf(fid, '\n');
end
