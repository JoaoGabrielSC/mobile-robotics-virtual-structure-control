% Formação LIMO-Bebop usando o mesmo controle do LIMO de limoControl.m.
% O LIMO rastreia a lemniscata pelo PoI; o Bebop mantém a formação cartesiana.
clear; clc; close all;

%% Configuração
cfg.T = 1 / 30;
cfg.Tsim = 120;
cfg.takeoff_wait_s = 5;
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
cfg.obstacle_influence_radius = 0.25;
cfg.use_obstacle_avoidance = true;
cfg.obstacle_potential_gain = 0.80;
cfg.obstacle_potential_exponent = 4;
cfg.obstacle_potential_shape_a = [];
cfg.obstacle_potential_shape_b = [];
cfg.obstacle_potential_vmax = 0.80;
cfg.crossing_center = [0.0; 0.0];
cfg.crossing_zone_radius = 0.01;
cfg.crossing_feedback_min = 0.20;
cfg.crossing_cross_track_gain = 0.35;
cfg.audit_enabled = true;
cfg.audit_period = 1.0;
cfg.audit_dir = fullfile('results', 'formacao_2');

TRAJ = 1; % 0: teste de posicionamento (LIMO para em [0;0]), 1: lemniscata

% Alvo fixo do Bebop no teste de posicionamento (TRAJ = 0). O offset em X
% existe para o drone NÃO ficar exatamente sobre o LIMO, que fica parado na
% origem nesse teste. Constante independente de offset_f de propósito: o
% alvo do teste não deve mudar sozinho quando beta_f/rho_f forem ajustados.
cfg.p2d_teste = [0.75; 0.00; 1.00];   % [x y z] em metros, referencial global
rho_f = 1.5; % obs.: com beta = 60, a altura fica 1.3, entao para manter a altura igual 1.5, temos que colocar rho = 1.73
alpha_f = 0;
beta_f = pi / 3;   % 60 graus (era pi/2 na especificação original)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do enunciado: beta_f é a ELEVAÇÃO e alpha_f é o AZIMUTE, medido a partir do eixo X global
%     x2 = x_f + rho_f*cos(beta_f)*cos(alpha_f)
%     y2 = y_f + rho_f*cos(beta_f)*sin(alpha_f)
%     z2 = z_f + rho_f*sin(beta_f)

%   beta_f = 60 graus -> offset = [0.750, 0.000, 1.299] 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
offset_f = [rho_f * cos(beta_f) * cos(alpha_f);
            rho_f * cos(beta_f) * sin(alpha_f);
            rho_f * sin(beta_f)];
Kp_B = diag([1.0, 1.0, 1.2]);
Ls_B = diag([0.6, 0.6, 0.6]);
KD_B = diag([2.5, 2.5, 2.0, 5.0]);

cfg.yaw_d_B = 0.0;      % guinada desejada [rad]; 0 = alinhado ao eixo X global
cfg.k_yaw_B = 1.0;      % ganho do controlador cinemático de guinada [1/s]
cfg.wd_B_max = 0.6;     % saturação da taxa de guinada desejada [rad/s]
f1 = diag([0.8417, 0.8354, 3.966, 9.8524]);
f2 = diag([0.18227, 0.17095, 4.001, 4.7295]);

cmdB_max = [0.5; 0.5; 0.3; 0.5];

cfg.bebop_limite_x_pos = 1.8;      % parede virtual: limite em +x [m]
cfg.bebop_limite_x_neg = -1.8;     % parede virtual: limite em -x [m]
cfg.bebop_limite_y_pos = 1.8;      % parede virtual: limite em +y [m]
cfg.bebop_limite_y_neg = -1.8;     % parede virtual: limite em -y [m]
cfg.bebop_limite_z_pos = 1.8;      % parede virtual: limite em +z [m]

MODO_BEBOP = 'teste'; % 'off', 'teste' ou 'voo'
BTN_STOP = 1;
use_preparation = strcmp(MODO_BEBOP, 'voo') && cfg.preparation_time_s > 0;
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
    fprintf(audit_fid, '=== AUDITORIA DE FORMAÇÃO LIMO-BEBOP ===\n');
    fprintf(audit_fid, 'Início: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(audit_fid, 'Modo Bebop: %s\n', MODO_BEBOP);
    fprintf(audit_fid, 'Trajetória: %d\n', TRAJ);
    fprintf(audit_fid, 'Formação: rho_f=%.3f m, alpha_f=%.1f deg, beta_f=%.1f deg\n', ...
        rho_f, rad2deg(alpha_f), rad2deg(beta_f));
    fprintf(audit_fid, 'Offset da formação (mundo): [%+.4f %+.4f %+.4f] m\n', offset_f);
    fprintf(audit_fid, 'Espera após takeoff: %.1f s\n', ...
        strcmp(MODO_BEBOP, 'voo') * cfg.takeoff_wait_s);
    fprintf(audit_fid, 'Preparação antes da trajetória: %.1f s\n', ...
        preparation_time_s);
    fprintf(audit_fid, 'Amostragem de auditoria: %.2f s\n', cfg.audit_period);
    if strcmp(MODO_BEBOP, 'off')
        fprintf(audit_fid, ['AVISO: Bebop desligado. A pose é gerada por uma planta virtual ', ...
            'com v_dot=f1*u-f2*v; os comandos calculados não são enviados.\n']);
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

if strcmp(MODO_BEBOP, 'voo')
    send(pub_TO, msg_TO);
    fprintf('Takeoff enviado. Aguardando %.1f s para estabilização inicial.\n', ...
        cfg.takeoff_wait_s);
    pause(cfg.takeoff_wait_s);
end

[x1, y1, z1, psi1, ts1] = ler_pose(pose_L);
if strcmp(MODO_BEBOP, 'off')
    virtual_B.p = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1); z1] + offset_f; % modificacao para beta = 60, ja que ela nasceria fora da formação
    virtual_B.psi = 0;
    virtual_B.v_body = zeros(4, 1);
    x2 = virtual_B.p(1);
    y2 = virtual_B.p(2);
    z2 = virtual_B.p(3);
    psi2 = virtual_B.psi;
    ts2 = ts1;
else
    [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
end

cfg.optitrack_timeout_s = 0.5;
last_ts_L = ts1; last_update_L = tic;
last_ts_B = ts2; last_update_B = tic;

v_limo_state = [0; 0];
vd_B_ant = [0; 0; 0; 0];
poseB_ant = [x2; y2; z2];
poseB_psi_ant = psi2;
t_ant = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODIFICAÇÃO: removidos os estados cmdB_prev (rate limiter) e dvd_B_filt
% (filtro de derivada). em_preparacao_ant é mantido: ele detecta a transição
% preparação -> formação, onde a derivada de vd_B ainda precisa ser zerada.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
em_preparacao_ant = true;

% Feedforward do Bebop: velocidade desejada do PoI no referencial do mundo,
% devolvida por limo_reference_controller.
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
    fprintf(['Preparando o Bebop por %.1f s com LIMO parado. ', ...
        'Botão %d do joystick para parar.\n'], preparation_time_s, BTN_STOP);
else
    fprintf('Iniciando formação. Botão %d do joystick para parar.\n', BTN_STOP);
end
t0 = tic;
kf = 0;
try
    for k = 1:N
        tloop = tic;
        t = toc(t0);
        if k == 1
            dt = cfg.T;
        else
            dt = max(t - t_ant, 1e-3);
        end

        btns = button(J);
        if numel(btns) >= BTN_STOP && btns(BTN_STOP)
            fprintf('Parada solicitada pelo joystick.\n');
            break;
        end

        [x1, y1, z1, psi1, ts1] = ler_pose(pose_L);
        if strcmp(MODO_BEBOP, 'off')
            x2 = virtual_B.p(1);
            y2 = virtual_B.p(2);
            z2 = virtual_B.p(3);
            psi2 = virtual_B.psi;
            ts2 = ts1;
        else
            [x2, y2, z2, psi2, ts2] = ler_pose(pose_B);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % MODIFICAÇÃO: verificação do watchdog de OptiTrack (ver ler_pose).
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if ts1 > last_ts_L
            last_ts_L = ts1; last_update_L = tic;
        end
        if ts2 > last_ts_B
            last_ts_B = ts2; last_update_B = tic;
        end
        if toc(last_update_L) > cfg.optitrack_timeout_s || ...
                (~strcmp(MODO_BEBOP, 'off') && toc(last_update_B) > cfg.optitrack_timeout_s)
            fprintf('OptiTrack perdido por mais de %.1f s. Abortando.\n', cfg.optitrack_timeout_s);
            break;
        end

        poi = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];
        em_preparacao = t < preparation_time_s;
        t_traj = max(0, t - preparation_time_s);

        % Mesmo laço externo e interno usados em limoControl.m.
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

        % Os ramos de preparação e de trajetória usam a MESMA geometria
        % (offset_f). Antes a preparação tinha a fórmula de beta = 90 graus
        % chumbada, o que com beta = 60 produzia um degrau de p2d de
        % [+0.750, 0, -0.201] m no início da trajetória: o Bebop era mandado
        % descer 0.229 m/s enquanto arrancava a 0.5 m/s.
        if em_preparacao
            p2d = [poi(1); poi(2); z1] + offset_f;
        elseif TRAJ == 0
            p2d = cfg.p2d_teste;
        else
            p2d = [poi(1); poi(2); z1] + offset_f;
        end
        p2 = [x2; y2; z2];

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % MODIFICAÇÃO: parede virtual (geofencing) para o Bebop.
        % Motivo: formacao_2.m não tinha nenhum limite espacial de segurança
        % para o Bebop; formacion_limo_bebop_final.m (que voou com sucesso)
        % aborta e aterrissa se o Bebop sai de uma caixa [-1.8,1.8] m em xy
        % e z<1.8 m. Copiado porque é uma rede de segurança independente do
        % controlador — protege mesmo se o controlador calcular um comando
        % ruim por qualquer motivo.
        % Impacto esperado: nenhum em operação normal; interrompe o
        % experimento com segurança em caso de comportamento anômalo.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if ~strcmp(MODO_BEBOP, 'off') && ...
                (p2(1) > cfg.bebop_limite_x_pos || p2(1) < cfg.bebop_limite_x_neg || ...
                 p2(2) > cfg.bebop_limite_y_pos || p2(2) < cfg.bebop_limite_y_neg || ...
                 p2(3) > cfg.bebop_limite_z_pos)
            fprintf('PAREDE VIRTUAL: Bebop fora dos limites (%.2f,%.2f,%.2f). Abortando.\n', ...
                p2(1), p2(2), p2(3));
            break;
        end

        % MODIFICAÇÃO: feedforward do Bebop vem agora direto de vel_poi (saída
        % do controlador cinemático do LIMO), e não mais de A1*cmdL, que era
        % reconstruído a partir do estado simulado do laço interno.
        % Zerado em TRAJ==0 porque nesse caso p2d é um ponto FIXO
        % (cfg.p2d_teste), logo sua derivada é nula.
        vel_poi_world = vel_poi_ff;
        %vel_poi_world = [cos(psi1), -cfg.a1 * sin(psi1);
        %                 sin(psi1),  cfg.a1 * cos(psi1)] * cmdL;
        if TRAJ == 0
            vel_poi_world = [0; 0];
        end

        % Laço externo do Bebop: eq. (5.7)/(5.12) do livro, v_r = v_d + L tanh(L\K q~).
        % O tanh já limita a parcela de erro de posição; a única saturação
        % restante é a do comando final (cmdB_max).
        dx2 = [vel_poi_world; 0] + Ls_B * tanh(Ls_B \ (Kp_B * (p2d - p2)));

        A2inv = [cos(psi2), sin(psi2), 0;
                 -sin(psi2), cos(psi2), 0;
                 0, 0, 1];
        if strcmp(MODO_BEBOP, 'off')
            vB_meas = virtual_B.v_body;
        else
            velWB = (p2 - poseB_ant) / dt;
            psidot2 = wrap_pi(psi2 - poseB_psi_ant) / dt;
            vB_meas = [A2inv * velWB; psidot2];
        end
        % MODIFICAÇÃO: quarto elemento deixa de ser 0 e passa a vir de um
        % controlador cinemático de guinada que regula o ÂNGULO psi2 (ver
        % cfg.yaw_d_B / cfg.k_yaw_B). wrap_pi trata o cruzamento em ±pi.
        w_d_B = cfg.k_yaw_B * wrap_pi(cfg.yaw_d_B - psi2);
        vd_B = [A2inv * dx2; saturar(w_d_B, cfg.wd_B_max)];

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % MODIFICAÇÃO: derivada por diferença finita bruta, como em
        % formacion_limo_bebop_final.m (linha 271). Removidos o filtro
        % passa-baixa e a saturação de dvd_B.
        % Mantido apenas o reset no primeiro ciclo e na transição
        % preparação -> formação: nesses dois instantes vd_B_ant não
        % corresponde ao ciclo anterior e a diferença finita produziria um
        % pico numérico sem significado físico.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        transicao_prep_formacao = em_preparacao_ant && ~em_preparacao;
        if k == 1 || transicao_prep_formacao
            dvd_B = zeros(4, 1);
        else
            dvd_B = (vd_B - vd_B_ant) / dt;
        end

        cmdB_raw = f1 \ (dvd_B + KD_B * (vd_B - vB_meas) + f2 * vB_meas);

        % Única saturação restante: comando final por eixo (ver cmdB_max).
        cmdB = max(min(cmdB_raw, cmdB_max), -cmdB_max);

        audit_step = max(1, round(cfg.audit_period / cfg.T));
        if audit_fid >= 0 && (k == 1 || mod(k - 1, audit_step) == 0)
            registrar_auditoria(audit_fid, t, t_traj, em_preparacao, dt, MODO_BEBOP, ...
                TRAJ, poi, ref_xy, psi1, ...
                p2, p2d, psi2, A2inv, dx2, vB_meas, vd_B, dvd_B, ...
                cmdB_raw, cmdB, f1, f2, KD_B);
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
        kf = k;
        poseB_ant = p2;
        poseB_psi_ant = psi2;
        vd_B_ant = vd_B;
        t_ant = t;
        em_preparacao_ant = em_preparacao;

        if mod(k, 30) == 0
            erro_xyz = p2d - p2;
            if em_preparacao
                fprintf(['Preparação t=%4.1fs | alvo=(%+.2f,%+.2f,%+.2f) ', ...
                    'erro=(%+.2f,%+.2f,%+.2f) |%.3fm| cmdB=(%+.2f,%+.2f,%+.2f,%+.2f)\n'], ...
                    t, p2d(1), p2d(2), p2d(3), erro_xyz(1), erro_xyz(2), erro_xyz(3), ...
                    norm(erro_xyz), cmdB(1), cmdB(2), cmdB(3), cmdB(4));
            else
                fprintf('t=%5.1fs | PoI=(%+.2f,%+.2f) ref=(%+.2f,%+.2f) | v=%+.2f w=%+.2f\n', ...
                    t_traj, poi(1), poi(2), ref_xy(1), ref_xy(2), cmdL(1), cmdL(2));
                fprintf(['           Bebop alvo=(%+.2f,%+.2f,%+.2f) ', ...
                    'erro=(%+.2f,%+.2f,%+.2f) |%.3fm| cmdB=(%+.2f,%+.2f,%+.2f,%+.2f)\n'], ...
                    p2d(1), p2d(2), p2d(3), erro_xyz(1), erro_xyz(2), erro_xyz(3), ...
                    norm(erro_xyz), cmdB(1), cmdB(2), cmdB(3), cmdB(4));
            end
        end
        pause(max(0, cfg.T - toc(tloop)));
    end
catch ME
    fprintf(2, 'ERRO no loop: %s\n', ME.message);
end

msg_L.Linear.X = 0;
msg_L.Linear.Y = 0;
msg_L.Linear.Z = 0;
msg_L.Angular.Z = 0;
send(pub_L, msg_L);
if ~strcmp(MODO_BEBOP, 'off')
    msg_B.Linear.X = 0;
    msg_B.Linear.Y = 0;
    msg_B.Linear.Z = 0;
    msg_B.Angular.Z = 0;
    send(pub_B, msg_B);
end
if strcmp(MODO_BEBOP, 'voo')
    send(pub_LD, msg_LD);
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
        fprintf(audit_fid, 'Amostras com saturação do Bebop: %d de %d\n', ...
            nnz(H.satB(idx_audit)), kf);
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

function [x, y, z, psi, tstamp] = ler_pose(sub)
    p = sub.LatestMessage;
    quat = [p.Pose.Orientation.W, p.Pose.Orientation.X, ...
            p.Pose.Orientation.Y, p.Pose.Orientation.Z];
    eul = quat2eul(quat);
    x = p.Pose.Position.X;
    y = p.Pose.Position.Y;
    z = p.Pose.Position.Z;
    psi = eul(1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MODIFICAÇÃO: ler_pose agora também retorna o timestamp do header.
    % Motivo: docs/diagnostico_formacao_2_queda_bebop.md (item 3) aponta que
    % o código lia apenas LatestMessage sem checar se a amostra era nova,
    % permitindo que uma pose repetida/atrasada do OptiTrack alimentasse
    % vB_meas e dvd_B com ruído/degraus espúrios, amplificados pelo
    % compensador dinâmico (KD_B, f1\...). O código que voou com sucesso
    % (formacion_limo_bebop_final.m) só atualiza pose e aciona emergência
    % se o timestamp ficar parado por mais de 0.5 s.
    % Impacto esperado: permite implementar o watchdog de OptiTrack no laço
    % principal sem alterar a lógica de controle em si.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tstamp = double(p.Header.Stamp.Sec) + double(p.Header.Stamp.Nsec) * 1e-9;
end

function [ref_xy, ref_xy_dot] = lemniscata_reference(t)
    phase_x = 2 * pi * t / 40;
    phase_y = 4 * pi * t / 40;
    ref_xy = [0.75 * sin(phase_x); 0.75 * sin(phase_y)];
    ref_xy_dot = [0.75 * (2 * pi / 40) * cos(phase_x);
                  0.75 * (4 * pi / 40) * cos(phase_y)];
end

% MODIFICAÇÃO: terceira saída vel_poi — a velocidade DESEJADA do PoI no
% referencial do mundo, colhida antes de A1inv. É o x1_r do ponto de split da
% Fig. 5.12 do livro, e é o sinal correto para alimentar o feedforward do
% Bebop. Antes o laço principal reconstruía essa velocidade como A1*cmdL, ou
% seja, a partir do estado SIMULADO de limo_inner_loop, o que introduzia de
% 4% a 49% de erro (a ida e volta A1inv->A1 só seria identidade se o laço
% interno fosse identidade, e ele amplifica omega em 1,77x).
% Não é saturada de propósito: v_max/w_max são limites do ROBÔ, não da
% referência da formação. O drone segue a referência; se o LIMO saturar e
% ficar para trás, p2d (que usa o PoI MEDIDO) fica para trás junto e o termo
% proporcional corrige. É o mesmo sinal que formacion_limo_bebop_final.m usa
% (v_cmd_L1, linha 213), também sem saturação e também incluindo a parcela
% de repulsão do obstáculo.
function [v_d, ref_xy, vel_poi] = limo_reference_controller(t, poi, psi, traj, cfg)
    if traj == 1
        [ref_xy, ref_xy_dot] = lemniscata_reference(t);
    else
        ref_xy = [0; 0];
        ref_xy_dot = [0; 0];
    end
    err_xy = ref_xy - poi;
    err_xy = attenuate_crossing_error(err_xy, ref_xy_dot, poi, cfg);
    [kq_eff, lq_eff] = crossing_gain_scale(poi, cfg);
    vel_poi = ref_xy_dot + lq_eff * tanh((kq_eff / max(lq_eff, 1e-6)) * err_xy);
    if cfg.use_obstacle_avoidance
        vel_poi = apply_obstacle_null_space_xy(vel_poi, poi, cfg);
    end
    A1inv = [cos(psi), sin(psi);
             -sin(psi) / cfg.a1, cos(psi) / cfg.a1];
    u = A1inv * vel_poi;
    v_d = [saturar(u(1), cfg.v_max); saturar(u(2), cfg.w_max)];
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
    cross_gain = cfg.crossing_cross_track_gain + ...
        (1 - cfg.crossing_cross_track_gain) * (1 - blend);
    err_xy = err_along + cross_gain * err_cross;
end

function [kq_eff, lq_eff] = crossing_gain_scale(poi, cfg)
    dist_cross = norm(poi - cfg.crossing_center);
    if dist_cross >= cfg.crossing_zone_radius
        kq_eff = cfg.kq;
        lq_eff = cfg.lq;
        return;
    end
    scale = cfg.crossing_feedback_min + (1 - cfg.crossing_feedback_min) * ...
        (dist_cross / cfg.crossing_zone_radius)^2;
    kq_eff = cfg.kq * scale;
    lq_eff = cfg.lq * scale;
end

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
    dx = offset(1);
    dy = offset(2);
    scale = cfg.obstacle_potential_gain * exp(-((dx / a)^n + (dy / b)^n)) * n;
    grad = [scale * sign(dx) * abs(dx)^(n - 1) / a^n;
            scale * sign(dy) * abs(dy)^(n - 1) / b^n];
    grad_norm = norm(grad);
    if grad_norm > cfg.obstacle_potential_vmax
        grad = grad * (cfg.obstacle_potential_vmax / grad_norm);
    end
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

function registrar_auditoria(fid, t, t_traj, em_preparacao, dt, modo_bebop, traj, poi, ref_xy, psi1, ...
        p2, p2d, psi2, A2inv, dx2, vB_meas, vd_B, dvd_B, ...
        cmdB_raw, cmdB, f1, f2, KD_B)
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

    fprintf(fid, '--- MATRIZES CINEMÁTICAS DO BEBOP ---\n');
    fprintf(fid, 'A2: velocidade no mundo = A2 * velocidade no corpo\n');
    imprimir_matriz(fid, A2);
    fprintf(fid, 'A2inv: velocidade no corpo = A2inv * velocidade no mundo\n');
    imprimir_matriz(fid, A2inv);

    fprintf(fid, '--- RESULTADO CINEMÁTICO ---\n');
    fprintf(fid, 'dx2 desejada no mundo [m/s]:\n'); fprintf(fid, '  %+.6f\n', dx2);
    fprintf(fid, 'vB desejada no corpo [vx; vy; vz; psidot]:\n'); fprintf(fid, '  %+.6f\n', vd_B);
    fprintf(fid, 'A2 * vB_desejada(1:3) [m/s]:\n'); fprintf(fid, '  %+.6f\n', dx2_reconstruido);
    fprintf(fid, 'Resíduo A2*vB_desejada - dx2 [m/s]:\n'); fprintf(fid, '  %+.6e\n', residuo_cinematico);
    fprintf(fid, 'Norma do resíduo cinemático: %.3e\n', norm(residuo_cinematico));

    fprintf(fid, '--- COMPENSADOR DINÂMICO DO BEBOP ---\n');
    fprintf(fid, 'vB medida [vx; vy; vz; psidot]:\n'); fprintf(fid, '  %+.6f\n', vB_meas);
    fprintf(fid, 'dvd_B [m/s²; rad/s²]:\n'); fprintf(fid, '  %+.6f\n', dvd_B);
    fprintf(fid, 'Aceleração requerida dvd_B + KD*(vd_B-vB_medida):\n');
    fprintf(fid, '  %+.6f\n', aceleracao_requerida);
    fprintf(fid, 'Comando bruto f1\\(...):\n'); fprintf(fid, '  %+.6f\n', cmdB_raw);
    fprintf(fid, 'Comando após saturação:\n'); fprintf(fid, '  %+.6f\n', cmdB);
    fprintf(fid, 'Erro causado pela saturação:\n'); fprintf(fid, '  %+.6f\n', erro_saturacao);
    fprintf(fid, 'f1*cmd_bruto - f2*vB_medida:\n');
    fprintf(fid, '  %+.6f\n', aceleracao_modelo_bruta);
    fprintf(fid, 'Resíduo dinâmico bruto:\n');
    fprintf(fid, '  %+.6e\n', aceleracao_modelo_bruta - aceleracao_requerida);
    fprintf(fid, 'Norma do resíduo dinâmico bruto: %.3e\n', ...
        norm(aceleracao_modelo_bruta - aceleracao_requerida));
    fprintf(fid, 'f1*cmd_saturado - f2*vB_medida:\n');
    fprintf(fid, '  %+.6f\n', aceleracao_modelo_saturada);
    fprintf(fid, '\n');
end

function estado = avancar_bebop_virtual(estado, u, dt, f1, f2)
    % Planta usada apenas em MODO_BEBOP='off'.
    R = [cos(estado.psi), -sin(estado.psi), 0;
         sin(estado.psi), cos(estado.psi), 0;
         0, 0, 1];
    estado.p = estado.p + dt * R * estado.v_body(1:3);
    estado.psi = wrap_pi(estado.psi + dt * estado.v_body(4));
    estado.v_body = estado.v_body + dt * (f1 * u - f2 * estado.v_body);
end

function imprimir_matriz(fid, M)
    for i = 1:size(M, 1)
        fprintf(fid, '  ');
        fprintf(fid, '%+12.6f ', M(i, :));
        fprintf(fid, '\n');
    end
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
