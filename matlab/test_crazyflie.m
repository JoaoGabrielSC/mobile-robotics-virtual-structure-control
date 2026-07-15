%% test_crazyflie.m - Teste isolado do Crazyflie (conexão ROS + OptiTrack + cmd_vel)
% Robótica Móvel 2026/1 - LAB-AIR
%
% Use este script ANTES do main.m para validar apenas o quadrimotor, do
% mesmo jeito que test_limo.m valida apenas o LIMO. Os dois scripts NÃO
% se comunicam entre si: a malha acoplada da formação (estrutura virtual,
% Sarcinelli-Filho & Carelli, 2023, Cap. 5.4-5.6) é responsabilidade do
% main.m, que ainda depende da validação individual feita aqui.
%
% ===== Pré-requisitos (MatlabScriptToUseLIMOCrazyflieAsTheRoboticFormation) =====
%  1) Motive: corpo rígido nomeado cfX (X = número do Crazyflie)
%  2) roslaunch natnet_ros_cpp natnet_ros.launch  (no rosserver .100)
%  3) Crazyflie ligado; no rosserver:
%       roslaunch crazyflie_server crazyflie_server.launch cfs:=[X]
%  4) JoyControl.m no path do MATLAB
%
% ===== ATENÇÃO — SEGURANÇA =====
%  Este script comanda um drone real. Antes do primeiro voo:
%   - Teste em bancada, COM AS HÉLICES REMOVIDAS, o modo 'teleop' para
%     confirmar que os comandos de phi/theta produzem os ângulos
%     esperados no visualizador do OptiTrack (Motive), com sinal correto.
%   - O mapeamento phi=Angular.X (rolagem, eixo Y do corpo) e
%     theta=Angular.Y (arfagem, eixo X do corpo), conforme o arquivo do
%     coordenador, está implementado com os sinais cfg.roll_sign e
%     cfg.pitch_sign. NÃO HÁ COMO DEDUZIR o sinal correto apenas da
%     documentação disponível — ajuste esses sinais em bancada antes de
%     confiar no controlador. Ver seção "Mapeamento atitude" abaixo.
%   - O botão de parada aciona 'land' (pouso controlado). Use o botão de
%     kill (motores cortados imediatamente) apenas em emergência real,
%     pois o drone cai em queda livre.
%
% ===== Modos =====
%  'monitor'      — só lê pose, não decola (mais seguro; primeiro teste)
%  'teleop'       — decola e joystick comanda phi/theta/zdot/psidot
%  'takeoff_land' — decola, paira alguns segundos (firmware), pousa
%  'hover'        — decola e mantém uma posição 3D fixa (laço completo)
%  'lemniscate'   — decola e segue a lemniscata do enunciado a z=1,5 m
%
% ===== Modelo teórico (Sarcinelli-Filho & Carelli, 2023) =====
%  Cinemática near-hover:      Cap. 2.2, eq. (2.25)-(2.26)
%  Dinâmica near-hover:        Cap. 3.3, eq. (3.29)-(3.31)
%  Laço externo (tanh):        Cap. 4.5, eq. (4.10), (4.12a-d)
%  Compensação dinâmica cascata: Cap. 4.11, eq. (4.47)
%
% ===== Suposição a confirmar com o professor =====
%  Os parâmetros f1, f2 do enunciado do trabalho foram identificados para
%  "o quadrimotor" da formação; a especificação escrita menciona Bebop 2,
%  mas a plataforma real definida para este semestre é o Crazyflie (ver
%  discussão registrada no histórico do projeto). Usamos f1,f2 como
%  melhor modelo disponível, mas eles quase certamente NÃO foram
%  identificados para o Crazyflie (massa/inércia muito diferentes de um
%  Bebop 2) — o comportamento em voo deve ser avaliado com cautela e,
%  idealmente, os parâmetros devem ser reidentificados (eq. 3.11-3.12 do
%  livro, adaptada) para o Crazyflie real.

clear;
clc;
close all;

%% ========================================================================
%  CONFIGURAÇÃO — edite aqui antes de rodar
%  ========================================================================

% --- Modo de operação ---------------------------------------------------
cfg.mode = 'monitor';
cfg.t_final = 80;                % duração do modo lemniscate (s); 80 = 2×40 s
cfg.T = 1 / 30;                  % período de amostragem (30 Hz, enunciado)
cfg.pose_timeout = 30;           % timeout aguardando primeira pose (s)

% --- Rede ROS -----------------------------------------------------------
cfg.ros_master_host = '192.168.0.100';   % rosserver (roscore)
cfg.ros_master_port = 11311;
cfg.cf_namespace = 'cf7';                % nome do corpo rígido no Motive / namespace do crazyflie_server
cfg.pose_topic_prefix = '/natnet_ros';

% --- Crazyflie: limites físicos (enunciado) ------------------------------
cfg.phi_max = deg2rad(5);        % rolagem máxima (rad) — eixo Y do corpo
cfg.theta_max = deg2rad(5);      % arfagem máxima (rad) — eixo X do corpo
cfg.vz_max = 1.0;                % |żd| máximo (m/s)
cfg.psidot_max = 100.0;          % |ψ̇d| máximo (rad/s) — limite do enunciado;
                                  % na prática use ganhos que nunca cheguem perto disso.

% --- Mapeamento atitude (ver aviso de segurança no cabeçalho) -----------
cfg.roll_sign = 1;                % sinal de u_vy -> phi (Angular.X)
cfg.pitch_sign = 1;               % sinal de u_vx -> theta (Angular.Y)

% --- Controle: laço externo (cinemático, tanh) --------------------------
% v = [vdx;vdy;vdz;vdpsi] = ẋd + Kp*tanh((Kp/Ks)*x̃), eq. (4.10)/(4.12a-d)
cfg.kp_xy = 0.6;                 % ganho proporcional em x,y
cfg.ks_xy = 0.25;                % saturação (m/s) em x,y
cfg.kp_z = 0.6;                  % ganho proporcional em z
cfg.ks_z = 0.25;                 % saturação (m/s) em z
cfg.kp_psi = 0.8;                % ganho proporcional em psi
cfg.ks_psi = 0.5;                % saturação (rad/s) em psi

% --- Controle: laço interno (compensador dinâmico near-hover) ----------
% v̇ = f1*u - f2*v (eq. 3.30); vr = f1^-1*(v̇d + KD*ṽ + f2*v) (eq. 4.47)
cfg.f1_cf = diag([0.8417, 0.8354, 3.9660, 9.8524]);   % [vx;vy;vz;vpsi], dados do enunciado
cfg.f2_cf = diag([0.18227, 0.17095, 4.0010, 4.7295]);
cfg.kd_cf = diag([2.0, 2.0, 1.8, 5.0]);               % KD, eq. (4.47)

% --- Modo 'hover' --------------------------------------------------------
cfg.hover_target = [0.0; 0.0; 1.5];   % [x;y;z] desejado (m)

% --- Modo 'lemniscate' ----------------------------------------------------
cfg.z_hover = 1.5;                % altura constante da formação (enunciado)

% --- Sequenciamento de voo -------------------------------------------------
cfg.takeoff_settle_time = 3.0;    % s de espera após o takeoff do firmware
cfg.land_settle_time = 3.0;       % s de espera após enviar land

% --- Joystick (JoyControl) ------------------------------------------------
cfg.joystick_axis_roll = 1;
cfg.joystick_axis_pitch = 2;
cfg.joystick_axis_yaw = 3;
cfg.joystick_axis_vz = 4;
cfg.joystick_deadzone = 0.15;
cfg.joystick_stop_button = 1;     % botão -> pouso controlado (land)
cfg.joystick_kill_button = 2;     % botão -> corte imediato dos motores (kill)

% --- Resultados (modos 'hover' e 'lemniscate') ----------------------------
cfg.save_results = true;
cfg.results_dir = fullfile('results', 'test_crazyflie');
cfg.save_gif = true;
cfg.gif_fps = 10;
cfg.gif_frame_step = 3;

%% ========================================================================
%  Fim da configuração — não é necessário editar abaixo desta linha
%  ========================================================================

flight_modes = {'teleop', 'takeoff_land', 'hover', 'lemniscate'};
requires_flight = any(strcmp(cfg.mode, flight_modes));

fprintf('=== Teste Crazyflie | modo: %s ===\n', cfg.mode);
fprintf('Botão %d: pouso controlado. Botão %d: KILL (corte de motores).\n', ...
    cfg.joystick_stop_button, cfg.joystick_kill_button);
if requires_flight
    fprintf(['[AVISO] Este modo faz o drone decolar. Confirme hélices/área livre ', ...
        'e sinais de phi/theta validados em bancada.\n']);
end

%% ROS
rosshutdown;
master_uri = sprintf('http://%s:%d', cfg.ros_master_host, cfg.ros_master_port);
fprintf('Conectando ao ROS master: %s\n', master_uri);
rosinit(master_uri);

pub_cmdvel = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.cf_namespace), 'geometry_msgs/Twist');
msg_cmdvel = rosmessage(pub_cmdvel);

sub_pose = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.cf_namespace), ...
    'geometry_msgs/PoseStamped');

takeoff_client = rossvcclient(sprintf('/%s/takeoff', cfg.cf_namespace), 'std_srvs/Trigger');
land_client = rossvcclient(sprintf('/%s/land', cfg.cf_namespace), 'std_srvs/Trigger');
kill_client = rossvcclient(sprintf('/%s/kill', cfg.cf_namespace), 'std_srvs/Trigger');

%% Aguardar primeira pose (antes do joystick — evita interferência)
pose_topic = sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.cf_namespace);
fprintf('Aguardando pose do OptiTrack em %s (timeout %d s)...\n', ...
    pose_topic, cfg.pose_timeout);

[pos, yaw, pose_ok, pose_info] = wait_for_pose(sub_pose, pose_topic, cfg.pose_timeout);

if pose_ok
    fprintf('Pose OK: x=%.3f y=%.3f z=%.3f yaw=%.1f deg\n', ...
        pos(1), pos(2), pos(3), rad2deg(yaw));
else
    rosshutdown;
    error(['Não foi possível ler pose do Crazyflie em %s.\n\n', ...
        'Diagnóstico: %s\n\n', ...
        'Verifique:\n', ...
        '  1) natnet_ros rodando no rosserver (.100)\n', ...
        '  2) %s visível no Motive (marcadores ativos)\n', ...
        '  3) No rosserver: rostopic echo %s\n'], ...
        pose_topic, pose_info, cfg.cf_namespace, pose_topic);
end

J = JoyControl;
fprintf('Joystick conectado.\n');

if requires_flight
    msg = struct( ...
        'teleop', 'Modo teleop: joystick comandará phi/theta/zdot/psidot após o takeoff.', ...
        'takeoff_land', 'Modo takeoff_land: decolagem, pairar e pouso automáticos.', ...
        'hover', sprintf('Modo hover: manter posição [%.2f %.2f %.2f] m.', cfg.hover_target), ...
        'lemniscate', sprintf(['Modo lemniscate: o Crazyflie seguirá a figura-8 por %.0f s a z=%.2f m.\n', ...
        '  Referência: xd=0.75*sin(2πt/40), yd=0.75*sin(4πt/40)'], cfg.t_final, cfg.z_hover));
    fprintf('%s\nÁrea livre e hélices conferidas?\n', msg.(cfg.mode));
    input('Pressione Enter para continuar (Ctrl+C para cancelar)...', 's');
end

%% Decolagem
took_off = false;
if requires_flight
    fprintf('Enviando takeoff...\n');
    took_off = call_trigger_service(takeoff_client, 'takeoff');
    if ~took_off
        rosshutdown;
        error('Falha no serviço de takeoff. Abortando.');
    end
    pause(cfg.takeoff_settle_time);
end

%% Loop principal
t0 = tic;
running = true;
log_counter = 0;
hist_t = [];
hist_pos = [];
hist_ref = [];
hist_error_xyz = [];
pose_prev = [];           % [x;y;z;psi] amostra anterior (estimação de v medido)
v_d_prev = [0.0; 0.0; 0.0; 0.0];
emergency_kill = false;

try
    while running
        loop_start = tic;
        t = toc(t0);

        mRead(J);
        Analog = J.pAnalog;
        Digital = J.pDigital;

        if is_stop_pressed(Digital, cfg.joystick_kill_button)
            fprintf('KILL solicitado pelo joystick — corte imediato dos motores.\n');
            emergency_kill = true;
            break;
        end
        if is_stop_pressed(Digital, cfg.joystick_stop_button)
            fprintf('Pouso solicitado pelo joystick.\n');
            break;
        end

        [pos, yaw, pose_ok] = read_pose(sub_pose);
        if ~pose_ok
            warning('Pose indisponível — mantendo último comando neutro.');
            send_cf_attitude_cmd(msg_cmdvel, pub_cmdvel, 0.0, 0.0, 0.0, 0.0);
            pause(cfg.T);
            continue;
        end

        pose_now = [pos(1); pos(2); pos(3); yaw];

        switch cfg.mode
            case 'monitor'
                % Só leitura de pose; nenhum comando é enviado (drone não decolou).

            case 'teleop'
                theta_cmd = cfg.theta_max * cfg.pitch_sign * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_pitch), cfg.joystick_deadzone);
                phi_cmd = cfg.phi_max * cfg.roll_sign * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_roll), cfg.joystick_deadzone);
                psidot_cmd = cfg.psidot_max * 0.1 * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_yaw), cfg.joystick_deadzone);
                zdot_cmd = cfg.vz_max * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_vz), cfg.joystick_deadzone);
                send_cf_attitude_cmd(msg_cmdvel, pub_cmdvel, phi_cmd, theta_cmd, zdot_cmd, psidot_cmd);

            case 'takeoff_land'
                send_cf_attitude_cmd(msg_cmdvel, pub_cmdvel, 0.0, 0.0, 0.0, 0.0);
                if t >= 5.0
                    fprintf('Tempo de pairar (5 s) concluído.\n');
                    running = false;
                end

            case 'hover'
                [v_d, ref_pos, err_xyz] = cf_outer_loop(t, pose_now(1:3), yaw, ...
                    cfg.hover_target, [0.0; 0.0; 0.0], cfg);
                [v_meas, pose_prev] = estimate_body_velocity(pose_now, pose_prev, cfg.T);
                v_dot_d = (v_d - v_d_prev) / cfg.T;
                v_r = cf_inner_loop(v_d, v_dot_d, v_meas, cfg);
                v_d_prev = v_d;
                send_cf_attitude_cmd(msg_cmdvel, pub_cmdvel, v_r(1), v_r(2), v_r(3), v_r(4));
                hist_t(end + 1, 1) = t; %#ok<AGROW>
                hist_pos(:, end + 1) = pose_now(1:3); %#ok<AGROW>
                hist_ref(:, end + 1) = ref_pos; %#ok<AGROW>
                hist_error_xyz(:, end + 1) = err_xyz; %#ok<AGROW>
                if t >= cfg.t_final
                    fprintf('Tempo final (%.0f s) atingido.\n', cfg.t_final);
                    running = false;
                end

            case 'lemniscate'
                [ref_pos, ref_vel] = lemniscata_reference_3d(t, cfg.z_hover);
                [v_d, ~, err_xyz] = cf_outer_loop(t, pose_now(1:3), yaw, ref_pos, ref_vel, cfg);
                [v_meas, pose_prev] = estimate_body_velocity(pose_now, pose_prev, cfg.T);
                v_dot_d = (v_d - v_d_prev) / cfg.T;
                v_r = cf_inner_loop(v_d, v_dot_d, v_meas, cfg);
                v_d_prev = v_d;
                send_cf_attitude_cmd(msg_cmdvel, pub_cmdvel, v_r(1), v_r(2), v_r(3), v_r(4));
                hist_t(end + 1, 1) = t; %#ok<AGROW>
                hist_pos(:, end + 1) = pose_now(1:3); %#ok<AGROW>
                hist_ref(:, end + 1) = ref_pos; %#ok<AGROW>
                hist_error_xyz(:, end + 1) = err_xyz; %#ok<AGROW>
                if t >= cfg.t_final
                    fprintf('Tempo final (%.0f s) atingido.\n', cfg.t_final);
                    running = false;
                end

            otherwise
                error('Modo desconhecido: %s', cfg.mode);
        end

        log_counter = log_counter + 1;
        if mod(log_counter, 30) == 0
            if strcmp(cfg.mode, 'hover') || strcmp(cfg.mode, 'lemniscate')
                fprintf('t=%5.1fs | pos=(%+.3f,%+.3f,%+.3f) ref=(%+.3f,%+.3f,%+.3f) |erro|=%.3f m\n', ...
                    t, pose_now(1), pose_now(2), pose_now(3), ref_pos(1), ref_pos(2), ref_pos(3), ...
                    norm(err_xyz));
            else
                fprintf('t=%5.1fs | x=%+.3f y=%+.3f z=%+.3f yaw=%+6.1f°\n', ...
                    t, pos(1), pos(2), pos(3), rad2deg(yaw));
            end
        end

        elapsed = toc(loop_start);
        pause(max(0.0, cfg.T - elapsed));
    end
catch ME
    fprintf('Erro: %s\n', ME.message);
end

%% Encerramento
if emergency_kill
    fprintf('Encerrando: KILL (corte imediato de motores).\n');
    call_trigger_service(kill_client, 'kill');
elseif took_off
    fprintf('Encerrando: enviando land.\n');
    send_cf_attitude_cmd(msg_cmdvel, pub_cmdvel, 0.0, 0.0, 0.0, 0.0);
    call_trigger_service(land_client, 'land');
    pause(cfg.land_settle_time);
end
rosshutdown;

if (strcmp(cfg.mode, 'hover') || strcmp(cfg.mode, 'lemniscate')) && ~isempty(hist_t)
    save_cf_results(hist_t, hist_pos, hist_ref, hist_error_xyz, cfg);
end

fprintf('Teste Crazyflie finalizado.\n');

%% Funções locais

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

        position = [pose_latest.Position.X; ...
                    pose_latest.Position.Y; ...
                    pose_latest.Position.Z];
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
        info = sprintf('Tópico %s ausente. natnet disponíveis: %s', ...
            pose_topic, strjoin(natnet_topics, ', '));
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

function send_cf_attitude_cmd(msg, pub, phi, theta, zdot, psidot)
    % Interface do crazyflie_server (MatlabScriptToUseLIMOCrazyflie...):
    % Angular.X=phi (rolagem, rad), Angular.Y=theta (arfagem, rad),
    % Linear.Z=zdot (m/s), Angular.Z=psidot (rad/s).
    msg.Linear.X = 0.0;
    msg.Linear.Y = 0.0;
    msg.Linear.Z = zdot;
    msg.Angular.X = phi;
    msg.Angular.Y = theta;
    msg.Angular.Z = psidot;
    send(pub, msg);
end

function value = read_axis(Analog, index)
    if index < 1 || index > numel(Analog)
        value = 0.0;
        return;
    end
    value = Analog(index);
end

function scaled = apply_deadzone(raw, deadzone)
    if abs(raw) < deadzone
        scaled = 0.0;
        return;
    end
    sign_raw = sign(raw);
    scaled = sign_raw * (abs(raw) - deadzone) / (1.0 - deadzone);
    scaled = max(min(scaled, 1.0), -1.0);
end

function y = clamp_scalar(x, limit)
    y = min(max(x, -limit), limit);
end

function [ref_pos, ref_vel] = lemniscata_reference_3d(t, z_hover)
    % Trajetória do enunciado no plano XY, altura constante (qd do
    % trabalho: xf=xd, yf=yd, zf=0 m referido ao chão; aqui medido no
    % referencial do drone, que deve ficar a z_hover constante).
    phase_x = 2.0 * pi * t / 40.0;
    phase_y = 4.0 * pi * t / 40.0;

    ref_pos = [0.75 * sin(phase_x); 0.75 * sin(phase_y); z_hover];
    ref_vel = [0.75 * (2.0 * pi / 40.0) * cos(phase_x); ...
               0.75 * (4.0 * pi / 40.0) * cos(phase_y); ...
               0.0];
end

function [v_d, ref_pos, err_xyz] = cf_outer_loop(t, pos, psi, ref_pos, ref_vel, cfg)
    % Laço externo cinemático (Sarcinelli-Filho & Carelli, 2023, eq. 4.10,
    % 4.12a-d): v = ẋd + Kp*tanh((Kp/Ks)*x̃); v_d (corpo) = A^-1(psi)*v.
    % Orientação da formação mantida em psi=0 (enunciado não pede giro do
    % drone), então o termo de yaw só amortece desvios acidentais.
    err_xyz = ref_pos - pos;
    psi_err = wrapToPi(0.0 - psi);

    nu_x = ref_vel(1) + cfg.ks_xy * tanh((cfg.kp_xy / cfg.ks_xy) * err_xyz(1));
    nu_y = ref_vel(2) + cfg.ks_xy * tanh((cfg.kp_xy / cfg.ks_xy) * err_xyz(2));
    nu_z = ref_vel(3) + cfg.ks_z * tanh((cfg.kp_z / cfg.ks_z) * err_xyz(3));
    nu_psi = cfg.ks_psi * tanh((cfg.kp_psi / cfg.ks_psi) * psi_err);

    % A^-1(psi) da cinemática near-hover, eq. (2.26).
    vx_b = nu_x * cos(psi) + nu_y * sin(psi);
    vy_b = -nu_x * sin(psi) + nu_y * cos(psi);

    v_d = [vx_b; vy_b; nu_z; nu_psi];
end

function [v_meas, pose_state] = estimate_body_velocity(pose_now, pose_state, T)
    % Estima [vx;vy;vz;psidot] no referencial do corpo por diferenças
    % finitas da pose do OptiTrack (o Crazyflie não expõe odometria de
    % velocidade linear ao MATLAB), para servir de realimentação v medida
    % ao compensador dinâmico (eq. 4.47).
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
    % Compensação dinâmica em cascata do quadrimotor near-hover
    % (Sarcinelli-Filho & Carelli, 2023, eq. 4.47):
    % v_r = f1^-1*(v̇d + KD*ṽ + f2*v_meas), com v_meas a velocidade REAL
    % (realimentação), não simulada — mesma correção aplicada ao LIMO.
    v_til = v_d - v_meas;
    v_r = cfg.f1_cf \ (v_dot_d + cfg.kd_cf * v_til + cfg.f2_cf * v_meas);

    % v_r = [u_vx; u_vy; u_vz; u_vpsi]. Interpretamos u_vx,u_vy como
    % comandos diretos de inclinação (rad), coerente com a hipótese de
    % near-hover (ângulos pequenos) e com a interface de atitude do
    % Crazyflie — ver aviso de segurança no cabeçalho do script.
    theta_cmd = cfg.pitch_sign * v_r(1);   % u_vx -> arfagem (eixo X do corpo)
    phi_cmd = cfg.roll_sign * v_r(2);      % u_vy -> rolagem (eixo Y do corpo)
    zdot_cmd = v_r(3);
    psidot_cmd = v_r(4);

    phi_cmd = clamp_scalar(phi_cmd, cfg.phi_max);
    theta_cmd = clamp_scalar(theta_cmd, cfg.theta_max);
    zdot_cmd = clamp_scalar(zdot_cmd, cfg.vz_max);
    psidot_cmd = clamp_scalar(psidot_cmd, cfg.psidot_max);

    v_r = [phi_cmd; theta_cmd; zdot_cmd; psidot_cmd];
end

function save_cf_results(hist_t, hist_pos, hist_ref, hist_error_xyz, cfg)
    if ~cfg.save_results
        return;
    end

    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    out_dir = cfg.results_dir;
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    prefix = sprintf('cf_%s_%s', cfg.mode, stamp);
    traj_png = fullfile(out_dir, [prefix, '_traj.png']);
    err_png = fullfile(out_dir, [prefix, '_error.png']);
    gif_file = fullfile(out_dir, [prefix, '_anim.gif']);
    mat_file = fullfile(out_dir, [prefix, '.mat']);

    rms_xyz = sqrt(mean(sum(hist_error_xyz.^2, 1)));

    fig_traj = figure('Name', 'Teste Crazyflie - trajetória', 'Visible', 'off');
    plot3(hist_ref(1, :), hist_ref(2, :), hist_ref(3, :), 'r--', 'LineWidth', 1.5, ...
        'DisplayName', 'Referência');
    hold on;
    plot3(hist_pos(1, :), hist_pos(2, :), hist_pos(3, :), 'b-', 'LineWidth', 1.2, ...
        'DisplayName', 'Crazyflie');
    plot3(hist_pos(1, 1), hist_pos(2, 1), hist_pos(3, 1), 'go', 'MarkerSize', 8, ...
        'DisplayName', 'Início');
    plot3(hist_pos(1, end), hist_pos(2, end), hist_pos(3, end), 'ks', 'MarkerSize', 8, ...
        'DisplayName', 'Fim');
    grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Trajetória — Crazyflie (%s)', stamp));
    legend('Location', 'best');
    view(45, 25);
    print(fig_traj, traj_png, '-dpng', '-r150');

    fig_err = figure('Name', 'Erro XYZ - Crazyflie', 'Visible', 'off');
    plot(hist_t, hist_error_xyz(1, :), 'r', 'DisplayName', 'Erro X');
    hold on;
    plot(hist_t, hist_error_xyz(2, :), 'g', 'DisplayName', 'Erro Y');
    plot(hist_t, hist_error_xyz(3, :), 'b', 'DisplayName', 'Erro Z');
    plot(hist_t, vecnorm(hist_error_xyz, 2, 1), 'k-', 'LineWidth', 1.2, 'DisplayName', '||erro||');
    grid on;
    xlabel('Tempo (s)'); ylabel('Erro (m)');
    legend('Location', 'best');
    title(sprintf('Erro de rastreamento (RMS=%.3f m)', rms_xyz));
    print(fig_err, err_png, '-dpng', '-r150');

    if cfg.save_gif
        save_cf_gif(hist_t, hist_pos, hist_ref, hist_error_xyz, gif_file, cfg);
    end

    results.meta.timestamp = stamp;
    results.meta.mode = cfg.mode;
    results.meta.rms_xyz = rms_xyz;
    results.hist_t = hist_t;
    results.hist_pos = hist_pos;
    results.hist_ref = hist_ref;
    results.hist_error_xyz = hist_error_xyz;
    results.cfg = cfg;
    save(mat_file, '-struct', 'results');

    close(fig_traj);
    close(fig_err);

    fprintf('Erro RMS de posição: %.3f m\n', rms_xyz);
    fprintf('Resultados salvos em %s:\n', out_dir);
    fprintf('  %s\n', traj_png);
    fprintf('  %s\n', err_png);
    if cfg.save_gif
        fprintf('  %s\n', gif_file);
    end
    fprintf('  %s\n', mat_file);
end

function save_cf_gif(hist_t, hist_pos, hist_ref, hist_error_xyz, gif_file, cfg)
    frame_step = max(1, round(cfg.gif_frame_step));
    delay = 1.0 / max(1, cfg.gif_fps);
    idx = 1:frame_step:numel(hist_t);
    n_frames = numel(idx);

    margin = 0.20;
    x_lim = [min([hist_ref(1, :), hist_pos(1, :)]) - margin, max([hist_ref(1, :), hist_pos(1, :)]) + margin];
    y_lim = [min([hist_ref(2, :), hist_pos(2, :)]) - margin, max([hist_ref(2, :), hist_pos(2, :)]) + margin];

    fig = figure('Name', 'Crazyflie GIF', 'Visible', 'off', ...
        'Position', [100, 100, 720, 640], 'Color', 'w');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');
    xlim(ax, x_lim); ylim(ax, y_lim);

    plot(ax, hist_ref(1, :), hist_ref(2, :), 'Color', [1.0, 0.6, 0.6], ...
        'LineStyle', '--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    h_trail = plot(ax, nan, nan, 'b-', 'LineWidth', 1.5);
    h_ref_now = plot(ax, nan, nan, 'r.', 'MarkerSize', 18);
    h_cf_now = plot(ax, nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    h_title = title(ax, '', 'FontSize', 11);

    legend(ax, [h_trail, h_ref_now, h_cf_now], {'Trajetória (topo)', 'Referência (t)', 'Crazyflie (t)'}, ...
        'Location', 'northeast');

    fprintf('Gerando GIF (%d frames)... ', n_frames);

    for fi = 1:n_frames
        k = idx(fi);
        t_now = hist_t(k);
        pos_now = hist_pos(:, k);
        ref_now = hist_ref(:, k);
        err = norm(hist_error_xyz(:, k));

        set(h_trail, 'XData', hist_pos(1, 1:k), 'YData', hist_pos(2, 1:k));
        set(h_ref_now, 'XData', ref_now(1), 'YData', ref_now(2));
        set(h_cf_now, 'XData', pos_now(1), 'YData', pos_now(2));

        set(h_title, 'String', sprintf('t = %.1f s | z = %.2f m | erro = %.3f m | frame %d/%d', ...
            t_now, pos_now(3), err, fi, n_frames));

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
