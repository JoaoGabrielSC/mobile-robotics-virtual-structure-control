%% drone_test.m - Teste isolado do Crazyflie (ROS + OptiTrack + cmd_vel)
% Robótica Móvel 2026/1 - LAB-AIR
%
% Use DEPOIS do test_limo.m e ANTES do main.m.
%
% ===== Pré-requisitos (refence.m) =====
%  1) Motive: corpo rígido cfX (ex.: cf7) = cfg.drone_namespace
%  2) roslaunch natnet_ros_cpp natnet_ros.launch  (rosserver .100)
%  3) roslaunch crazyflie_server crazyflie_server.launch cfs:=[X]
%  4) JoyControl.m no path do MATLAB
%  5) Modo 'formation': L1 visível no Motive (pode estar PARADO no chão)
%
% ===== MODOS =====
%  'monitor'    — 1º teste: lê pose; cmd_vel neutro
%  'hover'      — decola e mantém cfg.drone_altitude (só drone)
%  'teleop'     — joystick -> [phi; theta; zdot; psidot]
%  'formation'  — estrutura virtual (main.m); só cmd drone (command_limo=false)
%
% ===== O QUE CONFIGURAR POR MODO =====
%
%  | Parâmetro               | monitor | hover | teleop | formation |
%  |-------------------------|---------|-------|--------|-----------|
%  | cfg.mode                |    *    |   *   |   *    |     *     |
%  | cfg.drone_namespace     |    *    |   *   |   *    |     *     |
%  | cfg.do_takeoff          |   opt   |  sim  |  sim   |    sim    |
%  | cfg.takeoff_pause       |   opt   |   *   |   *    |     *     |
%  | cfg.drone_altitude      |    -    |   *   |   -    |     *     |
%  | cfg.altitude_kp         |    -    |   *   |   -    |     *     |
%  | cfg.kd_drone            |    -    |   *   |   -    |     *     |
%  | cfg.k_inner_drone       |    -    |   *   |   -    |     *     |
%  | cfg.max_phi/theta/zdot  |    -    |   *   |   *    |     *     |
%  | cfg.v_max_drone_xy/z    |    -    |  opt  |   -    |     *     |
%  | cfg.joystick_*          |    *    |   *   |   *    |     *     |
%  | cfg.limo_namespace      |    -    |   -   |   -    |     *     |
%  | cfg.a1, kq, lq          |    -    |   -   |   -    |     *     |
%  | cfg.rho_f, beta_f       |    -    |   -   |   -    |     *     |
%  | cfg.obstacle_*          |    -    |   -   |   -    |    opt    |
%  | cfg.command_limo        |    -    |   -   |   -    | false*    |
%  | cfg.t_final             |    -    |  opt  |   -    |     *     |
%
%  * = relevante   opt = opcional   - = ignorado
%  false* = manter false até integração no main.m
%
% ===== Encerramento =====
%  Botão stop (cfg.joystick_stop_button) -> land + sair
%  Botão kill  (cfg.joystick_kill_button) -> kill + sair

clear;
clc;
close all;

%% ========================================================================
%  CONFIGURAÇÃO — edite aqui antes de rodar
%  ========================================================================

% --- Modo de operação ---------------------------------------------------
% 'monitor' | 'hover' | 'teleop' | 'formation'
cfg.mode = 'monitor';
cfg.T = 1 / 30;                  % 30 Hz (enunciado)
cfg.pose_timeout = 30;           % timeout 1ª pose (s)
cfg.t_final = 80;                % duração hover / formation (s)

% --- Rede ROS -----------------------------------------------------------
cfg.ros_master_host = '192.168.0.100';
cfg.ros_master_port = 11311;
cfg.drone_namespace = 'cf7';     % = Motive + crazyflie_server
cfg.pose_topic_prefix = '/natnet_ros';

% --- LIMO (modo 'formation' — leitura; cmd desligado por padrão) --------
cfg.limo_namespace = 'L1';
cfg.a1 = 0.10;                   % PoI LIMO (m)
cfg.command_limo = false;        % true apenas no main.m (formação completa)
cfg.use_virtual_limo_ic = false; % true: CI fixa se LIMO sem pose (debug)
cfg.limo_ic = [0.40; -0.25; 0.0];
cfg.limo_ic_yaw = 0.0;

% --- Crazyflie: decolagem e altitude ------------------------------------
cfg.do_takeoff = false;          % monitor/chão: false | demais: true
cfg.takeoff_pause = 5.0;         % s após takeoff
cfg.drone_altitude = 1.5;        % m — enunciado
cfg.altitude_kp = 2.0;           % ganho PD em z

% --- Limites cmd_vel (enunciado + conservador lab) ----------------------
cfg.max_phi = deg2rad(5);
cfg.max_theta = deg2rad(5);
cfg.max_zdot = 0.50;             % spec: 1.0 m/s
cfg.max_psidot = 0.50;           % spec: 100 rad/s
cfg.v_max_drone_xy = 0.80;
cfg.v_max_drone_z = cfg.max_zdot;
cfg.v_max_state = 2.0;
cfg.k_attitude = cfg.max_theta / max(cfg.v_max_drone_xy, 1e-3);

% --- Laço interno drone -------------------------------------------------
cfg.kd_drone = 1.0;
cfg.k_inner_drone = 15.0;        % mesmo fator do main.m

% --- Formação virtual (modo 'formation') --------------------------------
cfg.kq = 1.2;
cfg.lq = 0.8;
cfg.kq_xy = cfg.kq;
cfg.kq_rho = cfg.kq * 1.25;
cfg.kq_alpha = cfg.kq * 0.83;
cfg.kq_beta = cfg.kq * 1.25;
cfg.lq_xy = cfg.lq;
cfg.lq_rho = cfg.lq * 1.25;
cfg.lq_alpha = cfg.lq * 0.62;
cfg.lq_beta = cfg.lq * 1.25;
cfg.rho_f = 1.5;                 % m
cfg.alpha_f = 0.0;
cfg.beta_f = pi / 2;             % drone à esquerda do LIMO

% --- Obstáculo (null-space no PoI LIMO — main.m) ------------------------
cfg.use_obstacle_avoidance = true;
cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence_radius = 0.50;
cfg.obstacle_influence = cfg.obstacle_influence_radius;

% --- Joystick (JoyControl) ----------------------------------------------
cfg.joystick_stop_button = 1;
cfg.joystick_kill_button = 2;
cfg.joystick_axis_pitch = 2;     % teleop -> theta
cfg.joystick_axis_roll = 3;      % teleop -> phi
cfg.joystick_axis_throttle = 4;  % teleop -> zdot
cfg.joystick_axis_yaw = 1;       % teleop -> psidot
cfg.joystick_deadzone = 0.15;
cfg.teleop_scale_phi = cfg.max_phi;
cfg.teleop_scale_theta = cfg.max_theta;
cfg.teleop_scale_zdot = cfg.max_zdot;
cfg.teleop_scale_psidot = cfg.max_psidot;

% --- Resultados ---------------------------------------------------------
cfg.save_results = true;
cfg.results_dir = fullfile('results', 'drone_test');

%% ========================================================================
%  Fim da configuração
%  ========================================================================

validate_drone_cfg(cfg);

fprintf('=== Teste DRONE | modo: %s | ns: %s ===\n', cfg.mode, cfg.drone_namespace);
fprintf('Stop=land (botão %d) | Kill (botão %d)\n', ...
    cfg.joystick_stop_button, cfg.joystick_kill_button);
if strcmp(cfg.mode, 'formation')
    fprintf('Formation: LIMO=%s (lido) | command_limo=%d\n', ...
        cfg.limo_namespace, cfg.command_limo);
end

%% ROS
rosshutdown;
master_uri = sprintf('http://%s:%d', cfg.ros_master_host, cfg.ros_master_port);
fprintf('Conectando ao ROS master: %s\n', master_uri);
rosinit(master_uri);

pub_cmdvel = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.drone_namespace), 'geometry_msgs/Twist');
msg_cmdvel = rosmessage(pub_cmdvel);

takeoff_client = rossvcclient( ...
    sprintf('/%s/takeoff', cfg.drone_namespace), 'std_srvs/Trigger');
takeoff_request = rosmessage(takeoff_client);
land_client = rossvcclient( ...
    sprintf('/%s/land', cfg.drone_namespace), 'std_srvs/Trigger');
land_request = rosmessage(land_client);
kill_client = rossvcclient( ...
    sprintf('/%s/kill', cfg.drone_namespace), 'std_srvs/Trigger');
kill_request = rosmessage(kill_client);

sub_pose_drone = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.drone_namespace), ...
    'geometry_msgs/PoseStamped');

sub_pose_limo = [];
if strcmp(cfg.mode, 'formation')
    sub_pose_limo = rossubscriber( ...
        sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.limo_namespace), ...
        'geometry_msgs/PoseStamped');
end

pub_cmdvel_limo = [];
msg_cmdvel_limo = [];
if cfg.command_limo
    pub_cmdvel_limo = rospublisher( ...
        sprintf('/%s/cmd_vel', cfg.limo_namespace), 'geometry_msgs/Twist');
    msg_cmdvel_limo = rosmessage(pub_cmdvel_limo);
end

%% Aguardar pose do drone
drone_pose_topic = sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.drone_namespace);
fprintf('Aguardando pose em %s (timeout %d s)...\n', drone_pose_topic, cfg.pose_timeout);

[pos_d, yaw_d, pose_ok, pose_info] = wait_for_optitrack_pose( ...
    sub_pose_drone, drone_pose_topic, cfg.pose_timeout);

if ~pose_ok
    send_neutral_drone_cmd(msg_cmdvel, pub_cmdvel, cfg);
    rosshutdown;
    error('Pose do drone indisponível: %s', pose_info);
end
fprintf('Pose OK: x=%.3f y=%.3f z=%.3f yaw=%.1f deg\n', ...
    pos_d(1), pos_d(2), pos_d(3), rad2deg(yaw_d));

J = JoyControl;
fprintf('Joystick conectado.\n');

if mode_requires_takeoff(cfg.mode) && cfg.do_takeoff
    fprintf('Chamando takeoff...\n');
    takeoff_resp = call(takeoff_client, takeoff_request, 'Timeout', 10);
    if ~takeoff_resp.Success
        send_neutral_drone_cmd(msg_cmdvel, pub_cmdvel, cfg);
        rosshutdown;
        error('Takeoff falhou: %s', takeoff_resp.Message);
    end
    fprintf('Takeoff OK. Aguardando %.1f s...\n', cfg.takeoff_pause);
    pause(cfg.takeoff_pause);
elseif mode_requires_takeoff(cfg.mode) && ~cfg.do_takeoff
    warning('Modo %s: do_takeoff=false — sem decolagem automática.', cfg.mode);
end

if is_motion_mode(cfg.mode)
    fprintf('Modo %s. Área livre?\n', cfg.mode);
    input('Enter para continuar (Ctrl+C para cancelar)...', 's');
end

%% Loop principal
t0 = tic;
running = true;
emergency_kill = false;
log_counter = 0;
v_drone = [0.0; 0.0; 0.0; 0.0];
v_limo_cmd = [0.0; 0.0];
hist_t = [];
hist_pos = [];
hist_yaw = [];
hist_z_err = [];
hist_rho = [];
hist_rho_err = [];

Kq = diag([cfg.kq_xy, cfg.kq_xy, cfg.kq_xy, cfg.kq_rho, cfg.kq_alpha, cfg.kq_beta]);
Lq = diag([cfg.lq_xy, cfg.lq_xy, cfg.lq_xy, cfg.lq_rho, cfg.lq_alpha, cfg.lq_beta]);

try
    while running
        loop_start = tic;
        t = toc(t0);

        mRead(J);
        Analog = J.pAnalog;
        Digital = J.pDigital;

        if is_button_pressed(Digital, cfg.joystick_kill_button)
            fprintf('Kill solicitado.\n');
            emergency_kill = true;
            break;
        end
        if is_button_pressed(Digital, cfg.joystick_stop_button)
            fprintf('Parada solicitada (land).\n');
            break;
        end

        [pos_d, yaw_d, ok_d] = read_optitrack_pose(sub_pose_drone);
        if ~ok_d
            warning('Pose drone indisponível — cmd neutro.');
            send_neutral_drone_cmd(msg_cmdvel, pub_cmdvel, cfg);
            pause(cfg.T);
            continue;
        end

        x2 = pos_d(1);
        y2 = pos_d(2);
        z2 = pos_d(3);
        psi2 = yaw_d;

        switch cfg.mode
            case 'monitor'
                u_cmd = [0.0; 0.0; 0.0; 0.0];

            case 'hover'
                zdot_d = cfg.altitude_kp * (cfg.drone_altitude - z2);
                v_d = [0.0; 0.0; zdot_d; 0.0];
                v_drone = drone_inner_loop(v_d, v_drone, cfg);
                u_cmd = velocity_to_crazyflie_cmd(v_drone, cfg);
                hist_z_err(end + 1, 1) = cfg.drone_altitude - z2; %#ok<AGROW>
                if t >= cfg.t_final
                    running = false;
                end

            case 'teleop'
                phi = cfg.teleop_scale_phi * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_roll), cfg.joystick_deadzone);
                theta = cfg.teleop_scale_theta * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_pitch), cfg.joystick_deadzone);
                zdot = cfg.teleop_scale_zdot * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_throttle), cfg.joystick_deadzone);
                psidot = cfg.teleop_scale_psidot * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_yaw), cfg.joystick_deadzone);
                u_cmd = [phi; theta; zdot; psidot];

            case 'formation'
                [pos_l, yaw_l, ok_l] = read_limo_pose_for_formation(sub_pose_limo, cfg);
                if ~ok_l
                    warning('Pose LIMO indisponível — cmd neutro.');
                    send_neutral_drone_cmd(msg_cmdvel, pub_cmdvel, cfg);
                    if cfg.command_limo
                        send_zero_limo_cmd(msg_cmdvel_limo, pub_cmdvel_limo);
                    end
                    pause(cfg.T);
                    continue;
                end

                poi_limo = [pos_l(1) + cfg.a1 * cos(yaw_l); ...
                            pos_l(2) + cfg.a1 * sin(yaw_l); ...
                            0.0];
                poi_drone = [x2; y2; z2];
                delta = poi_drone - poi_limo;
                rho = max(norm(delta(1:2)), 1e-3);
                alpha = 0.0;
                beta = atan2(delta(2), delta(1));

                q = [poi_limo; rho; alpha; beta];
                [qd, qd_dot] = desired_formation(t, cfg);
                error_q = qd - q;
                error_q(6) = atan2(sin(qd(6) - q(6)), cos(qd(6) - q(6)));

                tanh_term = tanh(Lq \ (Kq * error_q));
                q_r = qd_dot + Lq * tanh_term;
                if cfg.use_obstacle_avoidance
                    q_r = apply_obstacle_null_space(q_r, poi_limo(1:2), cfg);
                end

                S = [cos(alpha) * cos(beta), -rho * sin(alpha) * cos(beta), -rho * cos(alpha) * sin(beta); ...
                     cos(alpha) * sin(beta), -rho * sin(alpha) * sin(beta),  rho * cos(alpha) * cos(beta); ...
                     sin(alpha),              rho * cos(alpha),               0.0];
                J_inv = [eye(3), zeros(3); eye(3), S];
                x_r = J_inv * q_r;

                R_yaw = [cos(psi2), sin(psi2), 0.0; ...
                         -sin(psi2), cos(psi2), 0.0; ...
                          0.0, 0.0, 1.0];
                v_d_body = R_yaw * x_r(4:6);
                v_d_body(1:2) = clamp_vec(v_d_body(1:2), cfg.v_max_drone_xy);
                v_d_body(3) = clamp_scalar( ...
                    v_d_body(3) + cfg.altitude_kp * (cfg.drone_altitude - z2), cfg.v_max_drone_z);
                v_d = [v_d_body; 0.0];

                v_drone = drone_inner_loop(v_d, v_drone, cfg);
                u_cmd = velocity_to_crazyflie_cmd(v_drone, cfg);

                v_limo_cmd = [0.0; 0.0];
                if cfg.command_limo
                    A1_inv = [cos(yaw_l), sin(yaw_l); ...
                              -sin(yaw_l) / cfg.a1, cos(yaw_l) / cfg.a1];
                    v_limo_cmd = clamp_vec(A1_inv * x_r(1:2), 2.0);
                end

                hist_rho(end + 1, 1) = rho; %#ok<AGROW>
                hist_rho_err(end + 1, 1) = cfg.rho_f - rho; %#ok<AGROW>
                hist_z_err(end + 1, 1) = cfg.drone_altitude - z2; %#ok<AGROW>
                if t >= cfg.t_final
                    running = false;
                end

            otherwise
                error('Modo desconhecido: %s', cfg.mode);
        end

        send_drone_cmd(msg_cmdvel, pub_cmdvel, u_cmd);
        if cfg.command_limo && strcmp(cfg.mode, 'formation')
            send_limo_cmd(msg_cmdvel_limo, pub_cmdvel_limo, v_limo_cmd(1), v_limo_cmd(2));
        end

        hist_t(end + 1, 1) = t; %#ok<AGROW>
        hist_pos(:, end + 1) = pos_d; %#ok<AGROW>
        hist_yaw(end + 1, 1) = yaw_d; %#ok<AGROW>

        log_counter = log_counter + 1;
        if mod(log_counter, 30) == 0
            switch cfg.mode
                case 'monitor'
                    fprintf('t=%5.1fs | z=%+.3f | yaw=%+6.1f°\n', t, z2, rad2deg(yaw_d));
                case 'hover'
                    fprintf('t=%5.1fs | z=%+.3f alvo=%.2f err=%+.3f m\n', ...
                        t, z2, cfg.drone_altitude, cfg.drone_altitude - z2);
                case 'teleop'
                    fprintf('t=%5.1fs | z=%+.3f | cmd=[%.3f %.3f %.3f %.3f]\n', ...
                        t, z2, u_cmd(1), u_cmd(2), u_cmd(3), u_cmd(4));
                case 'formation'
                    fprintf('t=%5.1fs | z=%+.3f rho=%.3f ref=%.2f err_rho=%+.3f\n', ...
                        t, z2, hist_rho(end), cfg.rho_f, hist_rho_err(end));
            end
        end

        elapsed = toc(loop_start);
        pause(max(0.0, cfg.T - elapsed));
    end
catch ME
    fprintf('Erro no loop: %s\n', ME.message);
end

%% Encerramento
fprintf('Encerrando...\n');
send_neutral_drone_cmd(msg_cmdvel, pub_cmdvel, cfg);
if cfg.command_limo
    send_zero_limo_cmd(msg_cmdvel_limo, pub_cmdvel_limo);
end
pause(0.5);

if emergency_kill
    fprintf('Enviando kill.\n');
    try
        call(kill_client, kill_request, 'Timeout', 5);
    catch ME
        warning('Kill falhou: %s', ME.message);
    end
else
    fprintf('Enviando land.\n');
    try
        call(land_client, land_request, 'Timeout', 5);
    catch ME
        warning('Land falhou: %s', ME.message);
    end
end

pause(1.0);
rosshutdown;

if cfg.save_results && ~isempty(hist_t)
    save_drone_results(hist_t, hist_pos, hist_yaw, hist_z_err, hist_rho, hist_rho_err, cfg);
end

fprintf('Teste drone finalizado.\n');

%% Funções locais

function validate_drone_cfg(cfg)
    valid_modes = {'monitor', 'hover', 'teleop', 'formation'};
    if ~any(strcmp(cfg.mode, valid_modes))
        error('cfg.mode inválido: %s', cfg.mode);
    end
    if cfg.command_limo
        warning('command_limo=true: use apenas no main.m ou teste integrado.');
    end
end

function tf = mode_requires_takeoff(mode)
    tf = any(strcmp(mode, {'hover', 'teleop', 'formation'}));
end

function tf = is_motion_mode(mode)
    tf = any(strcmp(mode, {'hover', 'teleop', 'formation'}));
end

function v_state = drone_inner_loop(v_d, v_state, cfg)
    u_control = v_d + cfg.kd_drone * (v_d - v_state);
    v_state(1:3) = clamp_vec( ...
        v_state(1:3) + cfg.T * cfg.k_inner_drone * (u_control(1:3) - v_state(1:3)), ...
        cfg.v_max_state);
    v_state(3) = clamp_scalar(v_state(3), cfg.v_max_drone_z);
    v_state(4) = v_d(4);
end

function [pos, yaw, ok] = read_limo_pose_for_formation(sub_limo, cfg)
    ok = false;
    pos = cfg.limo_ic;
    yaw = cfg.limo_ic_yaw;

    if cfg.use_virtual_limo_ic
        ok = true;
        return;
    end

    if isempty(sub_limo)
        return;
    end

    [pos, yaw, ok] = read_optitrack_pose(sub_limo);
end

function [qd, qd_dot] = desired_formation(t, cfg)
    phase_x = 2.0 * pi * t / 40.0;
    phase_y = 4.0 * pi * t / 40.0;
    qd = [0.75 * sin(phase_x); ...
          0.75 * sin(phase_y); ...
          0.0; ...
          cfg.rho_f; ...
          cfg.alpha_f; ...
          cfg.beta_f];
    qd_dot = [0.75 * (2.0 * pi / 40.0) * cos(phase_x); ...
              0.75 * (4.0 * pi / 40.0) * cos(phase_y); ...
              0.0; ...
              0.0; ...
              0.0; ...
              0.0];
end

function q_r = apply_obstacle_null_space(q_r, poi_xy, cfg)
    offset = poi_xy - cfg.obstacle_center;
    distance = norm(offset);
    influence_r = obstacle_influence_radius(cfg);

    if distance >= influence_r || distance <= 1e-6
        return;
    end

    direction = offset / distance;
    J_obs = direction.';
    J_obs_pinv = direction;

    clearance = distance - cfg.obstacle_radius;
    if clearance <= 0.0
        obstacle_rate = 0.8;
    else
        obstacle_rate = 0.4 * (1.0 / clearance - 1.0 / (influence_r - cfg.obstacle_radius));
    end

    primary_velocity = J_obs_pinv * obstacle_rate;
    null_projector = eye(2) - J_obs_pinv * J_obs;
    q_r(1:2) = primary_velocity + null_projector * q_r(1:2);
end

function influence_r = obstacle_influence_radius(cfg)
    if isfield(cfg, 'obstacle_influence_radius') && ~isempty(cfg.obstacle_influence_radius)
        influence_r = cfg.obstacle_influence_radius;
    elseif isfield(cfg, 'obstacle_influence') && ~isempty(cfg.obstacle_influence)
        influence_r = cfg.obstacle_influence;
    else
        influence_r = 0.50;
    end
end

function [position, yaw, ok] = read_optitrack_pose(pose_subscriber)
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

function [position, yaw, ok, info] = wait_for_optitrack_pose(sub_pose, pose_topic, timeout_sec)
    ok = false;
    position = [0.0; 0.0; 0.0];
    yaw = 0.0;
    info = '';

    pause(2.0);

    topics = list_ros_topics();
    if ~any(strcmp(topics, pose_topic))
        info = sprintf('Tópico %s ausente no master.', pose_topic);
    else
        info = sprintf('Tópico %s listado.', pose_topic);
    end

    try
        [pose_msg, ~, ~] = receive(sub_pose, timeout_sec);
        [position, yaw, ok] = parse_pose_stamped(pose_msg);
        if ok
            info = [info, ' receive() OK.'];
            return;
        end
    catch ME
        info = [info, ' receive(): ', ME.message];
    end

    deadline = tic;
    while toc(deadline) < timeout_sec
        [position, yaw, ok] = read_optitrack_pose(sub_pose);
        if ok
            info = [info, ' LatestMessage OK.'];
            return;
        end
        pause(0.25);
    end

    info = [info, sprintf(' Sem dados em %.0f s.', timeout_sec)];
end

function topics = list_ros_topics()
    topics = {};
    try
        topics = rosTopicList.Name;
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

function u = velocity_to_crazyflie_cmd(v_body, cfg)
    phi = clamp_scalar(cfg.k_attitude * v_body(2), cfg.max_phi);
    theta = clamp_scalar(cfg.k_attitude * v_body(1), cfg.max_theta);
    zdot = clamp_scalar(v_body(3), cfg.max_zdot);
    psidot = clamp_scalar(v_body(4), cfg.max_psidot);
    u = [phi; theta; zdot; psidot];
end

function send_drone_cmd(msg, pub, u)
    msg.Linear.X = 0.0;
    msg.Linear.Y = 0.0;
    msg.Linear.Z = u(3);
    msg.Angular.X = u(1);
    msg.Angular.Y = u(2);
    msg.Angular.Z = u(4);
    send(pub, msg);
end

function send_neutral_drone_cmd(msg, pub, cfg)
    u = velocity_to_crazyflie_cmd([0.0; 0.0; 0.0; 0.0], cfg);
    send_drone_cmd(msg, pub, u);
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

function send_zero_limo_cmd(msg, pub)
    send_limo_cmd(msg, pub, 0.0, 0.0);
end

function pressed = is_button_pressed(Digital, button_index)
    pressed = false;
    if button_index >= 1 && numel(Digital) >= button_index
        pressed = logical(Digital(button_index));
    end
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
    if sign_raw == 0
        sign_raw = 1;
    end
    scaled = sign_raw * (abs(raw) - deadzone) / (1.0 - deadzone);
end

function y = clamp_vec(x, limit)
    y = min(max(x, -limit), limit);
end

function y = clamp_scalar(x, limit)
    y = min(max(x, -limit), limit);
end

function save_drone_results(hist_t, hist_pos, hist_yaw, hist_z_err, hist_rho, hist_rho_err, cfg)
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    out_dir = cfg.results_dir;
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    prefix = sprintf('drone_%s_%s', cfg.mode, stamp);
    mat_file = fullfile(out_dir, [prefix, '.mat']);
    fig_file = fullfile(out_dir, [prefix, '_plot.png']);

    results.meta.timestamp = stamp;
    results.meta.mode = cfg.mode;
    results.hist_t = hist_t;
    results.hist_pos = hist_pos;
    results.hist_yaw = hist_yaw;
    results.hist_z_err = hist_z_err;
    results.hist_rho = hist_rho;
    results.hist_rho_err = hist_rho_err;
    results.cfg = cfg;
    save(mat_file, '-struct', 'results');

    fig = figure('Name', 'Drone test', 'Visible', 'off');
    if any(strcmp(cfg.mode, {'hover', 'formation'})) && ~isempty(hist_z_err)
        subplot(2, 1, 1);
        plot(hist_t, hist_pos(3, :), 'b-', 'LineWidth', 1.2);
        hold on;
        yline(cfg.drone_altitude, 'r--');
        grid on;
        ylabel('Z (m)');
        title(sprintf('Altitude — %s', cfg.mode));
        subplot(2, 1, 2);
        plot(hist_t, hist_z_err, 'k-');
        grid on;
        xlabel('Tempo (s)');
        ylabel('Erro z (m)');
        if strcmp(cfg.mode, 'formation') && ~isempty(hist_rho)
            yyaxis right;
            plot(hist_t, hist_rho, 'g--');
            ylabel('rho (m)');
        end
    else
        plot(hist_pos(1, :), hist_pos(2, :), 'b-');
        axis equal;
        grid on;
        xlabel('X (m)');
        ylabel('Y (m)');
        title(sprintf('Trajetória XY — %s', cfg.mode));
    end
    print(fig, fig_file, '-dpng', '-r150');
    close(fig);

    fprintf('Resultados em %s\n  %s\n  %s\n', out_dir, mat_file, fig_file);
end
