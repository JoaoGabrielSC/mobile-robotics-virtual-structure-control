%% test_limo.m - Teste isolado do LIMO (conexão ROS + OptiTrack + cmd_vel)
% Robótica Móvel 2026/1 - LAB-AIR
%
% Use este script ANTES do main.m para validar apenas o robô terrestre.
%
% ===== Pré-requisitos (refence.m) =====
%  1) Motive: corpo rígido nomeado L1
%  2) roslaunch natnet_ros_cpp natnet_ros.launch  (no rosserver .100)
%  3) No LIMO (ssh agilex@192.168.0.104, senha agx):
%       roslaunch limo_base limo_base.launch namespace:=L1
%  4) JoyControl.m no path do MATLAB
%
% ===== Rede =====
%  rosinit -> 192.168.0.100:11311 (master)
%  LIMO onboard -> 192.168.0.104 (não usar :34329 no MATLAB)
%
% ===== Modos =====
%  'monitor'    — só lê pose (mais seguro; primeiro teste)
%  'teleop'     — joystick comanda v e ω
%  'pulse'      — sequência curta (frente → parar → girar)
%  'lemniscate' — segue a lemniscata (figura-8) do enunciado no PoI do LIMO

clear;
clc;
close all;

%% Configuração — rede ROS
cfg.ros_master_host = '192.168.0.100';   % rosserver (roscore)
cfg.ros_master_port = 11311;             % porta padrão do master
cfg.limo_host = '192.168.0.104';         % onboard do LIMO (só SSH/launch; não usar no rosinit)
cfg.limo_namespace = 'L1';       % ATENÇÃO: L + número 1, não I (i maiúsculo)
cfg.pose_topic_prefix = '/natnet_ros';

cfg.T = 1 / 30;                  % 30 Hz
cfg.mode = 'monitor';            % 'monitor' | 'teleop' | 'pulse' | 'lemniscate'
cfg.t_final = 80;                % duração no modo lemniscate (s); 80 s = 2 períodos de 40 s

cfg.a1 = 0.10;                   % PoI do LIMO (m)
cfg.kq = 1.2;                    % ganho proporcional (modo lemniscate)
cfg.lq = 0.8;                    % saturação tanh (modo lemniscate)

% Limites conservadores para teste no lab
cfg.v_max = 0.30;                % m/s
cfg.w_max = 0.50;                % rad/s
cfg.limo_differential = true;    % false => envia também Linear.Y (omni)

% Joystick (JoyControl) — ajuste os índices se necessário
cfg.joystick_axis_linear = 2;    % eixo analógico -> velocidade linear
cfg.joystick_axis_angular = 3;   % eixo analógico -> velocidade angular
cfg.joystick_deadzone = 0.15;
cfg.joystick_stop_button = 1;    % botão digital -> parar e sair

% Sequência automática (modo 'pulse')
cfg.pulse_linear_speed = 0.15;   % m/s
cfg.pulse_angular_speed = 0.25;  % rad/s
cfg.pulse_duration = 2.0;        % s por etapa

% Obstáculo (modo lemniscate — mesmo do enunciado)
cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence = 0.50;
cfg.use_obstacle_avoidance = true;

fprintf('=== Teste LIMO | modo: %s ===\n', cfg.mode);
fprintf('Botão %d: parar e encerrar.\n', cfg.joystick_stop_button);

%% ROS (refence.m)
rosshutdown;
master_uri = sprintf('http://%s:%d', cfg.ros_master_host, cfg.ros_master_port);
fprintf('Conectando ao ROS master: %s\n', master_uri);
rosinit(master_uri);

pub_cmdvel = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.limo_namespace), 'geometry_msgs/Twist');
msg_cmdvel = rosmessage(pub_cmdvel);

sub_pose = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.limo_namespace), ...
    'geometry_msgs/PoseStamped');

J = JoyControl;

%% Aguardar primeira pose
pose_topic = sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.limo_namespace);
fprintf('Aguardando pose do OptiTrack em %s (timeout 30 s)...\n', pose_topic);

pose_ok = false;
pos = [0; 0; 0];
yaw = 0;

try
    [pose_msg, ~, ~] = receive(sub_pose, 30);
    [pos, yaw, pose_ok] = parse_pose_stamped(pose_msg);
catch
    pose_ok = false;
end

if pose_ok
    fprintf('Pose OK: x=%.3f y=%.3f yaw=%.1f deg\n', pos(1), pos(2), rad2deg(yaw));
else
    send_zero_cmd(msg_cmdvel, pub_cmdvel);
    rosshutdown;
    error(['Não foi possível ler pose do LIMO em %s.\n\n', ...
        'Verifique:\n', ...
        '  1) cfg.limo_namespace = ''L1'' (letra L, não I)\n', ...
        '  2) natnet_ros rodando no rosserver\n', ...
        '  3) corpo rígido L1 visível no Motive\n', ...
        '  4) no terminal ROS: rostopic echo %s\n'], pose_topic, pose_topic);
end

if strcmp(cfg.mode, 'pulse') || strcmp(cfg.mode, 'lemniscate')
    if strcmp(cfg.mode, 'pulse')
        msg = 'Modo pulse: o LIMO fará movimentos curtos.';
    else
        msg = sprintf(['Modo lemniscate: o PoI do LIMO seguirá a figura-8 ', ...
            'por %.0f s.\n  Referência: xd=0.75*sin(2πt/40), yd=0.75*sin(4πt/40)'], cfg.t_final);
    end
    fprintf('%s Área livre?\n', msg);
    input('Pressione Enter para continuar (Ctrl+C para cancelar)...', 's');
end

%% Loop principal
t0 = tic;
running = true;
pulse_state = 'idle';
pulse_timer = 0;
log_counter = 0;
hist_t = [];
hist_poi = [];
hist_ref = [];
hist_error_xy = [];

try
    while running
        loop_start = tic;
        t = toc(t0);

        mRead(J);
        Analog = J.pAnalog;
        Digital = J.pDigital;

        if is_stop_pressed(Digital, cfg.joystick_stop_button)
            fprintf('Parada solicitada pelo joystick.\n');
            break;
        end

        [pos, yaw, pose_ok] = read_optitrack_pose(sub_pose);

        if ~pose_ok
            warning('Pose indisponível — enviando zero.');
            send_zero_cmd(msg_cmdvel, pub_cmdvel);
            pause(cfg.T);
            continue;
        end

        x1 = pos(1);
        y1 = pos(2);
        psi1 = yaw;
        poi = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];

        switch cfg.mode
            case 'monitor'
                v_cmd = 0.0;
                w_cmd = 0.0;

            case 'teleop'
                v_cmd = cfg.v_max * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_linear), cfg.joystick_deadzone);
                w_cmd = cfg.w_max * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_angular), cfg.joystick_deadzone);

            case 'pulse'
                [v_cmd, w_cmd, pulse_state, pulse_timer, running] = pulse_step( ...
                    t, pulse_state, pulse_timer, running, cfg);

            case 'lemniscate'
                [v_cmd, w_cmd, ref_xy, err_xy] = lemniscate_control(t, poi, psi1, cfg);
                hist_t(end + 1, 1) = t; %#ok<AGROW>
                hist_poi(:, end + 1) = poi; %#ok<AGROW>
                hist_ref(:, end + 1) = ref_xy; %#ok<AGROW>
                hist_error_xy(:, end + 1) = err_xy; %#ok<AGROW>
                if t >= cfg.t_final
                    fprintf('Tempo final (%.0f s) atingido.\n', cfg.t_final);
                    running = false;
                end

            otherwise
                error('Modo desconhecido: %s', cfg.mode);
        end

        send_limo_cmd(msg_cmdvel, pub_cmdvel, v_cmd, w_cmd, cfg.limo_differential);

        log_counter = log_counter + 1;
        if mod(log_counter, 30) == 0
            if strcmp(cfg.mode, 'lemniscate')
                fprintf('t=%5.1fs | PoI=(%+.3f,%+.3f) ref=(%+.3f,%+.3f) err=%+.3f m | v=%+.2f w=%+.2f\n', ...
                    t, poi(1), poi(2), ref_xy(1), ref_xy(2), norm(err_xy), v_cmd, w_cmd);
            else
                fprintf('t=%5.1fs | x=%+.3f y=%+.3f yaw=%+6.1f° | v=%+.2f w=%+.2f\n', ...
                    t, pos(1), pos(2), rad2deg(yaw), v_cmd, w_cmd);
            end
        end

        elapsed = toc(loop_start);
        pause(max(0.0, cfg.T - elapsed));
    end
catch ME
    fprintf('Erro: %s\n', ME.message);
end

%% Encerramento
fprintf('Encerrando: velocidade zero e rosshutdown.\n');
send_zero_cmd(msg_cmdvel, pub_cmdvel);
pause(0.5);
rosshutdown;

if strcmp(cfg.mode, 'lemniscate') && ~isempty(hist_t)
    figure('Name', 'Teste lemniscate - LIMO');
    plot(hist_ref(1, :), hist_ref(2, :), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Referência');
    hold on;
    plot(hist_poi(1, :), hist_poi(2, :), 'b-', 'LineWidth', 1.2, 'DisplayName', 'PoI LIMO');
    plot(hist_poi(1, 1), hist_poi(2, 1), 'go', 'MarkerSize', 8, 'DisplayName', 'Início');
    plot(hist_poi(1, end), hist_poi(2, end), 'ks', 'MarkerSize', 8, 'DisplayName', 'Fim');
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Lemniscata de Bernoulli — PoI do LIMO');
    legend('Location', 'best');

    figure('Name', 'Erro XY - lemniscate');
    plot(hist_t, hist_error_xy(1, :), 'r', 'DisplayName', 'Erro X');
    hold on;
    plot(hist_t, hist_error_xy(2, :), 'g', 'DisplayName', 'Erro Y');
    grid on;
    xlabel('Tempo (s)');
    ylabel('Erro (m)');
    legend('Location', 'best');
    title('Erro de rastreamento da figura-8');

    rms_xy = sqrt(mean(sum(hist_error_xy.^2, 1)));
    fprintf('Erro RMS de posição (PoI): %.3f m\n', rms_xy);
end

fprintf('Teste LIMO finalizado.\n');

%% Funções locais
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

    if isempty(msg) || ~isfield(msg, 'Pose') || isempty(msg.Pose)
        return;
    end

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
end

function pressed = is_stop_pressed(Digital, button_index)
    pressed = false;
    if button_index >= 1 && numel(Digital) >= button_index
        pressed = logical(Digital(button_index));
    end
end

function send_limo_cmd(msg, pub, v, w, differential)
    msg.Linear.X = v;
    msg.Linear.Y = 0.0;
    msg.Linear.Z = 0.0;
    msg.Angular.X = 0.0;
    msg.Angular.Y = 0.0;
    msg.Angular.Z = w;
    if ~differential
        % Reservado para LIMO omnidirecional (Linear.Y via segundo eixo).
        msg.Linear.Y = 0.0;
    end
    send(pub, msg);
end

function send_zero_cmd(msg, pub)
    send_limo_cmd(msg, pub, 0.0, 0.0, true);
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

function [v, w, state, timer, running] = pulse_step(t, state, timer, running, cfg)
    v = 0.0;
    w = 0.0;

    switch state
        case 'idle'
            state = 'forward';
            timer = t;
            fprintf('[pulse] Frente por %.1f s...\n', cfg.pulse_duration);

        case 'forward'
            v = cfg.pulse_linear_speed;
            if t - timer >= cfg.pulse_duration
                state = 'stop1';
                timer = t;
                fprintf('[pulse] Parando...\n');
            end

        case 'stop1'
            if t - timer >= 1.0
                state = 'turn';
                timer = t;
                fprintf('[pulse] Girando por %.1f s...\n', cfg.pulse_duration);
            end

        case 'turn'
            w = cfg.pulse_angular_speed;
            if t - timer >= cfg.pulse_duration
                state = 'stop2';
                timer = t;
                fprintf('[pulse] Parando...\n');
            end

        case 'stop2'
            if t - timer >= 1.0
                state = 'done';
                fprintf('[pulse] Sequência concluída.\n');
            end

        case 'done'
            running = false;
    end
end

function [ref_xy, ref_xy_dot] = lemniscata_reference(t)
    phase_x = 2.0 * pi * t / 40.0;
    phase_y = 4.0 * pi * t / 40.0;

    ref_xy = [0.75 * sin(phase_x); 0.75 * sin(phase_y)];
    ref_xy_dot = [0.75 * (2.0 * pi / 40.0) * cos(phase_x); ...
                  0.75 * (4.0 * pi / 40.0) * cos(phase_y)];
end

function [v_cmd, w_cmd, ref_xy, err_xy] = lemniscate_control(t, poi, psi, cfg)
    [ref_xy, ref_xy_dot] = lemniscata_reference(t);
    err_xy = ref_xy - poi;

    vel_poi = ref_xy_dot + cfg.lq * tanh((cfg.kq / cfg.lq) * err_xy);

    if cfg.use_obstacle_avoidance
        vel_poi = apply_obstacle_null_space_xy(vel_poi, poi, cfg);
    end

    A1_inv = [cos(psi), sin(psi); ...
              -sin(psi) / cfg.a1, cos(psi) / cfg.a1];
    u = A1_inv * vel_poi;

    v_cmd = clamp_scalar(u(1), cfg.v_max);
    w_cmd = clamp_scalar(u(2), cfg.w_max);
end

function vel_xy = apply_obstacle_null_space_xy(vel_xy, poi, cfg)
    offset = poi - cfg.obstacle_center;
    distance = norm(offset);

    if distance >= cfg.obstacle_influence || distance <= 1e-6
        return;
    end

    direction = offset / distance;
    J_obs_pinv = direction;

    clearance = distance - cfg.obstacle_radius;
    if clearance <= 0.0
        obstacle_rate = 0.8;
    else
        obstacle_rate = 0.4 * (1.0 / clearance - 1.0 / (cfg.obstacle_influence - cfg.obstacle_radius));
    end

    primary_velocity = J_obs_pinv * obstacle_rate;
    null_projector = eye(2) - J_obs_pinv * direction.';
    vel_xy = primary_velocity + null_projector * vel_xy;
end

function y = clamp_scalar(x, limit)
    y = min(max(x, -limit), limit);
end
