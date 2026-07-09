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
%  'spin'       — gira por N voltas (4WD: no eixo; car-like: curva mínima)
%  'lemniscate' — segue a lemniscata (figura-8) do enunciado no PoI do LIMO
%
% ===== Modo mecânico do LIMO (refence.m) =====
%  '4wd'     — luzes amarelas; aceita v=0 + ω (giro no próprio eixo)
%  'carlike' — luzes verdes; raio mínimo ~0.4 m (v acoplado a ω quando v≈0)
%  'omni'    — LIMO 105 + use_mcnamu:=true; permite Linear.Y
%  OBS.: o enunciado define o LIMO como "robô terrestre diferencial" —
%  usar '4wd'. A lei de controle da lemniscata assume modelo uniciclo
%  (v e w independentes), que só é válido em modo diferencial.

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
cfg.mode = 'monitor';            % 'monitor' | 'teleop' | 'pulse' | 'spin' | 'lemniscate'
cfg.t_final = 80;                % duração no modo lemniscate (s); 80 s = 2 períodos de 40 s

cfg.a1 = 0.10;                   % PoI do LIMO (m)
cfg.kq = 1.2;                    % ganho proporcional (laço externo, modo lemniscate)
cfg.lq = 0.30;                   % saturação tanh (m/s) — reduzido em relação ao
                                  % main.m (0.8): com a1=0.10 a conversão p/ (v,w)
                                  % amplifica por 1/a1 a componente transversal, e
                                  % 0.8 m/s exigiria w muito acima do fisicamente ok

% Compensador dinâmico do LIMO (laço interno) — parâmetros do enunciado
cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];
cfg.kd_limo = 4.0;               % ganho do compensador dinâmico

% Limites conservadores para teste no lab
cfg.v_max = 0.30;                % m/s
cfg.w_max = 1.20;                % rad/s (era 0.50 — insuficiente p/ o transiente
                                  % inicial + 1/a1; ajuste conforme validado em bancada)
cfg.limo_steering_mode = '4wd';  % '4wd' | 'carlike' | 'omni'
                                  % ATENÇÃO: enunciado descreve o LIMO como robô
                                  % diferencial. Em 'carlike' o robô não gira no
                                  % próprio eixo e a lei de controle (que assume
                                  % modelo uniciclo) fica incoerente com a
                                  % cinemática real, distorcendo a trajetória.
cfg.ackermann_min_radius = 0.40;     % m (manual AgileX; modo car-like)

% Joystick (JoyControl) — ajuste os índices se necessário
cfg.joystick_axis_linear = 2;    % eixo analógico -> velocidade linear
cfg.joystick_axis_angular = 3;   % eixo analógico -> velocidade angular
cfg.joystick_deadzone = 0.15;
cfg.joystick_stop_button = 1;    % botão digital -> parar e sair

% Sequência automática (modo 'pulse')
cfg.pulse_linear_speed = 0.15;   % m/s
cfg.pulse_angular_speed = 0.25;  % rad/s
cfg.pulse_duration = 2.0;        % s por etapa

% Rotação no próprio eixo (modo 'spin')
cfg.spin_angular_speed = 0.25;   % rad/s (+ anti-horário, − horário)
cfg.spin_turns = 1.0;            % voltas completas (360°); sinal = sentido

% Obstáculo (modo lemniscate — mesmo do enunciado)
cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence = 0.50;
cfg.use_obstacle_avoidance = true;
cfg.pose_timeout = 30;           % segundos aguardando primeira pose

fprintf('=== Teste LIMO | modo: %s | steering: %s ===\n', ...
    cfg.mode, cfg.limo_steering_mode);
fprintf('Botão %d: parar e encerrar.\n', cfg.joystick_stop_button);
if strcmp(cfg.limo_steering_mode, 'carlike')
    fprintf(['[AVISO] Car-like: giro no próprio eixo impossível; ', ...
        'v acoplado a ω quando v≈0 (R_min=%.2f m).\n'], cfg.ackermann_min_radius);
end
if strcmp(cfg.mode, 'spin') && strcmp(cfg.limo_steering_mode, 'carlike')
    fprintf('[AVISO] Modo spin em car-like: trajetória em curva fechada, não rotação pura.\n');
end

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

%% Aguardar primeira pose (antes do joystick — evita interferência)
pose_topic = sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.limo_namespace);
fprintf('Aguardando pose do OptiTrack em %s (timeout %d s)...\n', ...
    pose_topic, cfg.pose_timeout);

[pos, yaw, pose_ok, pose_info] = wait_for_limo_pose(sub_pose, pose_topic, cfg.pose_timeout);

if pose_ok
    fprintf('Pose OK: x=%.3f y=%.3f yaw=%.1f deg\n', pos(1), pos(2), rad2deg(yaw));
else
    send_zero_cmd(msg_cmdvel, pub_cmdvel);
    rosshutdown;
    error(['Não foi possível ler pose do LIMO em %s.\n\n', ...
        'Diagnóstico: %s\n\n', ...
        'Verifique:\n', ...
        '  1) natnet_ros rodando no rosserver (.100)\n', ...
        '  2) L1 visível no Motive (marcadores ativos)\n', ...
        '  3) No rosserver: rostopic echo %s\n', ...
        '  4) No MATLAB (após rosinit): rostopic(''list'') deve listar %s\n', ...
        '  5) PC do MATLAB (.101) na mesma rede; ROS_IP = IP deste PC\n'], ...
        pose_topic, pose_info, pose_topic, pose_topic);
end

J = JoyControl;
fprintf('Joystick conectado.\n');

if strcmp(cfg.mode, 'pulse') || strcmp(cfg.mode, 'spin') || strcmp(cfg.mode, 'lemniscate')
    if strcmp(cfg.mode, 'pulse')
        msg = 'Modo pulse: o LIMO fará movimentos curtos.';
    elseif strcmp(cfg.mode, 'spin')
        if strcmp(cfg.limo_steering_mode, 'carlike')
            msg = sprintf(['Modo spin (car-like): curva mínima R=%.2f m, ', ...
                '%.1f volta(s), ω=%.2f rad/s.'], ...
                cfg.ackermann_min_radius, cfg.spin_turns, cfg.spin_angular_speed);
        else
            msg = sprintf(['Modo spin: o LIMO girará no próprio eixo (v=0) ', ...
                'por %.1f volta(s) a %.2f rad/s.'], cfg.spin_turns, cfg.spin_angular_speed);
        end
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
spin_yaw_prev = [];
spin_angle_accum = 0.0;
log_counter = 0;
hist_t = [];
hist_poi = [];
hist_ref = [];
hist_error_xy = [];
v_limo_state = [0.0; 0.0];       % estado interno do compensador dinâmico [v; w]

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

            case 'spin'
                [v_cmd, w_cmd, spin_yaw_prev, spin_angle_accum, running] = spin_step( ...
                    psi1, spin_yaw_prev, spin_angle_accum, running, cfg);

            case 'lemniscate'
                [v_d, ref_xy, err_xy] = lemniscate_outer_loop(t, poi, psi1, cfg);
                v_limo_state = limo_inner_loop(v_d, v_limo_state, cfg);
                v_cmd = v_limo_state(1);
                w_cmd = v_limo_state(2);
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

        [v_cmd, w_cmd] = apply_steering_kinematics(v_cmd, w_cmd, cfg);
        send_limo_cmd(msg_cmdvel, pub_cmdvel, v_cmd, w_cmd, cfg.limo_steering_mode);

        log_counter = log_counter + 1;
        if mod(log_counter, 30) == 0
            if strcmp(cfg.mode, 'lemniscate')
                fprintf('t=%5.1fs | PoI=(%+.3f,%+.3f) ref=(%+.3f,%+.3f) err=%+.3f m | v=%+.2f w=%+.2f\n', ...
                    t, poi(1), poi(2), ref_xy(1), ref_xy(2), norm(err_xy), v_cmd, w_cmd);
            elseif strcmp(cfg.mode, 'spin')
                fprintf('t=%5.1fs | x=%+.3f y=%+.3f yaw=%+6.1f° | giro=%+.1f° | v=%+.2f w=%+.2f\n', ...
                    t, pos(1), pos(2), rad2deg(yaw), rad2deg(spin_angle_accum), v_cmd, w_cmd);
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

function [position, yaw, ok, info] = wait_for_limo_pose(sub_pose, pose_topic, timeout_sec)
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
        [position, yaw, ok] = read_optitrack_pose(sub_pose);
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

function send_limo_cmd(msg, pub, v, w, steering_mode)
    msg.Linear.X = v;
    msg.Linear.Y = 0.0;
    msg.Linear.Z = 0.0;
    msg.Angular.X = 0.0;
    msg.Angular.Y = 0.0;
    msg.Angular.Z = w;
    if strcmp(steering_mode, 'omni')
        % Reservado para LIMO omnidirecional (Linear.Y via segundo eixo).
        msg.Linear.Y = 0.0;
    end
    send(pub, msg);
end

function [v_out, w_out] = apply_steering_kinematics(v, w, cfg)
    v_out = v;
    w_out = w;

    if ~strcmp(cfg.limo_steering_mode, 'carlike')
        return;
    end

    if abs(w_out) > 1e-6 && abs(v_out) < 1e-6
        v_out = sign(w_out) * abs(w_out) * cfg.ackermann_min_radius;
        v_out = clamp_scalar(v_out, cfg.v_max);
    end
end

function send_zero_cmd(msg, pub)
    send_limo_cmd(msg, pub, 0.0, 0.0, '4wd');
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

function [v, w, yaw_prev, angle_accum, running] = spin_step(psi, yaw_prev, angle_accum, running, cfg)
    v = 0.0;

    turn_sign = sign(cfg.spin_turns);
    if turn_sign == 0.0
        turn_sign = sign(cfg.spin_angular_speed);
        if turn_sign == 0.0
            turn_sign = 1.0;
        end
    end
    w = turn_sign * clamp_scalar(abs(cfg.spin_angular_speed), cfg.w_max);

    if isempty(yaw_prev)
        yaw_prev = psi;
        fprintf('[spin] Início — alvo: %.1f volta(s), w=%+.2f rad/s\n', ...
            cfg.spin_turns, w);
        return;
    end

    delta = wrapToPi(psi - yaw_prev);
    angle_accum = angle_accum + delta;
    yaw_prev = psi;

    target_rad = abs(cfg.spin_turns) * 2.0 * pi;
    if target_rad <= 0.0
        return;
    end

    if abs(angle_accum) >= target_rad
        v = 0.0;
        w = 0.0;
        running = false;
        fprintf('[spin] Concluído: %.1f° acumulados (%.2f volta(s)).\n', ...
            rad2deg(angle_accum), angle_accum / (2.0 * pi));
    end
end

function [ref_xy, ref_xy_dot] = lemniscata_reference(t)
    phase_x = 2.0 * pi * t / 40.0;
    phase_y = 4.0 * pi * t / 40.0;

    ref_xy = [0.75 * sin(phase_x); 0.75 * sin(phase_y)];
    ref_xy_dot = [0.75 * (2.0 * pi / 40.0) * cos(phase_x); ...
                  0.75 * (4.0 * pi / 40.0) * cos(phase_y)];
end

function [v_d, ref_xy, err_xy] = lemniscate_outer_loop(t, poi, psi, cfg)
    % Laço externo: controlador cinemático da formação (PoI do LIMO
    % rastreando a lemniscata) + desvio de obstáculo em espaço nulo +
    % inversão do jacobiano do ponto de interesse (A1). Retorna a
    % velocidade de referência v_d=[v;w] já saturada nos limites físicos.
    [ref_xy, ref_xy_dot] = lemniscata_reference(t);
    err_xy = ref_xy - poi;

    vel_poi = ref_xy_dot + cfg.lq * tanh((cfg.kq / cfg.lq) * err_xy);

    if cfg.use_obstacle_avoidance
        vel_poi = apply_obstacle_null_space_xy(vel_poi, poi, cfg);
    end

    A1_inv = [cos(psi), sin(psi); ...
              -sin(psi) / cfg.a1, cos(psi) / cfg.a1];
    u = A1_inv * vel_poi;

    v_d = [clamp_scalar(u(1), cfg.v_max); clamp_scalar(u(2), cfg.w_max)];
end

function v_state = limo_inner_loop(v_d, v_state, cfg)
    % Laço interno: compensador dinâmico do LIMO (regressão linear nos
    % parâmetros theta_limo do enunciado), integrado a T=1/30 s. v_state
    % é o estado interno [v; w] usado como referência de velocidade
    % suavizada, coerente com a dinâmica real do robô, enviada ao cmd_vel.
    u_real = v_state(1);
    w_real = v_state(2);

    Y1 = [u_real, 0.0, w_real^2, 0.0, 0.0, 0.0; ...
          0.0, w_real, 0.0, u_real, u_real * w_real, w_real];

    KD = diag([cfg.kd_limo, cfg.kd_limo]);
    u_control = Y1 * cfg.theta_limo + KD * (v_d - v_state);

    M1 = [cfg.theta_limo(1), 0.0; 0.0, cfg.theta_limo(2)];
    C1 = [cfg.theta_limo(4) * u_real, cfg.theta_limo(3) * w_real; ...
          cfg.theta_limo(5) * u_real + cfg.theta_limo(6) * w_real, 0.0];

    v_dot = M1 \ (u_control - C1 * v_state);
    v_state = v_state + cfg.T * v_dot;

    v_state(1) = clamp_scalar(v_state(1), cfg.v_max);
    v_state(2) = clamp_scalar(v_state(2), cfg.w_max);
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
