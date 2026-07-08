%% main.m - Controle de Estrutura Virtual (LIMO + Crazyflie) — LAB-AIR
% Robótica Móvel 2026/1 - UFES
%
% Versão para testes em hardware real, baseada em refence.m (código validado
% pelo professor) e na mesma lógica de controle do simulador Python.
%
% ===== ANTES DE RODAR O MATLAB =====
%
% 1) Motive: criar corpos rígidos L1 (LIMO) e cfX (Crazyflie, ex.: cf7).
% 2) Terminal ROS:
%      roslaunch natnet_ros_cpp natnet_ros.launch
%      roslaunch crazyflie_server crazyflie_server.launch cfs:=[X]
% 3) LIMO (SSH agilex@192.168.0.XXX, senha agx):
%      roslaunch limo_base limo_base.launch namespace:=L1
%    (4wd: luz amarela | car-like: luz verde | omni: LIMO 105 + use_mcnamu)
% 4) Ajuste cfg.limo_namespace, cfg.drone_namespace e cfg.ros_ip abaixo.
% 5) Tenha JoyControl.m no path do MATLAB.

clear;
clc;
close all;

%% ===================== CONFIGURAÇÃO =====================
% Rede LAB-AIR:
%   ROS master: 192.168.0.100:11311  (MATLAB rosinit)
%   LIMO onboard: 192.168.0.104      (SSH; nó registra-se no master)
%   Motive/NatNet: 192.168.0.101
cfg.ros_master_host = '192.168.0.100';
cfg.ros_master_port = 11311;
cfg.limo_host = '192.168.0.104';
cfg.limo_namespace = 'L1';
cfg.drone_namespace = 'cf7';
cfg.pose_topic_prefix = '/natnet_ros';

cfg.T = 1 / 30;                % 30 Hz (especificação)
cfg.t_final = 100;             % duração do experimento (s)
cfg.takeoff_pause = 5;         % espera após takeoff (s)

cfg.a1 = 0.10;                 % PoI do LIMO (m)
cfg.kq = 1.2;
cfg.lq = 0.8;
cfg.kd_limo = 4.0;

cfg.rho_f = 1.5;
cfg.beta_f = pi / 2;
cfg.drone_altitude = 1.5;      % altitude alvo (m)

cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence = 0.50;

cfg.v_max_limo = 2.0;
cfg.v_max_drone_xy = 2.0;
cfg.v_max_drone_z = 1.0;
cfg.v_max_state = 3.0;

% Limites Crazyflie (refence.m / enunciado: arfagem e rolagem <= 5°)
cfg.max_phi = deg2rad(5);
cfg.max_theta = deg2rad(5);
cfg.max_zdot = 1.0;
cfg.max_psidot = 1.0;          % valor conservador para laboratório
cfg.k_attitude = cfg.max_theta / cfg.v_max_drone_xy;

cfg.limo_steering_mode = 'carlike';  % '4wd' | 'carlike' | 'omni'
cfg.ackermann_min_radius = 0.40;     % m (manual AgileX; modo car-like)
cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];

cfg.joystick_stop_button = 1;  % índice em J.pDigital (JoyControl)
cfg.joystick_kill_button = 2;  % botão extra para kill de emergência

Kq = diag([cfg.kq, cfg.kq, cfg.kq * 0.83, cfg.kq * 1.25, cfg.kq * 0.83, cfg.kq]);
Lq = diag([cfg.lq, cfg.lq, cfg.lq * 0.62, cfg.lq * 1.25, cfg.lq * 0.62, cfg.lq]);
KD_LIMO = diag([cfg.kd_limo, cfg.kd_limo]);

%% ===================== ROS (refence.m) =====================
rosshutdown;
rosinit(sprintf('http://%s:%d', cfg.ros_master_host, cfg.ros_master_port));

% LIMO: cmd_vel
pub_cmdvel_limo = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.limo_namespace), 'geometry_msgs/Twist');
msg_cmdvel_limo = rosmessage(pub_cmdvel_limo);

% Crazyflie: cmd_vel
pub_cmdvel_drone = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.drone_namespace), 'geometry_msgs/Twist');
msg_cmdvel_drone = rosmessage(pub_cmdvel_drone);

% Crazyflie: serviços takeoff / land / kill
takeoff_client = rossvcclient( ...
    sprintf('/%s/takeoff', cfg.drone_namespace), 'std_srvs/Trigger');
takeoff_request = rosmessage(takeoff_client);

land_client = rossvcclient( ...
    sprintf('/%s/land', cfg.drone_namespace), 'std_srvs/Trigger');
land_request = rosmessage(land_client);

kill_client = rossvcclient( ...
    sprintf('/%s/kill', cfg.drone_namespace), 'std_srvs/Trigger');
kill_request = rosmessage(kill_client);

% Poses via natnet_ros + OptiTrack (PoseStamped)
sub_pose_limo = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.limo_namespace), ...
    'geometry_msgs/PoseStamped');
sub_pose_drone = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.drone_namespace), ...
    'geometry_msgs/PoseStamped');

% Joystick (refence.m)
J = JoyControl;

%% ===================== DECOLAGEM =====================
fprintf('Chamando serviço takeoff do Crazyflie (%s)...\n', cfg.drone_namespace);
takeoff_response = call(takeoff_client, takeoff_request, 'Timeout', 5);
if ~takeoff_response.Success
    error('Takeoff falhou: %s', takeoff_response.Message);
end
pause(cfg.takeoff_pause);

%% ===================== ESTADO DO CONTROLADOR =====================
v_limo = [0.0; 0.0];              % [v; w]
v_drone = [0.0; 0.0; 0.0; 0.0];   % [vx; vy; vz; psidot] no corpo

hist_t = [];
hist_error = [];

t0 = tic;
running = true;
emergency_kill = false;

fprintf('Loop de controle a %.1f Hz. Botão %d: parar | Botão %d: kill.\n', ...
    1 / cfg.T, cfg.joystick_stop_button, cfg.joystick_kill_button);
if strcmp(cfg.limo_steering_mode, 'carlike')
    fprintf(['[AVISO] LIMO car-like: comandos v≈0 + ω serão acoplados ', ...
        '(R_min=%.2f m).\n'], cfg.ackermann_min_radius);
end

try
    while running && toc(t0) < cfg.t_final
        loop_start = tic;
        t = toc(t0);

        mRead(J);
        Digital = J.pDigital;

        if Digital(cfg.joystick_kill_button)
            fprintf('Kill de emergência acionado pelo joystick.\n');
            emergency_kill = true;
            break;
        end
        if Digital(cfg.joystick_stop_button)
            fprintf('Parada solicitada pelo joystick.\n');
            break;
        end

        [pos_limo, yaw_limo, ok_limo] = read_optitrack_pose(sub_pose_limo);
        [pos_drone, yaw_drone, ok_drone] = read_optitrack_pose(sub_pose_drone);

        if ~ok_limo || ~ok_drone
            warning('Pose indisponível. Enviando comandos neutros.');
            send_neutral_commands(msg_cmdvel_limo, pub_cmdvel_limo, ...
                msg_cmdvel_drone, pub_cmdvel_drone, cfg);
            pause(cfg.T);
            continue;
        end

        x1 = pos_limo(1);
        y1 = pos_limo(2);
        psi1 = yaw_limo;

        x2 = pos_drone(1);
        y2 = pos_drone(2);
        z2 = pos_drone(3);
        psi2 = yaw_drone;

        poi_limo = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1); 0.0];
        poi_drone = [x2; y2; z2];

        delta = poi_drone - poi_limo;
        dist_2d = sqrt(delta(1)^2 + delta(2)^2);

        rho = max(dist_2d, 1e-3);
        alpha = 0.0;
        beta = atan2(delta(2), delta(1));

        q = [poi_limo(1); poi_limo(2); poi_limo(3); rho; alpha; beta];
        [qd, qd_dot] = desired_formation(t, cfg);

        error_q = qd - q;
        error_q(6) = atan2(sin(qd(6) - q(6)), cos(qd(6) - q(6)));

        tanh_term = tanh(Lq \ (Kq * error_q));
        q_r = qd_dot + Lq * tanh_term;
        q_r = apply_obstacle_null_space(q_r, poi_limo(1:2), cfg);

        S = [cos(alpha) * cos(beta), -rho * sin(alpha) * cos(beta), -rho * cos(alpha) * sin(beta); ...
             cos(alpha) * sin(beta), -rho * sin(alpha) * sin(beta),  rho * cos(alpha) * cos(beta); ...
             sin(alpha),              rho * cos(alpha),               0.0];

        J_inv = [eye(3), zeros(3); eye(3), S];
        x_r = J_inv * q_r;

        A1_inv = [cos(psi1), sin(psi1); ...
                  -sin(psi1) / cfg.a1, cos(psi1) / cfg.a1];
        v_d_limo = clamp_vec(A1_inv * x_r(1:2), cfg.v_max_limo);

        R_yaw = [cos(psi2), sin(psi2), 0.0; ...
                 -sin(psi2), cos(psi2), 0.0; ...
                  0.0, 0.0, 1.0];
        v_d_drone_body = R_yaw * x_r(4:6);
        v_d_drone_body(1:2) = clamp_vec(v_d_drone_body(1:2), cfg.v_max_drone_xy);
        v_d_drone_body(3) = clamp_scalar( ...
            v_d_drone_body(3) + 2.0 * (cfg.drone_altitude - z2), cfg.v_max_drone_z);
        v_d_drone = [v_d_drone_body; 0.0];

        % Laço interno LIMO
        u_real = v_limo(1);
        w_real = v_limo(2);
        Y1 = [u_real, 0.0, w_real^2, 0.0, 0.0, 0.0; ...
              0.0, w_real, 0.0, u_real, u_real * w_real, w_real];
        u_control_limo = Y1 * cfg.theta_limo + KD_LIMO * (v_d_limo - v_limo);

        M1 = [cfg.theta_limo(1), 0.0; 0.0, cfg.theta_limo(2)];
        C1 = [cfg.theta_limo(4) * u_real, cfg.theta_limo(3) * w_real; ...
              cfg.theta_limo(5) * u_real + cfg.theta_limo(6) * w_real, 0.0];
        v_dot_limo = M1 \ (u_control_limo - C1 * v_limo);
        v_limo = clamp_vec(v_limo + cfg.T * v_dot_limo, cfg.v_max_state);

        % Laço interno drone (velocidades desejadas -> integrador interno)
        u_control_drone = v_d_drone + 1.0 * (v_d_drone - v_drone);
        v_drone(1:3) = clamp_vec( ...
            v_drone(1:3) + cfg.T * 15.0 * (u_control_drone(1:3) - v_drone(1:3)), cfg.v_max_state);
        v_drone(3) = clamp_scalar(v_drone(3), cfg.v_max_drone_z);

        % LIMO: u = [v; w] via cmd_vel (refence.m)
        [v_send, w_send] = apply_steering_kinematics(v_limo(1), v_limo(2), cfg);
        msg_cmdvel_limo.Linear.X = v_send;
        msg_cmdvel_limo.Linear.Y = 0.0;
        msg_cmdvel_limo.Linear.Z = 0.0;
        msg_cmdvel_limo.Angular.X = 0.0;
        msg_cmdvel_limo.Angular.Y = 0.0;
        msg_cmdvel_limo.Angular.Z = w_send;
        if strcmp(cfg.limo_steering_mode, 'omni')
            msg_cmdvel_limo.Linear.Y = 0.0; %#ok<*UNRCH>
        end
        send(pub_cmdvel_limo, msg_cmdvel_limo);

        % Crazyflie: u = [phi; theta; zdot; psidot] via cmd_vel (refence.m)
        u_drone = velocity_to_crazyflie_cmd(v_drone, cfg);
        msg_cmdvel_drone.Linear.X = 0.0;
        msg_cmdvel_drone.Linear.Y = 0.0;
        msg_cmdvel_drone.Linear.Z = u_drone(3);
        msg_cmdvel_drone.Angular.X = u_drone(1);
        msg_cmdvel_drone.Angular.Y = u_drone(2);
        msg_cmdvel_drone.Angular.Z = u_drone(4);
        send(pub_cmdvel_drone, msg_cmdvel_drone);

        hist_t(end + 1, 1) = t; %#ok<AGROW>
        hist_error(:, end + 1) = error_q; %#ok<AGROW>

        if mod(numel(hist_t), 30) == 0
            fprintf('t=%6.2f s | erro rho=%+.3f m | erro beta=%+.1f deg\n', ...
                t, error_q(4), rad2deg(error_q(6)));
        end

        elapsed = toc(loop_start);
        pause(max(0.0, cfg.T - elapsed));
    end
catch ME
    fprintf('Erro durante o loop: %s\n', ME.message);
end

%% ===================== ENCERRAMENTO =====================
fprintf('Encerrando experimento...\n');
send_neutral_commands(msg_cmdvel_limo, pub_cmdvel_limo, msg_cmdvel_drone, pub_cmdvel_drone, cfg);

if emergency_kill
    fprintf('Enviando kill ao Crazyflie.\n');
    try
        call(kill_client, kill_request, 'Timeout', 5);
    catch kill_error
        warning('Falha ao chamar kill: %s', kill_error.message);
    end
else
    fprintf('Enviando land ao Crazyflie.\n');
    try
        call(land_client, land_request, 'Timeout', 5);
    catch land_error
        warning('Falha ao chamar land: %s', land_error.message);
    end
end

pause(2);
rosshutdown;

if ~isempty(hist_t)
    figure('Name', 'Erros da formação');
    plot(hist_t, hist_error(1, :), 'r', 'DisplayName', 'Erro Xf');
    hold on;
    plot(hist_t, hist_error(2, :), 'g', 'DisplayName', 'Erro Yf');
    plot(hist_t, hist_error(4, :), 'b', 'DisplayName', 'Erro Rho');
    plot(hist_t, rad2deg(hist_error(6, :)), 'Color', [0.92 0.35 0.05], 'DisplayName', 'Erro Beta (deg)');
    grid on;
    xlabel('Tempo (s)');
    ylabel('Erro');
    legend('Location', 'best');
    title('Controle em hardware - erros da formação');
end

fprintf('Execução finalizada.\n');

%% ===================== FUNÇÕES LOCAIS =====================
function [qd, qd_dot] = desired_formation(t, cfg)
    phase_x = 2.0 * pi * t / 40.0;
    phase_y = 4.0 * pi * t / 40.0;

    qd = [0.75 * sin(phase_x); ...
          0.75 * sin(phase_y); ...
          0.0; ...
          cfg.rho_f; ...
          0.0; ...
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

    if distance >= cfg.obstacle_influence || distance <= 1e-6
        return;
    end

    direction = offset / distance;
    J_obs = direction.';
    J_obs_pinv = direction;

    clearance = distance - cfg.obstacle_radius;
    if clearance <= 0.0
        obstacle_rate = 0.8;
    else
        obstacle_rate = 0.4 * (1.0 / clearance - 1.0 / (cfg.obstacle_influence - cfg.obstacle_radius));
    end

    primary_velocity = J_obs_pinv * obstacle_rate;
    null_projector = eye(2) - J_obs_pinv * J_obs;
    secondary_velocity = null_projector * q_r(1:2);

    q_r(1:2) = primary_velocity + secondary_velocity;
end

function [position, yaw, ok] = read_optitrack_pose(pose_subscriber)
    ok = false;
    position = [0.0; 0.0; 0.0];
    yaw = 0.0;

    latest = pose_subscriber.LatestMessage;
    if isempty(latest)
        return;
    end

    try
        pose_latest = latest.Pose;
        quat = [pose_latest.Orientation.W, pose_latest.Orientation.X, ...
                pose_latest.Orientation.Y, pose_latest.Orientation.Z];
        eul_zyx = quat2eul(quat); % [yaw pitch roll]
        angles = [eul_zyx(3); eul_zyx(2); eul_zyx(1)]; % sequência XYZ (refence.m)
        yaw = angles(3);

        position = [pose_latest.Position.X; ...
                    pose_latest.Position.Y; ...
                    pose_latest.Position.Z];
        ok = true;
    catch
        ok = false;
    end
end

function u = velocity_to_crazyflie_cmd(v_body, cfg)
    % Mapeia velocidades desejadas no corpo para [phi; theta; zdot; psidot].
    phi = clamp_scalar(cfg.k_attitude * v_body(2), cfg.max_phi);
    theta = clamp_scalar(cfg.k_attitude * v_body(1), cfg.max_theta);
    zdot = clamp_scalar(v_body(3), cfg.max_zdot);
    psidot = clamp_scalar(v_body(4), cfg.max_psidot);
    u = [phi; theta; zdot; psidot];
end

function send_neutral_commands(msg_limo, pub_limo, msg_drone, pub_drone, cfg)
    msg_limo.Linear.X = 0.0;
    msg_limo.Linear.Y = 0.0;
    msg_limo.Linear.Z = 0.0;
    msg_limo.Angular.X = 0.0;
    msg_limo.Angular.Y = 0.0;
    msg_limo.Angular.Z = 0.0;
    send(pub_limo, msg_limo);

    u_neutral = velocity_to_crazyflie_cmd([0.0; 0.0; 0.0; 0.0], cfg);
    msg_drone.Linear.X = 0.0;
    msg_drone.Linear.Y = 0.0;
    msg_drone.Linear.Z = u_neutral(3);
    msg_drone.Angular.X = u_neutral(1);
    msg_drone.Angular.Y = u_neutral(2);
    msg_drone.Angular.Z = u_neutral(4);
    send(pub_drone, msg_drone);
end

function [v_out, w_out] = apply_steering_kinematics(v, w, cfg)
    v_out = v;
    w_out = w;

    if ~strcmp(cfg.limo_steering_mode, 'carlike')
        return;
    end

    if abs(w_out) > 1e-6 && abs(v_out) < 1e-6
        v_out = sign(w_out) * abs(w_out) * cfg.ackermann_min_radius;
        v_out = clamp_scalar(v_out, cfg.v_max_limo);
    end
end

function y = clamp_vec(x, limit)
    y = min(max(x, -limit), limit);
end

function y = clamp_scalar(x, limit)
    y = min(max(x, -limit), limit);
end
