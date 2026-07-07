%% main.m - Controle de Estrutura Virtual (LIMO + Bebop 2)
% Robótica Móvel 2026/1 - UFES / LAB-AIR
%
% Implementa a mesma arquitetura Inner-Outer Loop do simulador Python:
%   - laço externo: controlador cinemático da formação (lemniscata de Bernoulli)
%   - laço interno: compensador dinâmico do LIMO e do Bebop 2
%   - desvio de obstáculo por controle em espaço nulo (prioridade sobre a formação)
%
% Base ROS/OptiTrack conforme anexo do PDF "Especificação do projeto.pdf"
% (MATLAB 2021, tópicos cmd_vel / takeoff / land / vrpn_client_node).
%
% Antes de executar:
%   1. Ajuste ros_ip, limo_namespace e bebop_namespace abaixo.
%   2. Faça o launch dos nós ROS do LIMO e do Bebop 2.
%   3. Verifique se o OptiTrack está publicando as poses no vrpn_client_node.

clear;
clc;
close all;

%% ===================== CONFIGURAÇÃO =====================
cfg.ros_ip = '192.168.0.100';
cfg.limo_namespace = 'L1';    % exemplo do PDF
cfg.bebop_namespace = 'B1';   % exemplo do PDF

cfg.T = 1 / 30;               % período de amostragem (30 Hz)
cfg.t_final = 100;            % duração do experimento (s)
cfg.takeoff_pause = 5;        % tempo de espera após takeoff (s)

cfg.a1 = 0.10;                % deslocamento PoI do LIMO (m)
cfg.kq = 1.2;
cfg.lq = 0.8;
cfg.kd_limo = 4.0;

cfg.rho_f = 1.5;
cfg.beta_f = pi / 2;
cfg.bebop_altitude = 1.5;

cfg.obstacle_center = [-0.20; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence = 0.50;

cfg.v_max_limo = 2.0;
cfg.v_max_bebop_xy = 2.0;
cfg.v_max_bebop_z = 1.0;
cfg.v_max_state = 3.0;

cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];

cfg.joystick_stop_button = 1; % botão do vrjoystick para encerrar o loop

Kq = diag([cfg.kq, cfg.kq, cfg.kq * 0.83, cfg.kq * 1.25, cfg.kq * 0.83, cfg.kq]);
Lq = diag([cfg.lq, cfg.lq, cfg.lq * 0.62, cfg.lq * 1.25, cfg.lq * 0.62, cfg.lq]);
KD_LIMO = diag([cfg.kd_limo, cfg.kd_limo]);

%% ===================== ROS (PDF) =====================
% A interface do ROS deve estar fechada antes de ser aberta.
rosshutdown;

% Inicialização rede ROS MATLAB
rosinit(cfg.ros_ip);

% --- LIMO: cmd_vel ---
pub_cmdvel_limo = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.limo_namespace), 'geometry_msgs/Twist');
msg_cmdvel_limo = rosmessage(pub_cmdvel_limo);

% --- Bebop: cmd_vel, takeoff, land ---
pub_cmdvel_bebop = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.bebop_namespace), 'geometry_msgs/Twist');
msg_cmdvel_bebop = rosmessage(pub_cmdvel_bebop);

pub_takeoff_bebop = rospublisher( ...
    sprintf('/%s/takeoff', cfg.bebop_namespace), 'std_msgs/Empty');
msg_takeoff_bebop = rosmessage(pub_takeoff_bebop);

pub_land_bebop = rospublisher( ...
    sprintf('/%s/land', cfg.bebop_namespace), 'std_msgs/Empty');
msg_land_bebop = rosmessage(pub_land_bebop);

% Leitura da pose dos robôs via OptiTrack
sub_pose_limo = rossubscriber( ...
    sprintf('vrpn_client_node/%s/pose', cfg.limo_namespace));
sub_pose_bebop = rossubscriber( ...
    sprintf('vrpn_client_node/%s/pose', cfg.bebop_namespace));

% Joystick (PDF)
J = vrjoystick(1);

%% ===================== DECOLAGEM DO DRONE =====================
fprintf('Enviando takeoff para o Bebop 2...\n');
send(pub_takeoff_bebop, msg_takeoff_bebop);
pause(cfg.takeoff_pause);

%% ===================== ESTADO DO CONTROLADOR =====================
v_limo = [0.0; 0.0];              % [u; omega]
v_bebop = [0.0; 0.0; 0.0; 0.0];   % [vx; vy; vz; psi_dot] no corpo

hist_t = [];
hist_error = [];

t0 = tic;
running = true;

fprintf('Iniciando loop de controle a %.1f Hz...\n', 1 / cfg.T);

try
    while running && toc(t0) < cfg.t_final
        loop_start = tic;
        t = toc(t0);

        % --- Joystick: botão de parada de emergência (PDF) ---
        Digital = button(J);
        if Digital(cfg.joystick_stop_button)
            fprintf('Parada solicitada pelo joystick.\n');
            break;
        end

        % --- Leitura das poses via OptiTrack (PDF) ---
        [pos_limo, yaw_limo, ok_limo] = read_optitrack_pose(sub_pose_limo);
        [pos_bebop, yaw_bebop, ok_bebop] = read_optitrack_pose(sub_pose_bebop);

        if ~ok_limo || ~ok_bebop
            warning('Pose indisponível. Enviando velocidade zero.');
            send_zero_velocities(msg_cmdvel_limo, pub_cmdvel_limo, msg_cmdvel_bebop, pub_cmdvel_bebop);
            pause(cfg.T);
            continue;
        end

        x1 = pos_limo(1);
        y1 = pos_limo(2);
        psi1 = yaw_limo;

        x2 = pos_bebop(1);
        y2 = pos_bebop(2);
        z2 = pos_bebop(3);
        psi2 = yaw_bebop;

        % --- Pontos de interesse ---
        poi_limo = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1); 0.0];
        poi_bebop = [x2; y2; z2];

        delta = poi_bebop - poi_limo;
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

        % --- Jacobiano inverso ---
        S = [cos(alpha) * cos(beta), -rho * sin(alpha) * cos(beta), -rho * cos(alpha) * sin(beta); ...
             cos(alpha) * sin(beta), -rho * sin(alpha) * sin(beta),  rho * cos(alpha) * cos(beta); ...
             sin(alpha),              rho * cos(alpha),               0.0];

        J_inv = [eye(3), zeros(3); eye(3), S];
        x_r = J_inv * q_r;

        % --- Cinemática inversa local: LIMO ---
        A1_inv = [cos(psi1), sin(psi1); ...
                  -sin(psi1) / cfg.a1, cos(psi1) / cfg.a1];
        v_d_limo = clamp_vec(A1_inv * x_r(1:2), cfg.v_max_limo);

        % --- Cinemática inversa local: Bebop ---
        R_yaw = [cos(psi2), sin(psi2), 0.0; ...
                 -sin(psi2), cos(psi2), 0.0; ...
                  0.0, 0.0, 1.0];
        v_d_bebop_body = R_yaw * x_r(4:6);
        v_d_bebop_body(1:2) = clamp_vec(v_d_bebop_body(1:2), cfg.v_max_bebop_xy);
        v_d_bebop_body(3) = clamp_scalar( ...
            v_d_bebop_body(3) + 2.0 * (cfg.bebop_altitude - z2), cfg.v_max_bebop_z);
        v_d_bebop = [v_d_bebop_body; 0.0];

        % --- Laço interno: LIMO ---
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

        % --- Laço interno: Bebop (modelo simplificado v_dot = f1*u - f2*v) ---
        u_control_bebop = v_d_bebop + 1.0 * (v_d_bebop - v_bebop);
        v_bebop(1:3) = clamp_vec( ...
            v_bebop(1:3) + cfg.T * 15.0 * (u_control_bebop(1:3) - v_bebop(1:3)), cfg.v_max_state);
        v_bebop(3) = clamp_scalar(v_bebop(3), cfg.v_max_bebop_z);

        % --- Envio dos comandos via ROS (PDF) ---
        % LIMO: apenas Linear.X e Angular.Z (PDF)
        msg_cmdvel_limo.Linear.X = v_limo(1);
        msg_cmdvel_limo.Linear.Y = 0.0;
        msg_cmdvel_limo.Linear.Z = 0.0;
        msg_cmdvel_limo.Angular.X = 0.0;
        msg_cmdvel_limo.Angular.Y = 0.0;
        msg_cmdvel_limo.Angular.Z = v_limo(2);
        send(pub_cmdvel_limo, msg_cmdvel_limo);

        % Bebop: cmd_vel completo no referencial do corpo
        msg_cmdvel_bebop.Linear.X = v_bebop(1);
        msg_cmdvel_bebop.Linear.Y = v_bebop(2);
        msg_cmdvel_bebop.Linear.Z = v_bebop(3);
        msg_cmdvel_bebop.Angular.X = 0.0;
        msg_cmdvel_bebop.Angular.Y = 0.0;
        msg_cmdvel_bebop.Angular.Z = 0.0;
        send(pub_cmdvel_bebop, msg_cmdvel_bebop);

        % --- Log ---
        hist_t(end + 1, 1) = t; %#ok<AGROW>
        hist_error(:, end + 1) = error_q; %#ok<AGROW>

        if mod(numel(hist_t), 30) == 0
            fprintf('t=%6.2f s | erro rho=%+.3f m | erro beta=%+.1f deg\n', ...
                t, error_q(4), rad2deg(error_q(6)));
        end

        % --- Sincronização 30 Hz ---
        elapsed = toc(loop_start);
        pause(max(0.0, cfg.T - elapsed));
    end
catch ME
    fprintf('Erro durante o loop: %s\n', ME.message);
end

%% ===================== ENCERRAMENTO (PDF) =====================
fprintf('Encerrando: velocidade zero, pouso e rosshutdown.\n');
send_zero_velocities(msg_cmdvel_limo, pub_cmdvel_limo, msg_cmdvel_bebop, pub_cmdvel_bebop);
send(pub_land_bebop, msg_land_bebop);
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
    if isempty(latest) || isempty(latest.Pose)
        return;
    end

    pose_latest = latest.Pose;
    quat = [pose_latest.Orientation.W, pose_latest.Orientation.X, ...
            pose_latest.Orientation.Y, pose_latest.Orientation.Z];
    eul_zyx = quat2eul(quat, 'ZYX'); % [yaw pitch roll]
    yaw = eul_zyx(1);

    position = [pose_latest.Position.X; ...
                pose_latest.Position.Y; ...
                pose_latest.Position.Z];
    ok = true;
end

function send_zero_velocities(msg_limo, pub_limo, msg_bebop, pub_bebop)
    msg_limo.Linear.X = 0.0;
    msg_limo.Linear.Y = 0.0;
    msg_limo.Linear.Z = 0.0;
    msg_limo.Angular.X = 0.0;
    msg_limo.Angular.Y = 0.0;
    msg_limo.Angular.Z = 0.0;
    send(pub_limo, msg_limo);

    msg_bebop.Linear.X = 0.0;
    msg_bebop.Linear.Y = 0.0;
    msg_bebop.Linear.Z = 0.0;
    msg_bebop.Angular.X = 0.0;
    msg_bebop.Angular.Y = 0.0;
    msg_bebop.Angular.Z = 0.0;
    send(pub_bebop, msg_bebop);
end

function y = clamp_vec(x, limit)
    y = min(max(x, -limit), limit);
end

function y = clamp_scalar(x, limit)
    y = min(max(x, -limit), limit);
end
