%% main.m
% Controle de formação heterogênea LIMO + Crazyflie.

clear;
clc;
close all;

%% CONFIGURAÇÃO
cfg.T = 1 / 30;
cfg.t_final = 80;
cfg.ros_master = 'http://192.168.0.100:11311';
cfg.limo_namespace = 'L1';
cfg.cf_namespace = 'cf7';
cfg.pose_prefix = '/natnet_ros';
cfg.pose_timeout = 15;

cfg.a_limo = 0.10;
cfg.rho_f = 1.5;
cfg.alpha_f = 0;
cfg.beta_f = pi / 2;
cfg.Kq = diag([1.2, 1.2, 1.0, 1.5, 1.0, 1.5]);
cfg.Lq = diag([0.4, 0.4, 0.3, 0.5, 0.4, 0.5]);
cfg.K_drone = diag([0.8, 0.8, 0.8]);
cfg.L_drone = diag([0.4, 0.4, 0.3]);
cfg.beta_singularity_threshold = deg2rad(5);

cfg.obstacle_center = [-0.2; 0.425];
cfg.obstacle_radius = 0.15;
cfg.obstacle_influence_radius = 0.5;
cfg.obstacle_gain = 0.35;

cfg.theta_limo = [0.1521; 0.0953; 0.0031; 0.9840; -0.0451; 1.6422];
cfg.KD_limo = diag([4.0, 4.0]);
cfg.f1_cf = diag([0.8417, 0.8354, 3.9660, 9.8524]);
cfg.f2_cf = diag([0.18227, 0.17095, 4.0010, 4.7295]);
cfg.KD_cf = diag([2.0, 2.0, 1.8, 5.0]);

cfg.v_limo_max = 0.30;
cfg.w_limo_max = 1.20;
cfg.phi_max = deg2rad(5);
cfg.theta_max = deg2rad(5);
cfg.zdot_max = 1.0;
cfg.psidot_max = 100.0;
cfg.roll_sign = 1;
cfg.pitch_sign = 1;

cfg.dry_run = true;
cfg.use_joystick = true;
cfg.stop_button = 1;
cfg.kill_button = 2;
cfg.takeoff_settle_time = 3;

%% ROS
rosshutdown;
cleanup_ros = onCleanup(@() rosshutdown);
rosinit(cfg.ros_master);

pub_limo = rospublisher(sprintf('/%s/cmd_vel', cfg.limo_namespace), 'geometry_msgs/Twist');
pub_cf = rospublisher(sprintf('/%s/cmd_vel', cfg.cf_namespace), 'geometry_msgs/Twist');

sub_limo = rossubscriber(sprintf('%s/%s/pose', cfg.pose_prefix, cfg.limo_namespace), ...
    'geometry_msgs/PoseStamped');
sub_cf = rossubscriber(sprintf('%s/%s/pose', cfg.pose_prefix, cfg.cf_namespace), ...
    'geometry_msgs/PoseStamped');

takeoff_client = rossvcclient(sprintf('/%s/takeoff', cfg.cf_namespace), 'std_srvs/Trigger');
land_client = rossvcclient(sprintf('/%s/land', cfg.cf_namespace), 'std_srvs/Trigger');
kill_client = rossvcclient(sprintf('/%s/kill', cfg.cf_namespace), 'std_srvs/Trigger');

wait_for_pose(sub_limo, cfg.pose_timeout, cfg.limo_namespace);
wait_for_pose(sub_cf, cfg.pose_timeout, cfg.cf_namespace);

if cfg.use_joystick
    joystick = JoyControl;
else
    joystick = [];
end

if ~cfg.dry_run
    response = call(takeoff_client, rosmessage(takeoff_client), 'Timeout', 5);
    if ~response.Success
        error('Decolagem do Crazyflie falhou: %s', response.Message);
    end
    pause(cfg.takeoff_settle_time);
end

%% MEMÓRIAS E HISTÓRICO
[p_limo, yaw_limo] = read_pose(sub_limo);
[p_cf, yaw_cf] = read_pose(sub_cf);

memory.prev_poi_limo = p_limo(1:2) + cfg.a_limo * [cos(yaw_limo); sin(yaw_limo)];
memory.prev_cf_position = p_cf;
memory.prev_cf_yaw = yaw_cf;
memory.prev_vd_limo = [0; 0];
memory.prev_vd_cf = [0; 0; 0; 0];

N = ceil(cfg.t_final / cfg.T);
H.t = zeros(1, N);
H.q = zeros(6, N);
H.qd = zeros(6, N);
H.p_limo = zeros(3, N);
H.p_cf = zeros(3, N);
H.error_q = zeros(6, N);
H.cmd_limo = zeros(2, N);
H.cmd_cf = zeros(4, N);
H.obstacle_distance = zeros(1, N);
H.obstacle_active = false(1, N);
H.drone_cartesian_mode = false(1, N);

%% LAÇO DE CONTROLE
running = true;
emergency_kill = false;
k = 0;
t0 = tic;

try
    while running && k < N && toc(t0) < cfg.t_final
        loop_start = tic;
        t = toc(t0);
        k = k + 1;

        if cfg.use_joystick
            mRead(joystick);
            buttons = joystick.pDigital;
            if is_button_pressed(buttons, cfg.kill_button)
                emergency_kill = true;
                break;
            end
            if is_button_pressed(buttons, cfg.stop_button)
                break;
            end
        end

        [p_limo, yaw_limo] = read_pose(sub_limo);
        [p_cf, yaw_cf] = read_pose(sub_cf);
        [q, poi_limo] = get_formation_state(p_limo, yaw_limo, p_cf, cfg.a_limo);
        [qd, qd_dot] = formation_reference(t, cfg);
        q_dot_formation = formation_controller(q, qd, qd_dot, cfg.Kq, cfg.Lq);
        [q_dot_ref, obstacle_active, obstacle_distance] = null_space_avoidance( ...
            q_dot_formation, poi_limo(1:2), cfg);

        x_dot_ref = calc_jacobian_inv(q) * q_dot_ref;
        A_limo_inv = [cos(yaw_limo), sin(yaw_limo); ...
            -sin(yaw_limo) / cfg.a_limo, cos(yaw_limo) / cfg.a_limo];
        v_limo_des = A_limo_inv * x_dot_ref(1:2);

        use_cartesian_drone = abs(cos(q(6))) < cfg.beta_singularity_threshold;
        if use_cartesian_drone
            p_cf_des = poi_limo + cluster_offset(qd(4:6));
            v_cf_world = x_dot_ref(1:3) + cfg.L_drone * tanh( ...
                cfg.L_drone \ (cfg.K_drone * (p_cf_des - p_cf)));
        else
            v_cf_world = x_dot_ref(4:6);
        end
        R_world_to_body = [cos(yaw_cf), sin(yaw_cf), 0; ...
            -sin(yaw_cf), cos(yaw_cf), 0; ...
            0, 0, 1];
        v_cf_des = [R_world_to_body * v_cf_world; 0];

        v_limo_world = (poi_limo(1:2) - memory.prev_poi_limo) / cfg.T;
        v_limo_meas = A_limo_inv * v_limo_world;
        v_cf_world_meas = (p_cf - memory.prev_cf_position) / cfg.T;
        v_cf_meas = [R_world_to_body * v_cf_world_meas; ...
            wrap_to_pi(yaw_cf - memory.prev_cf_yaw) / cfg.T];

        cmd_limo = dynamic_compensator('limo', v_limo_des, v_limo_meas, ...
            memory.prev_vd_limo, cfg);
        cmd_cf = dynamic_compensator('crazyflie', v_cf_des, v_cf_meas, ...
            memory.prev_vd_cf, cfg);

        if ~cfg.dry_run
            send_cmd(pub_limo, cmd_limo, 'limo');
            send_cmd(pub_cf, cmd_cf, 'crazyflie');
        end

        H.t(k) = t;
        H.q(:, k) = q;
        H.qd(:, k) = qd;
        H.p_limo(:, k) = poi_limo;
        H.p_cf(:, k) = p_cf;
        H.error_q(:, k) = qd - q;
        H.error_q(5:6, k) = wrap_to_pi(H.error_q(5:6, k));
        H.cmd_limo(:, k) = cmd_limo;
        H.cmd_cf(:, k) = cmd_cf;
        H.obstacle_distance(k) = obstacle_distance;
        H.obstacle_active(k) = obstacle_active;
        H.drone_cartesian_mode(k) = use_cartesian_drone;

        memory.prev_poi_limo = poi_limo(1:2);
        memory.prev_cf_position = p_cf;
        memory.prev_cf_yaw = yaw_cf;
        memory.prev_vd_limo = v_limo_des;
        memory.prev_vd_cf = v_cf_des;

        if mod(k, 30) == 0
            fprintf('t=%5.1f s | rho=%.2f m | obstaculo=%.2f m | modo drone=%s\n', ...
                t, q(4), obstacle_distance, ternary(use_cartesian_drone, 'cartesiano', 'jacobiano'));
        end
        pause(max(0, cfg.T - toc(loop_start)));
    end
catch ME
    fprintf(2, 'Erro no controle: %s\n', ME.message);
end

%% ENCERRAMENTO
if ~cfg.dry_run
    send_cmd(pub_limo, [0; 0], 'limo');
    send_cmd(pub_cf, [0; 0; 0; 0], 'crazyflie');
    if emergency_kill
        call(kill_client, rosmessage(kill_client), 'Timeout', 5);
    else
        call(land_client, rosmessage(land_client), 'Timeout', 5);
    end
end

H = trim_history(H, k);
plot_results(H, cfg);
clear cleanup_ros;

%% FUNÇÕES LOCAIS
function [qd, qd_dot] = formation_reference(t, cfg)
    wx = 2 * pi / 40;
    wy = 4 * pi / 40;
    qd = [0.75 * sin(wx * t); 0.75 * sin(wy * t); 0; ...
        cfg.rho_f; cfg.alpha_f; cfg.beta_f];
    qd_dot = [0.75 * wx * cos(wx * t); 0.75 * wy * cos(wy * t); ...
        0; 0; 0; 0];
end

function offset = cluster_offset(cluster_shape)
    rho = cluster_shape(1);
    alpha = cluster_shape(2);
    beta = cluster_shape(3);
    offset = rho * [cos(alpha) * cos(beta); ...
        sin(alpha) * cos(beta); sin(beta)];
end

function [position, yaw] = read_pose(subscriber)
    message = subscriber.LatestMessage;
    if isempty(message)
        error('Pose indisponível.');
    end
    pose = message.Pose;
    quaternion = [pose.Orientation.W, pose.Orientation.X, ...
        pose.Orientation.Y, pose.Orientation.Z];
    euler_zyx = quat2eul(quaternion);
    position = [pose.Position.X; pose.Position.Y; pose.Position.Z];
    yaw = euler_zyx(1);
end

function wait_for_pose(subscriber, timeout, robot_name)
    try
        receive(subscriber, timeout);
    catch ME
        error('Pose de %s não recebida: %s', robot_name, ME.message);
    end
end

function send_cmd(publisher, u, robot_type)
    message = rosmessage(publisher);
    message.Linear.X = 0;
    message.Linear.Y = 0;
    message.Linear.Z = 0;
    message.Angular.X = 0;
    message.Angular.Y = 0;
    message.Angular.Z = 0;
    if strcmp(robot_type, 'limo')
        message.Linear.X = u(1);
        message.Angular.Z = u(2);
    elseif strcmp(robot_type, 'crazyflie')
        message.Angular.X = u(1);
        message.Angular.Y = u(2);
        message.Linear.Z = u(3);
        message.Angular.Z = u(4);
    else
        error('Tipo de robô inválido: %s', robot_type);
    end
    send(publisher, message);
end

function pressed = is_button_pressed(buttons, index)
    pressed = index >= 1 && index <= numel(buttons) && logical(buttons(index));
end

function value = wrap_to_pi(value)
    value = atan2(sin(value), cos(value));
end

function value = ternary(condition, true_value, false_value)
    if condition
        value = true_value;
    else
        value = false_value;
    end
end

function H = trim_history(H, k)
    fields = fieldnames(H);
    for i = 1:numel(fields)
        value = H.(fields{i});
        H.(fields{i}) = value(:, 1:k);
    end
end
