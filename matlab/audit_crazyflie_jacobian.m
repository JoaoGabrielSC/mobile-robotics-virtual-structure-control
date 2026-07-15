%% audit_crazyflie_jacobian.m
% Auditoria cinemática do Crazyflie.
% Lê uma única pose pelo VRPN, calcula a transformação entre os referenciais global e do corpo
% e imprime todos os valores. Não envia comandos ao drone.

clear;
clc;

%% CONFIGURAÇÃO
ros_master = '192.168.0.100';
namespace = 'cf7';             % Ajuste para o nome do Crazyflie no Motive
reference_position = [0; 0; 0]; % Apenas auditoria: não é uma altitude de voo
kp_position = 0.5;             % Ganho usado somente no cálculo auditado
pose_timeout = 10;             % s

pose_topic = sprintf('vrpn_client_node/%s/pose', namespace);

fprintf('=== AUDITORIA CINEMÁTICA: CRAZYFLIE ===\n');
fprintf('Namespace: %s\n', namespace);
fprintf('Tópico de pose: %s\n', pose_topic);
fprintf('Referência de auditoria [m]:\n');
disp(reference_position);

%% ROS E LEITURA DA POSE
rosshutdown;
cleanup_ros = onCleanup(@() rosshutdown);
rosinit(ros_master);

pose_subscriber = rossubscriber(pose_topic, 'geometry_msgs/PoseStamped');
fprintf('\nAguardando uma pose válida (timeout: %.1f s)...\n', pose_timeout);

try
    pose_message = receive(pose_subscriber, pose_timeout);
catch ME
    error('Não foi possível ler pose em %s: %s', pose_topic, ME.message);
end

%% ESTADO MEDIDO
pose = pose_message.Pose;
quaternion = [pose.Orientation.W; ...
              pose.Orientation.X; ...
              pose.Orientation.Y; ...
              pose.Orientation.Z];
euler_zyx = quat2eul(quaternion.');
angles_xyz = [euler_zyx(3); euler_zyx(2); euler_zyx(1)];
yaw = angles_xyz(3);
position = [pose.Position.X; pose.Position.Y; pose.Position.Z];

fprintf('\n--- ESTADO MEDIDO ---\n');
fprintf('Quaternion [w; x; y; z]:\n');
disp(quaternion);
fprintf('Euler XYZ [roll; pitch; yaw] (rad):\n');
disp(angles_xyz);
fprintf('Yaw: %.6f rad (%.3f graus)\n', yaw, rad2deg(yaw));
fprintf('Posição global p [m]:\n');
disp(position);

%% JACOBIANO E TRANSFORMAÇÃO
% p_dot_global = J(yaw) * v_body
J = [cos(yaw), -sin(yaw), 0; ...
     sin(yaw),  cos(yaw), 0; ...
     0,         0,        1];
J_inv = J.';

position_error = reference_position - position;
velocity_global_ref = kp_position * position_error;
velocity_body_ref = J_inv * velocity_global_ref;
velocity_global_reconstructed = J * velocity_body_ref;
residual = velocity_global_reconstructed - velocity_global_ref;

fprintf('\n--- MATRIZES ---\n');
fprintf('J(yaw): p_dot_global = J * v_body\n');
disp(J);
fprintf('J_inv(yaw): v_body = J_inv * p_dot_global\n');
disp(J_inv);

fprintf('\n--- RESULTADOS DA TRANSFORMAÇÃO ---\n');
fprintf('Erro e = p_ref - p [m]:\n');
disp(position_error);
fprintf('v_global_ref = Kp * e [m/s]:\n');
disp(velocity_global_ref);
fprintf('v_body_ref = J_inv * v_global_ref [m/s]:\n');
disp(velocity_body_ref);
fprintf('J * v_body_ref [m/s]:\n');
disp(velocity_global_reconstructed);
fprintf('Resíduo J*v_body_ref - v_global_ref [m/s]:\n');
disp(residual);
fprintf('Norma do resíduo: %.3e\n', norm(residual));

fprintf('\nAuditoria concluída. Nenhum comando foi enviado ao Crazyflie.\n');
clear cleanup_ros;
