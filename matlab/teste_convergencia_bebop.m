% Teste isolado de convergência do Bebop até o ponto de formação, com o
% LIMO real parado. Bebop é virtual (planta v_dot = f1*u - f2*v) — nenhum
% comando é enviado a nenhum robô real. Serve só para validar se o
% controlador converge para o lugar certo antes de qualquer voo real.
clear; clc; close all;

%% Configuração
cfg.T = 1 / 30;
cfg.Tsim = 60; % duração do teste (s)
cfg.a1 = 0.10;
rho_f = 1.5;
alpha_f = 0;
beta_f = pi / 3;

Kp_B = diag([1.0, 1.0, 1.2]);
Ls_B = diag([0.6, 0.6, 0.6]);
KD_B = diag([2.5, 2.5, 2.0, 2.5]);
f1 = diag([0.8417, 0.8354, 3.966, 9.8524]);
f2 = diag([0.18227, 0.17095, 4.001, 4.7295]);
vd_B_max = [0.5; 0.5; 0.3];
cmdB_max = [0.5; 0.5; 0.3; 0.5];

cfg.soft_start_time_s = 8.0;
cfg.soft_start_gamma_min = 0.3;

N = round(cfg.Tsim / cfg.T);

%% ROS e OptiTrack (LIMO real e parado; Bebop virtual)
rosshutdown;
rosinit('http://192.168.0.100:11311');
pose_L = rossubscriber('/natnet_ros/L1/pose', 'geometry_msgs/PoseStamped');
fprintf('Aguardando pose do LIMO...\n');
receive(pose_L, 10);

%% Estado inicial do Bebop virtual (pousado ao lado do LIMO)
[x1, y1, ~, psi1, ~] = ler_pose(pose_L);
poi_limo = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];

p2 = [poi_limo; 0];
psi2 = 0;
v_body_B = zeros(4, 1);

H.t = zeros(1, N);
H.p2 = zeros(3, N);
H.p2d = zeros(3, N);
H.erro = zeros(1, N);

%% Loop de convergência
fprintf('Iniciando teste de convergência (%d s). Ctrl+C para interromper.\n', cfg.Tsim);
t0 = tic;
try
    for k = 1:N
        tloop = tic;
        t = toc(t0);

        [x1, y1, ~, psi1, ~] = ler_pose(pose_L); % LIMO real, deve estar parado
        poi_limo = [x1 + cfg.a1 * cos(psi1); y1 + cfg.a1 * sin(psi1)];
        p2d = [poi_limo; 0] + cluster_offset(rho_f, alpha_f, beta_f);

        gamma = min(max(t / cfg.soft_start_time_s, cfg.soft_start_gamma_min), 1);
        Kp_B_eff = gamma * Kp_B;

        vd_B_world = Ls_B * tanh(Ls_B \ (Kp_B_eff * (p2d - p2)));
        vd_B_world = [saturar(vd_B_world(1), vd_B_max(1));
                      saturar(vd_B_world(2), vd_B_max(2));
                      saturar(vd_B_world(3), vd_B_max(3))];

        A2inv = [cos(psi2), sin(psi2), 0; -sin(psi2), cos(psi2), 0; 0, 0, 1];
        vB_meas = v_body_B; % planta virtual: estado interno real, sem ruído de pose
        vd_B = [A2inv * vd_B_world; 0];
        KD_B_eff = gamma * KD_B;

        cmdB_raw = f1 \ (KD_B_eff * (vd_B - vB_meas) + f2 * vB_meas);
        cmdB = max(min(cmdB_raw, cmdB_max), -cmdB_max);

        % Avanço da planta virtual do Bebop
        R = [cos(psi2), -sin(psi2), 0; sin(psi2), cos(psi2), 0; 0, 0, 1];
        p2 = p2 + cfg.T * R * v_body_B(1:3);
        psi2 = wrap_pi(psi2 + cfg.T * v_body_B(4));
        v_body_B = v_body_B + cfg.T * (f1 * cmdB - f2 * v_body_B);

        H.t(k) = t;
        H.p2(:, k) = p2;
        H.p2d(:, k) = p2d;
        H.erro(k) = norm(p2d - p2);

        if mod(k, 30) == 0
            fprintf('t=%5.1fs | erro=%.3f m | p2=(%+.2f,%+.2f,%+.2f) alvo=(%+.2f,%+.2f,%+.2f)\n', ...
                t, H.erro(k), p2(1), p2(2), p2(3), p2d(1), p2d(2), p2d(3));
        end
        pause(max(0, cfg.T - toc(tloop)));
    end
catch ME
    fprintf(2, 'ERRO no loop: %s\n', ME.message);
end

%% Encerramento
rosshutdown;
fprintf('Erro final: %.4f m | Erro RMS: %.4f m | Erro máximo: %.4f m\n', ...
    H.erro(end), sqrt(mean(H.erro.^2)), max(H.erro));

%% Gráfico
figure('Name', 'Teste de convergência do Bebop', 'Color', 'w');
subplot(1, 2, 1);
plot(H.t, H.erro, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('||p2d - p2|| [m]'); title('Erro de convergência');

subplot(1, 2, 2);
plot(H.t, H.p2(3, :), 'r', H.t, H.p2d(3, :), 'k--', 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('z [m]'); legend('real', 'alvo'); title('Altura do Bebop');

%% Funções locais

function offset = cluster_offset(rho, alpha, beta)
    offset = rho * [cos(alpha) * cos(beta); sin(alpha) * cos(beta); sin(beta)];
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

function y = saturar(u, umax)
    y = max(min(u, umax), -umax);
end

function ang = wrap_pi(ang)
    ang = atan2(sin(ang), cos(ang));
end
