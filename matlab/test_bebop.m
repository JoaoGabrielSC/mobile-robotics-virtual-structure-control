%% test_bebop.m - Teste isolado do Bebop 2 (conexão ROS + OptiTrack + cmd_vel)
% Robótica Móvel 2026/1 - LAB-AIR
%
% Use este script ANTES do main.m para validar apenas o quadrirrotor Bebop 2.
%
% ===== Pré-requisitos =====
%  1) Motive: corpo rígido nomeado B1 (ou cfg.bebop_namespace)
%  2) roslaunch natnet_ros_cpp natnet_ros.launch  (no rosserver .100)
%  3) Bebop 2 ligado e conectado via Wi-Fi
%  4) roslaunch bebop_driver bebop_node.launch
%  5) JoyControl.m no path do MATLAB
%
% ===== Rede =====
%  rosinit -> 192.168.0.100:11311 (master)
%  Bebop 2 -> 192.168.42.1 (rede Wi-Fi própria do drone)
%
%  IMPORTANTE: O Bebop 2 cria sua própria rede Wi-Fi ao ligar.
%  Para usar no lab com OptiTrack + rosmaster, você precisa de DUAS
%  interfaces de rede no PC que roda o bebop_driver:
%    - eth0 (cabo): conectado à rede do lab (192.168.0.x)
%    - wlan0 (Wi-Fi): conectado à rede do Bebop (192.168.42.x)
%
%  Configuração típica:
%    1) Conecte o cabo de rede ao PC (rede do lab)
%    2) Ligue o Bebop 2 e aguarde a rede Wi-Fi aparecer (Bebop2-XXXXXX)
%    3) Conecte o Wi-Fi do PC à rede do Bebop (senha padrão: nenhuma)
%    4) Verifique rotas: o bebop_driver usa 192.168.42.1, ROS usa 192.168.0.x
%    5) Se necessário, adicione rota estática:
%         sudo ip route add 192.168.42.0/24 dev wlan0
%
% ===== Modos =====
%  'monitor'      — só lê pose, não decola (mais seguro; primeiro teste)
%  'teleop'       — decola e joystick comanda vx/vy/vz/vyaw
%  'takeoff_land' — decola, paira alguns segundos (firmware), pousa
%  'hover'        — decola e mantém uma posição 3D fixa (laço completo)
%  'lemniscate'   — decola e segue a lemniscata do enunciado a z=1,5 m
%
% ===== Interface cmd_vel do Bebop 2 =====
%  O driver bebop_autonomy interpreta cmd_vel como:
%   - Linear.X  : velocidade linear para frente/trás (m/s)
%   - Linear.Y  : velocidade lateral esquerda/direita (m/s)
%   - Linear.Z  : velocidade vertical subir/descer (m/s)
%   - Angular.Z : velocidade de guinada (yaw rate, rad/s)
%  Os campos Angular.X e Angular.Y são ignorados (não há controle direto
%  de phi/theta como no Crazyflie; o firmware do Bebop converte
%  velocidades em atitudes internamente).
%
% ===== Modelo dinâmico near-hover (Sarcinelli-Filho & Carelli, 2023) =====
%  Cinemática near-hover:      Cap. 2.2, eq. (2.25)-(2.26)
%  Dinâmica near-hover:        Cap. 3.3, eq. (3.29)-(3.31)
%  Laço externo (tanh):        Cap. 4.5, eq. (4.10), (4.12a-d)
%  Compensação dinâmica cascata: Cap. 4.11, eq. (4.47)
%
% ===== Encerramento =====
%  Botão stop (cfg.joystick_stop_button) -> land + sair
%  Botão kill (cfg.joystick_kill_button) -> reset (emergência) + sair

clear;
clc;
close all;

%% ========================================================================
%  CONFIGURAÇÃO — edite aqui antes de rodar
%  ========================================================================

% --- Modo de operação ---------------------------------------------------
% 'monitor' | 'teleop' | 'takeoff_land' | 'hover' | 'lemniscate'
cfg.mode = 'monitor';
cfg.t_final = 80;                % duração do modo lemniscate (s); 80 = 2×40 s
cfg.T = 1 / 30;                  % período de amostragem (30 Hz, enunciado)
cfg.pose_timeout = 30;           % timeout aguardando primeira pose (s)

% --- Rede ROS -----------------------------------------------------------
cfg.ros_master_host = '192.168.0.100';   % rosserver (roscore)
cfg.ros_master_port = 11311;
cfg.bebop_namespace = 'bebop';           % namespace do bebop_driver
cfg.pose_topic_prefix = '/natnet_ros';
cfg.bebop_rigid_body = 'B1';             % nome do corpo rígido no Motive

% --- Bebop 2: limites físicos -------------------------------------------
% Especificações do Bebop 2 (datasheet + conservador para lab)
cfg.vxy_max = 2.0;               % velocidade horizontal máxima (m/s)
cfg.vz_max = 1.0;                % velocidade vertical máxima (m/s)
cfg.vyaw_max = deg2rad(100);     % velocidade de yaw máxima (rad/s)

% Limites conservadores para testes iniciais
cfg.vxy_max_safe = 0.5;          % limite conservador em testes (m/s)
cfg.vz_max_safe = 0.3;           % limite conservador em testes (m/s)
cfg.vyaw_max_safe = deg2rad(30); % limite conservador em testes (rad/s)

% --- Controle: laço externo (cinemático, tanh) --------------------------
% v = ẋd + Kp*tanh((Kp/Ks)*x̃), eq. (4.10)/(4.12a-d)
cfg.kp_xy = 0.6;                 % ganho proporcional em x,y
cfg.ks_xy = 0.25;                % saturação (m/s) em x,y
cfg.kp_z = 0.6;                  % ganho proporcional em z
cfg.ks_z = 0.25;                 % saturação (m/s) em z
cfg.kp_psi = 0.8;                % ganho proporcional em psi
cfg.ks_psi = 0.5;                % saturação (rad/s) em psi

% --- Controle: laço interno (compensador dinâmico near-hover) -----------
% v̇ = f1*u - f2*v (eq. 3.30); modelo identificado Bebop 2
% Parâmetros do enunciado (Bebop 2):
cfg.f1_bebop = diag([0.8417, 0.8354, 3.9660, 9.8524]);  % [vx;vy;vz;vpsi]
cfg.f2_bebop = diag([0.18227, 0.17095, 4.0010, 4.7295]);
cfg.kd_bebop = diag([2.0, 2.0, 1.8, 5.0]);              % KD, eq. (4.47)

% --- Modo 'hover' -------------------------------------------------------
cfg.hover_target = [0.0; 0.0; 1.5];   % [x;y;z] desejado (m)
cfg.hover_yaw_target = 0.0;           % yaw desejado (rad)

% --- Modo 'lemniscate' --------------------------------------------------
cfg.z_hover = 1.5;               % altura constante da formação (enunciado)
cfg.psi_ref = 0.0;               % yaw de referência (olhar para +X)

% --- Sequenciamento de voo ----------------------------------------------
cfg.takeoff_settle_time = 5.0;   % s de espera após o takeoff
cfg.land_settle_time = 3.0;      % s de espera após enviar land

% --- Joystick (JoyControl) ----------------------------------------------
cfg.joystick_axis_vx = 2;        % eixo para velocidade frontal
cfg.joystick_axis_vy = 1;        % eixo para velocidade lateral
cfg.joystick_axis_vz = 4;        % eixo para velocidade vertical
cfg.joystick_axis_vyaw = 3;      % eixo para velocidade de yaw
cfg.joystick_deadzone = 0.15;
cfg.joystick_stop_button = 1;    % botão -> pouso controlado (land)
cfg.joystick_kill_button = 2;    % botão -> emergência (reset)

% --- Resultados (modos 'hover' e 'lemniscate') --------------------------
cfg.save_results = true;
cfg.results_dir = fullfile('results', 'test_bebop');
cfg.save_gif = true;
cfg.gif_fps = 10;
cfg.gif_frame_step = 3;

%% ========================================================================
%  Fim da configuração — não é necessário editar abaixo desta linha
%  ========================================================================

flight_modes = {'teleop', 'takeoff_land', 'hover', 'lemniscate'};
requires_flight = any(strcmp(cfg.mode, flight_modes));

fprintf('=== Teste Bebop 2 | modo: %s ===\n', cfg.mode);
fprintf('Botão %d: pouso controlado. Botão %d: EMERGÊNCIA (reset).\n', ...
    cfg.joystick_stop_button, cfg.joystick_kill_button);
if requires_flight
    fprintf(['[AVISO] Este modo faz o drone decolar. Confirme área livre ', ...
        'e bateria suficiente.\n']);
end

%% ROS
rosshutdown;
master_uri = sprintf('http://%s:%d', cfg.ros_master_host, cfg.ros_master_port);
fprintf('Conectando ao ROS master: %s\n', master_uri);
rosinit(master_uri);

pub_cmdvel = rospublisher( ...
    sprintf('/%s/cmd_vel', cfg.bebop_namespace), 'geometry_msgs/Twist');
msg_cmdvel = rosmessage(pub_cmdvel);

takeoff_client = rossvcclient( ...
    sprintf('/%s/takeoff', cfg.bebop_namespace), 'std_srvs/Empty');
takeoff_request = rosmessage(takeoff_client);

land_client = rossvcclient( ...
    sprintf('/%s/land', cfg.bebop_namespace), 'std_srvs/Empty');
land_request = rosmessage(land_client);

reset_client = rossvcclient( ...
    sprintf('/%s/reset', cfg.bebop_namespace), 'std_srvs/Empty');
reset_request = rosmessage(reset_client);

sub_pose = rossubscriber( ...
    sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.bebop_rigid_body), ...
    'geometry_msgs/PoseStamped');

%% Aguardar primeira pose (antes do joystick)
pose_topic = sprintf('%s/%s/pose', cfg.pose_topic_prefix, cfg.bebop_rigid_body);
fprintf('Aguardando pose do OptiTrack em %s (timeout %d s)...\n', ...
    pose_topic, cfg.pose_timeout);

[pos, yaw, pose_ok, pose_info] = wait_for_bebop_pose(sub_pose, pose_topic, cfg.pose_timeout);

if pose_ok
    fprintf('Pose OK: x=%.3f y=%.3f z=%.3f yaw=%.1f deg\n', ...
        pos(1), pos(2), pos(3), rad2deg(yaw));
else
    send_zero_cmd(msg_cmdvel, pub_cmdvel);
    rosshutdown;
    error(['Não foi possível ler pose do Bebop em %s.\n\n', ...
        'Diagnóstico: %s\n\n', ...
        'Verifique:\n', ...
        '  1) natnet_ros rodando no rosserver (.100)\n', ...
        '  2) %s visível no Motive (marcadores ativos)\n', ...
        '  3) No rosserver: rostopic echo %s\n', ...
        '  4) No MATLAB (após rosinit): rostopic(''list'') deve listar %s\n'], ...
        pose_topic, pose_info, cfg.bebop_rigid_body, pose_topic, pose_topic);
end

J = JoyControl;
fprintf('Joystick conectado.\n');

%% Decolagem (se necessário)
if requires_flight
    fprintf('Modo %s: preparando decolagem.\n', cfg.mode);
    fprintf('Área livre? Bateria OK?\n');
    input('Pressione Enter para decolar (Ctrl+C para cancelar)...', 's');
    
    fprintf('Enviando takeoff...\n');
    try
        call(takeoff_client, takeoff_request, 'Timeout', 10);
        fprintf('Takeoff enviado. Aguardando %.1f s para estabilização...\n', ...
            cfg.takeoff_settle_time);
        pause(cfg.takeoff_settle_time);
    catch ME
        warning('Falha no takeoff: %s', ME.message);
    end
end

%% Loop principal
t0 = tic;
running = true;
emergency_reset = false;
log_counter = 0;

v_state = [0.0; 0.0; 0.0; 0.0];      % estado interno do compensador [vx;vy;vz;vyaw]
hist_t = [];
hist_pos = [];
hist_yaw = [];
hist_ref = [];
hist_error = [];

try
    while running
        loop_start = tic;
        t = toc(t0);
        
        mRead(J);
        Analog = J.pAnalog;
        Digital = J.pDigital;
        
        if is_button_pressed(Digital, cfg.joystick_kill_button)
            fprintf('Emergência (reset) solicitada.\n');
            emergency_reset = true;
            break;
        end
        if is_button_pressed(Digital, cfg.joystick_stop_button)
            fprintf('Parada solicitada (land).\n');
            break;
        end
        
        [pos, yaw, pose_ok] = read_optitrack_pose(sub_pose);
        
        if ~pose_ok
            warning('Pose indisponível — enviando zero.');
            send_zero_cmd(msg_cmdvel, pub_cmdvel);
            pause(cfg.T);
            continue;
        end
        
        x = pos(1);
        y = pos(2);
        z = pos(3);
        psi = yaw;
        
        switch cfg.mode
            case 'monitor'
                v_cmd = [0.0; 0.0; 0.0; 0.0];
                
            case 'teleop'
                vx = cfg.vxy_max_safe * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_vx), cfg.joystick_deadzone);
                vy = cfg.vxy_max_safe * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_vy), cfg.joystick_deadzone);
                vz = cfg.vz_max_safe * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_vz), cfg.joystick_deadzone);
                vyaw = cfg.vyaw_max_safe * apply_deadzone( ...
                    read_axis(Analog, cfg.joystick_axis_vyaw), cfg.joystick_deadzone);
                v_cmd = [vx; vy; vz; vyaw];
                
            case 'takeoff_land'
                v_cmd = [0.0; 0.0; 0.0; 0.0];
                if t >= 10.0
                    fprintf('Tempo de hover atingido (%.1f s). Iniciando pouso.\n', t);
                    running = false;
                end
                
            case 'hover'
                ref = [cfg.hover_target; cfg.hover_yaw_target];
                [v_d, err] = outer_loop_hover(pos, psi, ref, cfg);
                v_state = inner_loop_bebop(v_d, v_state, cfg);
                v_cmd = velocity_to_body_frame(v_state, psi);
                
                hist_t(end + 1, 1) = t; %#ok<AGROW>
                hist_pos(:, end + 1) = pos; %#ok<AGROW>
                hist_yaw(end + 1, 1) = psi; %#ok<AGROW>
                hist_ref(:, end + 1) = ref; %#ok<AGROW>
                hist_error(:, end + 1) = err; %#ok<AGROW>
                
                if t >= cfg.t_final
                    fprintf('Tempo final (%.0f s) atingido.\n', cfg.t_final);
                    running = false;
                end
                
            case 'lemniscate'
                [ref, ref_dot] = lemniscata_reference_3d(t, cfg);
                [v_d, err] = outer_loop_lemniscate(pos, psi, ref, ref_dot, cfg);
                v_state = inner_loop_bebop(v_d, v_state, cfg);
                v_cmd = velocity_to_body_frame(v_state, psi);
                
                hist_t(end + 1, 1) = t; %#ok<AGROW>
                hist_pos(:, end + 1) = pos; %#ok<AGROW>
                hist_yaw(end + 1, 1) = psi; %#ok<AGROW>
                hist_ref(:, end + 1) = ref; %#ok<AGROW>
                hist_error(:, end + 1) = err; %#ok<AGROW>
                
                if t >= cfg.t_final
                    fprintf('Tempo final (%.0f s) atingido.\n', cfg.t_final);
                    running = false;
                end
                
            otherwise
                error('Modo desconhecido: %s', cfg.mode);
        end
        
        v_cmd = clamp_velocities(v_cmd, cfg);
        send_bebop_cmd(msg_cmdvel, pub_cmdvel, v_cmd);
        
        log_counter = log_counter + 1;
        if mod(log_counter, 30) == 0
            switch cfg.mode
                case 'monitor'
                    fprintf('t=%5.1fs | x=%+.3f y=%+.3f z=%+.3f yaw=%+6.1f°\n', ...
                        t, x, y, z, rad2deg(psi));
                case 'teleop'
                    fprintf('t=%5.1fs | x=%+.3f y=%+.3f z=%+.3f | cmd=[%.2f %.2f %.2f %.2f]\n', ...
                        t, x, y, z, v_cmd(1), v_cmd(2), v_cmd(3), v_cmd(4));
                case 'takeoff_land'
                    fprintf('t=%5.1fs | x=%+.3f y=%+.3f z=%+.3f | pairando\n', ...
                        t, x, y, z);
                case {'hover', 'lemniscate'}
                    err_norm = norm(hist_error(1:3, end));
                    fprintf('t=%5.1fs | pos=(%.3f,%.3f,%.3f) ref=(%.3f,%.3f,%.3f) err=%.3f m\n', ...
                        t, x, y, z, hist_ref(1, end), hist_ref(2, end), hist_ref(3, end), err_norm);
            end
        end
        
        elapsed = toc(loop_start);
        pause(max(0.0, cfg.T - elapsed));
    end
catch ME
    fprintf('Erro no loop: %s\n', ME.message);
end

%% Encerramento
fprintf('Encerrando: velocidade zero.\n');
send_zero_cmd(msg_cmdvel, pub_cmdvel);
pause(0.5);

if emergency_reset
    fprintf('Enviando reset (emergência).\n');
    try
        call(reset_client, reset_request, 'Timeout', 5);
    catch ME
        warning('Reset falhou: %s', ME.message);
    end
elseif requires_flight
    fprintf('Enviando land.\n');
    try
        call(land_client, land_request, 'Timeout', 10);
        fprintf('Aguardando %.1f s para pouso...\n', cfg.land_settle_time);
        pause(cfg.land_settle_time);
    catch ME
        warning('Land falhou: %s', ME.message);
    end
end

pause(1.0);
rosshutdown;

if cfg.save_results && ~isempty(hist_t)
    save_bebop_results(hist_t, hist_pos, hist_yaw, hist_ref, hist_error, cfg);
end

fprintf('Teste Bebop 2 finalizado.\n');

%% ========================================================================
%  Funções locais
%  ========================================================================

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

function [position, yaw, ok, info] = wait_for_bebop_pose(sub_pose, pose_topic, timeout_sec)
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
        info = [info, ' receive() retornou mensagem inválida.'];
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

function [ref, ref_dot] = lemniscata_reference_3d(t, cfg)
    phase_x = 2.0 * pi * t / 40.0;
    phase_y = 4.0 * pi * t / 40.0;
    
    ref = [0.75 * sin(phase_x); ...
           0.75 * sin(phase_y); ...
           cfg.z_hover; ...
           cfg.psi_ref];
    
    ref_dot = [0.75 * (2.0 * pi / 40.0) * cos(phase_x); ...
               0.75 * (4.0 * pi / 40.0) * cos(phase_y); ...
               0.0; ...
               0.0];
end

function [v_d, err] = outer_loop_hover(pos, psi, ref, cfg)
    err = ref - [pos; psi];
    err(4) = wrapToPi(err(4));
    
    v_d = zeros(4, 1);
    v_d(1) = cfg.ks_xy * tanh((cfg.kp_xy / cfg.ks_xy) * err(1));
    v_d(2) = cfg.ks_xy * tanh((cfg.kp_xy / cfg.ks_xy) * err(2));
    v_d(3) = cfg.ks_z * tanh((cfg.kp_z / cfg.ks_z) * err(3));
    v_d(4) = cfg.ks_psi * tanh((cfg.kp_psi / cfg.ks_psi) * err(4));
end

function [v_d, err] = outer_loop_lemniscate(pos, psi, ref, ref_dot, cfg)
    err = ref - [pos; psi];
    err(4) = wrapToPi(err(4));
    
    fb = zeros(4, 1);
    fb(1) = cfg.ks_xy * tanh((cfg.kp_xy / cfg.ks_xy) * err(1));
    fb(2) = cfg.ks_xy * tanh((cfg.kp_xy / cfg.ks_xy) * err(2));
    fb(3) = cfg.ks_z * tanh((cfg.kp_z / cfg.ks_z) * err(3));
    fb(4) = cfg.ks_psi * tanh((cfg.kp_psi / cfg.ks_psi) * err(4));
    
    v_d = ref_dot + fb;
end

function v_state = inner_loop_bebop(v_d, v_state, cfg)
    F1 = cfg.f1_bebop;
    F2 = cfg.f2_bebop;
    KD = cfg.kd_bebop;
    T = cfg.T;
    
    v_tilde = v_d - v_state;
    u_control = F1 \ (KD * v_tilde + F2 * v_state);
    v_dot = F1 * u_control - F2 * v_state;
    v_state = v_state + T * v_dot;
end

function v_body = velocity_to_body_frame(v_global, psi)
    R = [cos(psi), sin(psi), 0, 0; ...
        -sin(psi), cos(psi), 0, 0; ...
         0, 0, 1, 0; ...
         0, 0, 0, 1];
    v_body = R * v_global;
end

function v_clamped = clamp_velocities(v, cfg)
    v_clamped = v;
    v_clamped(1) = clamp_scalar(v(1), cfg.vxy_max_safe);
    v_clamped(2) = clamp_scalar(v(2), cfg.vxy_max_safe);
    v_clamped(3) = clamp_scalar(v(3), cfg.vz_max_safe);
    v_clamped(4) = clamp_scalar(v(4), cfg.vyaw_max_safe);
end

function send_bebop_cmd(msg, pub, v)
    msg.Linear.X = v(1);
    msg.Linear.Y = v(2);
    msg.Linear.Z = v(3);
    msg.Angular.X = 0.0;
    msg.Angular.Y = 0.0;
    msg.Angular.Z = v(4);
    send(pub, msg);
end

function send_zero_cmd(msg, pub)
    send_bebop_cmd(msg, pub, [0.0; 0.0; 0.0; 0.0]);
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
    scaled = sign_raw * (abs(raw) - deadzone) / (1.0 - deadzone);
    scaled = max(min(scaled, 1.0), -1.0);
end

function y = clamp_scalar(x, limit)
    y = min(max(x, -limit), limit);
end

function save_bebop_results(hist_t, hist_pos, hist_yaw, hist_ref, hist_error, cfg)
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    out_dir = cfg.results_dir;
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    
    prefix = sprintf('bebop_%s_%s', cfg.mode, stamp);
    mat_file = fullfile(out_dir, [prefix, '.mat']);
    fig_file = fullfile(out_dir, [prefix, '_plot.png']);
    
    rms_xy = sqrt(mean(sum(hist_error(1:2, :).^2, 1)));
    rms_z = sqrt(mean(hist_error(3, :).^2));
    rms_3d = sqrt(mean(sum(hist_error(1:3, :).^2, 1)));
    
    results.meta.timestamp = stamp;
    results.meta.mode = cfg.mode;
    results.meta.rms_xy = rms_xy;
    results.meta.rms_z = rms_z;
    results.meta.rms_3d = rms_3d;
    results.hist_t = hist_t;
    results.hist_pos = hist_pos;
    results.hist_yaw = hist_yaw;
    results.hist_ref = hist_ref;
    results.hist_error = hist_error;
    results.cfg = cfg;
    save(mat_file, '-struct', 'results');
    
    fig = figure('Name', 'Teste Bebop 2', 'Visible', 'off', ...
        'Position', [100, 100, 1200, 800]);
    
    subplot(2, 2, 1);
    plot3(hist_ref(1, :), hist_ref(2, :), hist_ref(3, :), 'r--', 'LineWidth', 1.5);
    hold on;
    plot3(hist_pos(1, :), hist_pos(2, :), hist_pos(3, :), 'b-', 'LineWidth', 1.2);
    plot3(hist_pos(1, 1), hist_pos(2, 1), hist_pos(3, 1), 'go', 'MarkerSize', 8);
    plot3(hist_pos(1, end), hist_pos(2, end), hist_pos(3, end), 'ks', 'MarkerSize', 8);
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(sprintf('Trajetória 3D — %s', cfg.mode));
    legend('Referência', 'Bebop', 'Início', 'Fim', 'Location', 'best');
    view(45, 30);
    axis equal;
    
    subplot(2, 2, 2);
    plot(hist_ref(1, :), hist_ref(2, :), 'r--', 'LineWidth', 1.5);
    hold on;
    plot(hist_pos(1, :), hist_pos(2, :), 'b-', 'LineWidth', 1.2);
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Trajetória XY');
    axis equal;
    
    subplot(2, 2, 3);
    plot(hist_t, hist_pos(3, :), 'b-', 'LineWidth', 1.2);
    hold on;
    plot(hist_t, hist_ref(3, :), 'r--', 'LineWidth', 1.5);
    grid on;
    xlabel('Tempo (s)');
    ylabel('Z (m)');
    title(sprintf('Altitude (RMS z = %.3f m)', rms_z));
    legend('Bebop', 'Referência');
    
    subplot(2, 2, 4);
    plot(hist_t, hist_error(1, :), 'r-', 'DisplayName', 'Erro X');
    hold on;
    plot(hist_t, hist_error(2, :), 'g-', 'DisplayName', 'Erro Y');
    plot(hist_t, hist_error(3, :), 'b-', 'DisplayName', 'Erro Z');
    plot(hist_t, vecnorm(hist_error(1:3, :), 2, 1), 'k-', 'LineWidth', 1.2, 'DisplayName', '||erro||');
    grid on;
    xlabel('Tempo (s)');
    ylabel('Erro (m)');
    title(sprintf('Erros de posição (RMS 3D = %.3f m)', rms_3d));
    legend('Location', 'best');
    
    sgtitle(sprintf('Teste Bebop 2 — %s (%s)', cfg.mode, stamp));
    print(fig, fig_file, '-dpng', '-r150');
    close(fig);
    
    if cfg.save_gif && strcmp(cfg.mode, 'lemniscate')
        gif_file = fullfile(out_dir, [prefix, '_anim.gif']);
        save_bebop_gif(hist_t, hist_pos, hist_ref, cfg, gif_file);
    end
    
    fprintf('Erro RMS posição: XY=%.3f m, Z=%.3f m, 3D=%.3f m\n', rms_xy, rms_z, rms_3d);
    fprintf('Resultados salvos em %s:\n', out_dir);
    fprintf('  %s\n', mat_file);
    fprintf('  %s\n', fig_file);
    if cfg.save_gif && strcmp(cfg.mode, 'lemniscate')
        fprintf('  %s\n', gif_file);
    end
end

function save_bebop_gif(hist_t, hist_pos, hist_ref, cfg, gif_file)
    frame_step = max(1, round(cfg.gif_frame_step));
    delay = 1.0 / max(1, cfg.gif_fps);
    idx = 1:frame_step:numel(hist_t);
    n_frames = numel(idx);
    
    margin = 0.20;
    all_x = [hist_ref(1, :), hist_pos(1, :)];
    all_y = [hist_ref(2, :), hist_pos(2, :)];
    all_z = [hist_ref(3, :), hist_pos(3, :)];
    x_lim = [min(all_x) - margin, max(all_x) + margin];
    y_lim = [min(all_y) - margin, max(all_y) + margin];
    z_lim = [min(all_z) - margin, max(all_z) + margin];
    
    fig = figure('Name', 'Bebop GIF', 'Visible', 'off', ...
        'Position', [100, 100, 800, 600], 'Color', 'w');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    grid(ax, 'on');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    xlim(ax, x_lim);
    ylim(ax, y_lim);
    zlim(ax, z_lim);
    view(ax, 45, 30);
    
    plot3(ax, hist_ref(1, :), hist_ref(2, :), hist_ref(3, :), ...
        'Color', [1.0, 0.6, 0.6], 'LineStyle', '--', 'LineWidth', 1.2);
    
    h_trail = plot3(ax, nan, nan, nan, 'b-', 'LineWidth', 1.5);
    h_pos = plot3(ax, nan, nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    h_ref = plot3(ax, nan, nan, nan, 'r.', 'MarkerSize', 18);
    h_title = title(ax, '', 'FontSize', 11);
    
    fprintf('Gerando GIF (%d frames)... ', n_frames);
    
    for fi = 1:n_frames
        k = idx(fi);
        t_now = hist_t(k);
        
        set(h_trail, 'XData', hist_pos(1, 1:k), ...
            'YData', hist_pos(2, 1:k), 'ZData', hist_pos(3, 1:k));
        set(h_pos, 'XData', hist_pos(1, k), ...
            'YData', hist_pos(2, k), 'ZData', hist_pos(3, k));
        set(h_ref, 'XData', hist_ref(1, k), ...
            'YData', hist_ref(2, k), 'ZData', hist_ref(3, k));
        
        err = norm(hist_pos(:, k) - hist_ref(1:3, k));
        set(h_title, 'String', sprintf('t = %.1f s | erro = %.3f m | frame %d/%d', ...
            t_now, err, fi, n_frames));
        
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
