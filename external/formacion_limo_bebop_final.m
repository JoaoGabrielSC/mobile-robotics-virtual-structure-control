%% PROYECTO FINAL: Formación LIMO (L1) y Bebop 2 (B1)
% Control de Estructura Virtual + Compensadores Dinámicos + Gráficas y CSV + Fin Automático

clear; clc; rosshutdown; pause(1);

% ========================================================================
% 1. PARÁMETROS DEL SISTEMA Y LA FORMACIÓN
% ========================================================================
T = 1/30; % Frecuencia de muestreo de 30 Hz

% Cinemática extendida LIMO (L1)
a = 0.10; % Punto de control desplazado 10 cm
alfa = 0; % Ángulo de 0 grados con el eje X

% Formación Bebop 2 (B1)
altura_deseada = 1.5; % z_f = 1.5 m

% Obstáculo (Balde cilíndrico)
obs_center = [-0.20; 0.425];
r_obs = 0.15;                
R_infl = 0.25;               
K_obs = 0.18;                 

% Ganancias Lazo Cinemático (Externo)
K_kin_L1 = diag([1.5, 1.5]);
Kp_B1    = diag([2.0, 2.0, 2.0]); 

% Ganancias Lazo Dinámico (Interno)
Kd_L1 = diag([2.0, 2.0]);
Kd_B1 = diag([2.0, 2.0, 1.8, 5.0]); 

% Parámetros Dinámicos LIMO (L1)
th = [0.1521, 0.0953, 0.0031, 0.9840, -0.0451, 1.6422];
H_L1 = diag([th(1), th(2)]);

% Parámetros Dinámicos Bebop 2 (B1)
f1_B1 = diag([0.8417, 0.8354, 3.9660, 9.8524]);
f2_B1 = diag([0.18227, 0.17095, 4.0010, 4.7295]);

% Límites de Seguridad (Pared Virtual)
limite_xy = 1.8; 
limite_z  = 1.8;

% ========================================================================
% 2. INICIALIZACIÓN ROS Y JOYSTICK
% ========================================================================
fprintf('Iniciando ROS...\n');
rosinit('192.168.0.100'); 

% Publishers
pub_L1  = rospublisher('/L1/cmd_vel', 'geometry_msgs/Twist');
pub_B1  = rospublisher('/B1/cmd_vel', 'geometry_msgs/Twist');
pub_tkf = rospublisher('/B1/takeoff', 'std_msgs/Empty');
pub_lnd = rospublisher('/B1/land', 'std_msgs/Empty');

msg_L1  = rosmessage(pub_L1);
msg_B1  = rosmessage(pub_B1);
msg_tkf = rosmessage(pub_tkf);
msg_lnd = rosmessage(pub_lnd);

% Subscribers OptiTrack
sub_L1 = rossubscriber('/natnet_ros/L1/pose', 'geometry_msgs/PoseStamped');
sub_B1 = rossubscriber('/natnet_ros/B1/pose', 'geometry_msgs/PoseStamped');

% Joystick y Mapeo de Xbox Series
J = vrjoystick(1);
BOTON_A = 1; 
BOTON_B = 2; 

fprintf('Esperando datos de OptiTrack...\n');
pause(2);
if isempty(sub_L1.LatestMessage) || isempty(sub_B1.LatestMessage)
    error('Fallo en OptiTrack. Verifica que L1 y B1 estén visibles en Motive. Abortando.');
end

% ========================================================================
% 3. VARIABLES DE ESTADO Y MEMORIA PARA CSV/GRÁFICAS
% ========================================================================
tempo = 0;
volando = false;
emergencia = false;
print_counter = 0;

% Variables para el fin de la misión
fin_mision = false;
t_separacion = 0;

pos_L1_prev = []; pos_B1_prev = [];
yaw_L1_prev = 0;  yaw_B1_prev = 0;
vd_L1_prev  = [0; 0];
vd_B1_prev  = [0; 0; 0; 0];

ultimo_msg_L1 = 0; ultimo_msg_B1 = 0;
t_recibido_L1 = tic; t_recibido_B1 = tic;

% Históricos para CSV y Gráficas
hist_tempo = [];
hist_pd_L1 = []; hist_pos_L1 = [];
hist_pd_B1 = []; hist_pos_B1 = [];

fprintf('\n=== SISTEMA LISTO ===\n');
fprintf('Boton A (Verde): DESPEGAR E INICIAR TRAYECTORIA\n');
fprintf('Boton B (Rojo) : DETENER Y ATERRIZAR\n\n');

% ========================================================================
% 4. BUCLE PRINCIPAL DE CONTROL 
% ========================================================================
try
    while true
        tic;
        
        % --------------------------------------------------------
        % 4.1. MÁQUINA DE ESTADOS Y PARADA
        % --------------------------------------------------------
        if button(J, BOTON_B) == 1
            emergencia = true;
            fprintf('\n🚨 FINALIZANDO: Boton B presionado. Deteniendo robots...\n');
            break;
            
        elseif button(J, BOTON_A) == 1 && ~volando
            fprintf('🛫 Despegando Bebop 2...\n');
            %send(pub_tkf, msg_tkf);
            %pause(5); % Esperar estabilización a ~1m
            volando = true;
            tempo = 0; 
            fprintf('Iniciando seguimiento de trayectoria...\n\n');
        end
        
        if ~volando
            pause(T); continue;
        end
        
        % --------------------------------------------------------
        % 4.2. LECTURA DE SENSORES Y TIMEOUT OPTITRACK
        % --------------------------------------------------------
        msg_1 = sub_L1.LatestMessage;
        msg_2 = sub_B1.LatestMessage;
        
        if ~isempty(msg_1)
            ts_1 = double(msg_1.Header.Stamp.Sec) + double(msg_1.Header.Stamp.Nsec)*1e-9;
            if ts_1 > ultimo_msg_L1
                t_recibido_L1 = tic; ultimo_msg_L1 = ts_1;
                pose_1 = msg_1.Pose;
                pos_L1 = [pose_1.Position.X; pose_1.Position.Y];
                quat_1 = [pose_1.Orientation.W, pose_1.Orientation.X, pose_1.Orientation.Y, pose_1.Orientation.Z];
                eul_1 = quat2eul(quat_1); 
                psi_L1 = eul_1(1);
            end
        end
        
        if ~isempty(msg_2)
            ts_2 = double(msg_2.Header.Stamp.Sec) + double(msg_2.Header.Stamp.Nsec)*1e-9;
            if ts_2 > ultimo_msg_B1
                t_recibido_B1 = tic; ultimo_msg_B1 = ts_2;
                pose_2 = msg_2.Pose;
                pos_B1 = [pose_2.Position.X; pose_2.Position.Y; pose_2.Position.Z];
                quat_2 = [pose_2.Orientation.W, pose_2.Orientation.X, pose_2.Orientation.Y, pose_2.Orientation.Z];
                eul_2 = quat2eul(quat_2); 
                psi_B1 = eul_2(1);
            end
        end
        
        if toc(t_recibido_L1) > 0.5 || toc(t_recibido_B1) > 0.5
            fprintf('🚨 EMERGENCIA: OptiTrack perdido por > 0.5s\n');
            emergencia = true; break;
        end
        
        if isempty(pos_L1_prev)
            pos_L1_prev = pos_L1; pos_B1_prev = pos_B1;
            yaw_L1_prev = psi_L1; yaw_B1_prev = psi_B1;
            continue;
        end
        
        % --------------------------------------------------------
        % 4.3. GEOFENCING (Pared Virtual)
        % --------------------------------------------------------
        if abs(pos_B1(1)) > limite_xy || abs(pos_B1(2)) > limite_xy || pos_B1(3) > limite_z
            fprintf('🚨 EMERGENCIA: Límite espacial vulnerado. Pos(%.2f, %.2f, %.2f)\n', pos_B1(1), pos_B1(2), pos_B1(3));
            emergencia = true; break;
        end
        
        % ========================================================
        % 4.4. GENERACIÓN DE TRAYECTORIAS Y SEPARACIÓN
        % ========================================================
        % Detectar el fin de las 2 vueltas (T = 40s * 2 = 80s)
        if tempo >= 80.0 && ~fin_mision
            fprintf('\n✅ DOS VUELTAS COMPLETADAS. Iniciando maniobra de separación...\n');
            fin_mision = true;
            t_separacion = tic;
        end
        
        if ~fin_mision
            % --- NAVEGACIÓN NORMAL (LEMNISCATA) ---
            w_tray = 2*pi/40;
            xd = 0.75 * sin(w_tray * tempo);
            yd = 0.75 * sin(2 * w_tray * tempo);
            vxd = 0.75 * w_tray * cos(w_tray * tempo);
            vyd = 0.75 * 2 * w_tray * cos(2 * w_tray * tempo);
            
            p_d = [xd; yd]; 
            v_d = [vxd; vyd];
            
            p_ctrl_L1 = pos_L1 + a * [cos(psi_L1); sin(psi_L1)];
            dist_obs = norm(p_ctrl_L1 - obs_center);
            
            if dist_obs < R_infl && dist_obs > 0.05
                n_vec = (p_ctrl_L1 - obs_center) / dist_obs;
                v_rep = K_obs * ((1/dist_obs) - (1/R_infl)) * n_vec;
                N_proj = eye(2) - n_vec * n_vec';
                v_kin = v_d + K_kin_L1 * (p_d - p_ctrl_L1);
                v_cmd_L1 = v_rep + N_proj * v_kin;
            else
                v_cmd_L1 = v_d + K_kin_L1 * (p_d - p_ctrl_L1);
            end
            
            % El Bebop sigue el techo virtual del LIMO
            pd_B1 = [p_ctrl_L1(1); p_ctrl_L1(2); altura_deseada];
            vd_B1_global = [v_cmd_L1(1); v_cmd_L1(2); 0]; 
            
        else
            % --- FASE DE SEPARACIÓN ---
            % 1. Frenar al LIMO
            v_cmd_L1 = [0; 0];
            
            % 2. Mover el Bebop 2 a una distancia segura (1 metro en X)
            pd_B1 = [pos_L1(1) + 1.0; pos_L1(2); altura_deseada];
            vd_B1_global = [0; 0; 0];
            
            % 3. Aterrizar después de 4 segundos
            if toc(t_separacion) > 4.0
                fprintf('🛬 Separación completada. Aterrizando el Bebop 2 de forma segura...\n');
                emergencia = true; % Reutilizamos la rutina de aterrizaje seguro
                break;
            end
        end
        
        % ========================================================
        % 4.5. CINEMÁTICA Y DINÁMICA LIMO (L1)
        % ========================================================
        u_d_L1 = v_cmd_L1(1)*cos(psi_L1) + v_cmd_L1(2)*sin(psi_L1);
        w_d_L1 = (-v_cmd_L1(1)*sin(psi_L1) + v_cmd_L1(2)*cos(psi_L1)) / a;
        vd_L1 = [u_d_L1; w_d_L1];
        
        u_curr = (norm(pos_L1 - pos_L1_prev)/T) * sign((pos_L1-pos_L1_prev)' * [cos(psi_L1); sin(psi_L1)]);
        w_curr = angdiff(yaw_L1_prev, psi_L1) / T;
        v_curr_L1 = [u_curr; w_curr];
        
        dot_vd_L1 = (vd_L1 - vd_L1_prev) / T;
        C_v = [0, -th(3)*w_curr; th(3)*w_curr, 0];
        F_v = [th(4), 0; 0, th(6) + (th(5) - th(3))*u_curr];
        
        vr_L1 = H_L1 * (dot_vd_L1 + Kd_L1*(vd_L1 - v_curr_L1)) + C_v*vd_L1 + F_v*vd_L1;
        
        % ========================================================
        % 4.6. CINEMÁTICA Y DINÁMICA BEBOP 2 (B1)
        % ========================================================
        yaw_d_B1 = 0; 
        v_cmd_B1_global = vd_B1_global + Kp_B1 * (pd_B1 - pos_B1);
        
        R_z = [cos(psi_B1) sin(psi_B1) 0; -sin(psi_B1) cos(psi_B1) 0; 0 0 1];
        vb_cmd_B1 = R_z * v_cmd_B1_global;
        w_cmd_B1 = 1.0 * angdiff(psi_B1, yaw_d_B1); 
        
        vd_B1 = [vb_cmd_B1; w_cmd_B1];
        
        vel_global_B1 = (pos_B1 - pos_B1_prev) / T;
        vb_curr_B1 = R_z * vel_global_B1;
        w_curr_B1 = angdiff(yaw_B1_prev, psi_B1) / T;
        v_curr_B1 = [vb_curr_B1; w_curr_B1];
        
        dot_vd_B1 = (vd_B1 - vd_B1_prev) / T;
        vr_B1 = f1_B1 \ (dot_vd_B1 + Kd_B1*(vd_B1 - v_curr_B1) + f2_B1*v_curr_B1);
        
        % ========================================================
        % 4.7. ENVÍO DE COMANDOS
        % ========================================================
        msg_L1.Linear.X = max(min(vr_L1(1), 0.6), -0.6);
        msg_L1.Angular.Z = max(min(vr_L1(2), 1.5), -1.5);
        send(pub_L1, msg_L1);
        
        msg_B1.Linear.X = max(min(vr_B1(1), 0.5), -0.5);
        msg_B1.Linear.Y = max(min(vr_B1(2), 0.5), -0.5);
        msg_B1.Linear.Z = max(min(vr_B1(3), 0.3), -0.3);
        msg_B1.Angular.Z = max(min(vr_B1(4), 0.5), -0.5);
        send(pub_B1, msg_B1);
        
        % ========================================================
        % 4.8. ALMACENAMIENTO Y MONITOREO (IMPRESIÓN CADA 0.5s)
        % ========================================================
        if ~fin_mision % Solo almacenamos la trayectoria, no la separación
            hist_tempo(end+1) = tempo;
            hist_pd_L1(:,end+1)  = p_d;
            hist_pos_L1(:,end+1) = pos_L1;
            hist_pd_B1(:,end+1)  = pd_B1;
            hist_pos_B1(:,end+1) = pos_B1;
        end
        
        print_counter = print_counter + 1;
        if mod(print_counter, 15) == 0
            if ~fin_mision
                err_L1 = norm(p_d - pos_L1);
                err_B1 = norm(pd_B1 - pos_B1);
                fprintf('[%.2f s] Err LIMO: %.3f m | Err Bebop: %.3f m\n', tempo, err_L1, err_B1);
            end
        end
        
        pos_L1_prev = pos_L1; yaw_L1_prev = psi_L1;
        pos_B1_prev = pos_B1; yaw_B1_prev = psi_B1;
        vd_L1_prev = vd_L1;   vd_B1_prev = vd_B1;
        
        tempo = tempo + T;
        t_gasto = toc;
        if t_gasto < T
            pause(T - t_gasto);
        end
    end
    
catch ME
    fprintf('🚨 EXCEPCIÓN CRÍTICA CAPTURADA:\n%s\n', ME.message);
    emergencia = true;
end

% ========================================================================
% 5. PROTOCOLO SEGURO DE ATERRIZAMIENTO 
% ========================================================================
if emergencia
    fprintf('=== EJECUTANDO PROTOCOLO DE PARADA Y ATERRIZAJE ===\n');
    
    % 1. Forzar Velocidades Nulas a ambos robots
    msg_L1.Linear.X = 0; msg_L1.Angular.Z = 0;
    send(pub_L1, msg_L1);
    
    msg_B1.Linear.X = 0; msg_B1.Linear.Y = 0; msg_B1.Linear.Z = 0; msg_B1.Angular.Z = 0;
    send(pub_B1, msg_B1);
    
    % 2. PAUSA CRÍTICA
    pause(0.5); 
    
    % 3. Enviar Comando de Aterrizaje Varias Veces
    fprintf('🛬 Aterrizando Bebop 2...\n');
    for i = 1:3
        send(pub_lnd, msg_lnd);
        pause(0.5);
    end
    
    pause(2.0); 
end

rosshutdown;
fprintf('Conexión ROS cerrada.\n\n');

% ========================================================================
% 6. EXPORTACIÓN CSV
% ========================================================================
if ~isempty(hist_tempo)
    disp('Exportando datos a CSV...');
    tabela = table(hist_tempo', hist_pd_L1(1,:)', hist_pd_L1(2,:)', ...
                   hist_pos_L1(1,:)', hist_pos_L1(2,:)', ...
                   hist_pd_B1(1,:)', hist_pd_B1(2,:)', hist_pd_B1(3,:)', ...
                   hist_pos_B1(1,:)', hist_pos_B1(2,:)', hist_pos_B1(3,:)', ...
                   'VariableNames', {'Tempo', 'L1_X_Ref', 'L1_Y_Ref', 'L1_X_Real', 'L1_Y_Real', ...
                                     'B1_X_Ref', 'B1_Y_Ref', 'B1_Z_Ref', 'B1_X_Real', 'B1_Y_Real', 'B1_Z_Real'});
    writetable(tabela, 'trayectorias_formacion.csv');
    disp('Archivo "trayectorias_formacion.csv" generado exitosamente.');

    % ========================================================================
    % 7. GENERACIÓN DE GRÁFICOS
    % ========================================================================
    disp('Generando Gráficas...');
    figure('Name', 'Resultados de la Formación', 'Position', [100, 100, 1000, 450]);
    
    % Gráfica LIMO (2D)
    subplot(1, 2, 1);
    hold on; grid on; axis equal;
    plot(hist_pd_L1(1,:), hist_pd_L1(2,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Referencia');
    plot(hist_pos_L1(1,:), hist_pos_L1(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Real');
    
    % Dibujar Obstáculo
    ang = linspace(0, 2*pi, 100);
    plot(obs_center(1) + r_obs*cos(ang), obs_center(2) + r_obs*sin(ang), 'r', 'LineWidth', 2, 'DisplayName', 'Obstáculo');
    plot(obs_center(1) + R_infl*cos(ang), obs_center(2) + R_infl*sin(ang), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Z. Influencia');
    
    title('LIMO (L1) - Trayectoria Lemniscata 2D');
    xlabel('X [m]'); ylabel('Y [m]');
    legend('Location', 'best');
    
    % Gráfica Bebop 2 (3D)
    subplot(1, 2, 2);
    hold on; grid on;
    plot3(hist_pd_B1(1,:), hist_pd_B1(2,:), hist_pd_B1(3,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Techo LIMO');
    plot3(hist_pos_B1(1,:), hist_pos_B1(2,:), hist_pos_B1(3,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Bebop 2 Real');
    
    title('Bebop 2 (B1) - Trayectoria 3D');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    legend('Location', 'best');
    view(45, 30); 
end
disp('Prueba Finalizada.');