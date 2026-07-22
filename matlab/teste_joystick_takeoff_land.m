% Teste isolado: takeoff/land do Bebop via joystick, sem controlador,
% sem lemniscata, sem LIMO. Só confirma que os botões disparam os
% serviços certos antes de qualquer voo com formação.
clear; clc; close all;

%% Configuração
BTN_TAKEOFF = 2; % mesmo botão de "confirmar início" usado em formacao_2.m
BTN_LAND = 1;     % mesmo botão de "parada" (BTN_STOP) usado em formacao_2.m
poll_dt = 0.05;   % s

%% ROS e joystick
rosshutdown;
rosinit('http://192.168.0.100:11311');
pub_TO = rospublisher('/B1/takeoff', 'std_msgs/Empty');
msg_TO = rosmessage(pub_TO);
pub_LD = rospublisher('/B1/land', 'std_msgs/Empty');
msg_LD = rosmessage(pub_LD);
J = vrjoystick(1);

fprintf('Botão %d = takeoff | Botão %d = land | Ctrl+C para encerrar.\n', BTN_TAKEOFF, BTN_LAND);

%% Loop: dispara takeoff/land na BORDA DE SUBIDA do botão (não repete enquanto segura)
btn_takeoff_ant = false;
btn_land_ant = false;
try
    while true
        btns = button(J);
        btn_takeoff = numel(btns) >= BTN_TAKEOFF && logical(btns(BTN_TAKEOFF));
        btn_land = numel(btns) >= BTN_LAND && logical(btns(BTN_LAND));

        if btn_takeoff && ~btn_takeoff_ant
            fprintf('[%s] Takeoff enviado.\n', datestr(now, 'HH:MM:SS'));
            send(pub_TO, msg_TO);
        end
        if btn_land && ~btn_land_ant
            fprintf('[%s] Land enviado.\n', datestr(now, 'HH:MM:SS'));
            send(pub_LD, msg_LD);
        end

        btn_takeoff_ant = btn_takeoff;
        btn_land_ant = btn_land;
        pause(poll_dt);
    end
catch ME
    fprintf(2, 'Encerrado: %s\n', ME.message);
end

%% Encerramento
rosshutdown;
