% Bisecção: comando fixo (open-loop, sem controlador) para isolar se o
% problema é o CÁLCULO do cmd_vel (teste_limo_lemniscata.m) ou o ENVIO/
% canal ROS até o LIMO. Se o LIMO andar aqui, o problema está no cálculo.
% Se não andar aqui também, o problema é no canal (driver/rede/firmware).
clear; clc; close all;

%% Configuração
cfg.T = 1 / 30;
v_forward = 0.15; % m/s — mesma ordem de grandeza do modo 'pulse' que já funcionou
duration_s = 3.0; % s

%% ROS
rosshutdown;
rosinit('http://192.168.0.100:11311');
pub_L = rospublisher('/L1/cmd_vel', 'geometry_msgs/Twist');
msg_L = rosmessage(pub_L);

%% Envio do comando fixo
fprintf('Enviando v=%.2f m/s por %.1f s. Ctrl+C para interromper.\n', v_forward, duration_s);
N = round(duration_s / cfg.T);
try
    for k = 1:N
        tloop = tic;

        msg_L.Linear.X = v_forward;
        msg_L.Linear.Y = 0;
        msg_L.Linear.Z = 0;
        msg_L.Angular.X = 0;
        msg_L.Angular.Y = 0;
        msg_L.Angular.Z = 0;
        send(pub_L, msg_L);

        if mod(k, 30) == 0
            fprintf('t=%.1fs | enviando Linear.X=%.2f\n', k * cfg.T, v_forward);
        end
        pause(max(0, cfg.T - toc(tloop)));
    end
catch ME
    fprintf(2, 'ERRO no loop: %s\n', ME.message);
end

%% Encerramento
msg_L.Linear.X = 0; msg_L.Linear.Y = 0; msg_L.Linear.Z = 0;
msg_L.Angular.X = 0; msg_L.Angular.Y = 0; msg_L.Angular.Z = 0;
send(pub_L, msg_L);
pause(0.3);
rosshutdown;
fprintf('Encerrado.\n');
