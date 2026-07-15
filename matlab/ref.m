%% A interface do ROS deve estar fechada antes de ser aberta. Para assegurar que ela esteja fechada,ponha este comando no início e no final do código.
rosshutdown;
%FORA DO LOOP DE CONTROLE %
%% Namespace do robô (corpo rígido no Motive / launch ROS)
% Exemplos: 'B1' (Bebop 2), 'L1' (LIMO)
namespace = 'B1';
%% Inicialização rede ROS MATLAB
rosinit('192.168.0.100'); % onde 192.168.0.100 é o IP do servidor ROS
%% Definição do tópico a ser inscrito e o tipo de mensagem a ser enviado/lido neste tópico
% cmd_vel
pub_cmdvel = rospublisher(sprintf('/%s/cmd_vel', namespace), 'geometry_msgs/Twist');
msg_cmdvel = rosmessage(pub_cmdvel);
% take off
pub_takeoff = rospublisher(sprintf('/%s/takeoff', namespace), 'std_msgs/Empty');
msg_takeoff = rosmessage(pub_takeoff);
% land
pub_land = rospublisher(sprintf('/%s/land', namespace), 'std_msgs/Empty');
msg_land = rosmessage(pub_land);
% leitura da pose do robô via OptiTrack
pose = rossubscriber(sprintf('vrpn_client_node/%s/pose', namespace));
%% Criação de objeto para o joystick em Matlab
J = vrjoystick(1);
%% Exemplo de uso dos botões e eixos do joystick
Analog = axis(J);
Digital = button(J);
%% DENTRO DO LOOP DE CONTROLE %%
%% Exemplo de envio de comandos u = [1; 0.5; -1.0; 0.3] via cmd_vel para o drone.
% Adaptar para o LIMO (msg_cmdvel.Linear.Y = 0; msg_cmdvel.Linear.Z = 0)
msg_cmdvel.Linear.X = 1;
msg_cmdvel.Linear.Y = 0.5;
msg_cmdvel.Linear.Z = -1;
msg_cmdvel.Angular.Z = 0.3;
send(pub_cmdvel,msg_cmdvel)
%% Exemplo de envio de comando para takeoff (para o drone decolar)
send(pub_takeoff,msg_takeoff);
%% Exemplo de envio de comando para land (para o drone pousar)
send(pub_land,msg_land);
%% Exemplo de leitura da pose do robô via OptiTrack
pose_latest = pose.LatestMessage.Pose
quat = [pose_latest.Orientation.W pose_latest.Orientation.X pose_latest.Orientation.Y pose_latest.Orientation.Z];
EulZYX = quat2eul(quat); % ângulos em radianos, na sequência ZYX
angles = [EulZYX(3); EulZYX(2); EulZYX(1)]; % converter para ângulos na sequência XYZ
position = [pose_latest.Position.X;pose_latest.Position.Y;pose_latest.Position.Z];
%% Exemplo de uso dos botões e eixos do joystick
Analog = axis(J);
Digital = button(J);
%% A interface do ROS deve ser fechada antes de ser reaberta.
%% Para assegurar isso, ponha este comando no final e no início do código.
rosshutdown;
