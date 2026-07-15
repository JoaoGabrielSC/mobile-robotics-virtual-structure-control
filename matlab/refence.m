CÓDIGOS VALIDADOS EM MATLAB 2021

%% A interface do ROS deve ser fechada antes de ser reaberta. Ponha este comando no final e no início do código.
rosshutdown;

ANTES DE RODAR O MATLAB

%% No Motive, criar o corpo rígido do Crazyflie e nomeá-lo como cfX, em que X é o número do Crazyflie. O nome do Crazyflie é importante para o correto funcionamento do código.
%% Para o Limo, criar o corpo rígido e nomeá-lo como L1. O namespace para o OptiTrack do Limo não precisa ser o mesmo namespace do launch dele, mas isso é recomendado para facilitar a organização do código.
% Exemplo: L1
% Exemplo: cf7

%% Rodar o nó do natnet_ros para utilização do OptiTrack. Ele cria os tópicos de pose de todos os corpos rígidos criados no Motive.
roslaunch natnet_ros_cpp natnet_ros.launch

%% Inicialização do servidor do Crazyflie no ROS
% Ligue o Crazyflie.
% Inicie rodando "roslaunch crazyflie_server crazyflie_server.launch cfs:=[X]", em que X é o número do Crazyflie que você deseja controlar.
% Exemplo: "roslaunch crazyflie_server crazyflie_server.launch cfs:=[7]".

%% Inicialização do servidor do Limo no ROS
% Ligue o Limo.
% Conecte-se ao Limo via SSH no terminal do Linux:
% O comando padrão para a conexão é: "ssh agilex@192.168.0.XXX", em que XXX é o número que está adesivado na lateral do Limo.
% A senha padrão é: "agx"
% Para a utilização do Limo como diferencial ou carlike, dentro do terminal do Limo, rode o comando: "roslaunch limo_base limo_base.launch namespace:=L1".
% Para rodá-lo como omnidirecional, utilize o Limo 105 e rode o comando: "roslaunch limo_base limo_base.launch namespace:=L1 use_mcnamu:=true".
% Obs.: Para o Limo funcionar como diferencial ou omnidirecional, pelo menos uma das luzes da frente deve estar laranja antes de o launch ser rodado.
% Obs.: Para o Limo funcionar como carlike, as duas luzes da frente devem estar verdes.

ANTES DO LOOP DE CONTROLE

%% Inicialização da rede ROS no MATLAB
rosinit('192.168.0.100'); % em que 192.168.0.100 é o IP do servidor ROS

%% Coletar a posição dos robôs
pose = rossubscriber(['/natnet_ros/NAMESPACE/pose'],'geometry_msgs/PoseStamped');
% Onde o namespace é o nome do corpo rígido criado no Motive. Cada robô terá uma pose diferente.
% Exemplo: /natnet_ros/L1/pose
% Exemplo: /natnet_ros/cf7/pose

%% Publicação de comandos de controle para os robôs. Cada robô terá seu próprio cmd_vel.
% cmd_vel
pub_cmdvel = rospublisher(['/NAMESPACE/cmd_vel'],'geometry_msgs/Twist');
msg_cmdvel = rosmessage(pub_cmdvel);
% Exemplo: /L1/cmd_vel
% Exemplo: /cf7/cmd_vel

%% Serviços para o Crazyflie
% take off
takeoffClient = rossvcclient(['/cfX/takeoff'], 'std_srvs/Trigger');
takeoffRequest = rosmessage(takeoffClient);

% land
landClient = rossvcclient(['/cfX/land'], 'std_srvs/Trigger');
landRequest = rosmessage(landClient);

% kill
killClient = rossvcclient(['/cfX/kill'], 'std_srvs/Trigger');
killRequest = rosmessage(killClient);

%% Criação de objeto para utilizar o joystick em MATLAB
J = JoyControl;

DENTRO DO LOOP DE CONTROLE

%% Exemplo de leitura da pose do robô via OptiTrack
pose_latest = pose.LatestMessage.Pose
quat = [pose_latest.Orientation.W pose_latest.Orientation.X pose_latest.Orientation.Y pose_latest.Orientation.Z];
EulZYX = quat2eul(quat); % in rad
angles = [EulZYX(3); EulZYX(2); EulZYX(1)];
position = [pose_latest.Position.X;pose_latest.Position.Y;pose_latest.Position.Z];

%% Envio de comandos para o Crazyflie

%% Exemplo de envio de comando para takeoff (drone alçar voo)
%% Obs.: O takeoff deve ser feito antes de enviar comandos via cmd_vel e apenas uma vez.
takeoffResponse = call(takeoffClient, takeoffRequest, 'Timeout', 5);

%% Exemplo de envio de comando para land (drone pousar)
landResponse = call(landClient, landRequest, 'Timeout', 5);

%% Exemplo de envio de comando para kill (emergência). Desliga todos os motores do drone.
killResponse = call(killClient, killRequest, 'Timeout', 5);

%% Exemplo de envio de comandos u = [phi; theta; zdot; psidot] via cmd_vel
%% Lembre-se de limitar os valores máximos dos ângulos phi e theta, da velocidade angular em psi e da velocidade linear em z!
% u = [0.1; 0.1; 0.5; 0.3]
% phi e theta em radianos
% zdot em m/s
% psidot em rad/s
msg_cmdvel.Angular.X = 0.1;
msg_cmdvel.Angular.Y = 0.1;
msg_cmdvel.Linear.Z = 0.5;
msg_cmdvel.Angular.Z = 0.3;
send(pub_cmdvel,msg_cmdvel)

%% Exemplo de envio de comandos para o Limo
%% O envio de comandos para robôs terrestres do tipo diferencial deve ser feito via cmd_vel. O vetor de comandos u = [v; w] é definido como:
% v é a velocidade linear em m/s
% w é a velocidade angular em rad/s
msg_cmdvel.Linear.X = 0.5;
msg_cmdvel.Angular.Z = 0.3;
send(pub_cmdvel,msg_cmdvel)

%% Para o Limo no modo omnidirecional, adicionalmente, podem ser enviados comandos de velocidade linear em Y. O vetor de comandos u = [vx; vy; w] é definido como:
% vx é a velocidade linear em X em m/s
% vy é a velocidade linear em Y em m/s
% w é a velocidade angular em rad/s
msg_cmdvel.Linear.X = 0.5;
msg_cmdvel.Linear.Y = 0.2;
msg_cmdvel.Angular.Z = 0.3;
send(pub_cmdvel,msg_cmdvel)

%% Exemplo de uso dos botões e eixos do joystick
mRead(J);
Analog = J.pAnalog;
Digital = J.pDigital;

%% A interface do ROS deve ser fechada antes de ser reaberta. Ponha este comando no final e no início do código.
rosshutdown;
