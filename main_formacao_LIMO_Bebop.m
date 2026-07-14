%   1) roscore no rosserver (192.168.0.100)
%   2) OptiTrack publicando as poses (corpos rigidos L1 e B1)
%   3) LIMO com limo_base.launch namespace:=L1  (modo diferencial/4wd)
%   4) Bebop com o launch do bebop_autonomy namespace:=B1
%   5) Joystick conectado (parada de emergencia)
clear; clc; close all;

%% ------------------------------------------------------------------ %%
%% 1) PARAMETROS (todos vindos da especificacao do projeto)           %%
%% ------------------------------------------------------------------ %%
T = 1/30;
Tsim = 120; % (3 periodos da lemniscata)
N = round(Tsim/T);

a = 0.10; % deslocamento do ponto de controle do LIMO (em metros)

% --- Formacao desejada: drone 1,5 m acima do LIMO ---
rho_f = 1.5; % distancia LIMO-drone
alpha_f = 0; % azimute [rad]
beta_f  = pi/2; % elevacao [rad] (90 graus -> drone na vertical)

% --- Obstaculo e zona de influencia ---
xo = -0.20; % centro da base
yo = 0.425; % centro da base
Robs = 0.15; % raio fisico do obstaculo 
Rinf = 0.25; % zona de influencia
nexp = 2; % expoente (par) da funcao potencial
aexp = 0.28; % espalhamento da funcao exponencial
bexp = 0.28; % espalhamento da funcao exponencial
Vd   = 0.05; % potencial desejado (limiar pequeno)
kobs = 1.0; % ganho da subtarefa de desvio

% --- Ganhos do controlador CINEMATICO da formacao (Eq. 5.7) ---
% Os valors de referencia que encontrei estão na pag 117 do livro
K = diag([0.8 0.8 0.8 1.5 1.0 1.5]); % valor de ref do livro: diag(0.2,0.2,0.2,3,1,1.8)
L = diag([0.30 0.30 0.30 0.8 0.6 0.8]); % valor de ref do livro: diag([1 1 1 1 1 1]);  saturacao tanh (menor em x,y)

% --- Ganhos do controle CARTESIANO do drone (robusto a singularidade beta=90) ---
Kp_B = diag([1.0 1.0 1.2]); % ganho de posicao do drone [x y z]
Ls_B = diag([0.6 0.6 0.6]); % saturacao tanh do drone

% --- Ganhos dos COMPENSADORES DINAMICOS (Eq. 4.44 e 4.47) ---
KD_L = diag([4 4]);
KD_B = diag([4 4 4 4]); % [vx ; vy ; vz ; psidot]

% --- Modelo dinamico do LIMO (da especificacao, Eq. 3.1-3.6) ---
th = [0.1521 0.0953 0.0031 0.9840 -0.0451 1.6422];

% --- Modelo dinamico do Bebop (f1, f2 da especificacao):  vdot = f1*u - f2*v
f1 = diag([0.8417 0.8354 3.966  9.8524]);
f2 = diag([0.18227 0.17095 4.001 4.7295]);

% --- Saturacoes fisicas dos comandos ---
umax_L = 0.30;  wmax_L = 1.20;   % LIMO  [m/s] e [rad/s]
umax_B = 1.0;                    % Bebop: cmd_vel normalizado em +/-1

% #### Indicação da IA é utilizar um filtro passa baixa para a estimativa de velocidade 
% --- Filtro passa-baixa da estimativa de velocidade (0<alpha<=1) ---
% >>> FILTRO DESATIVADO (comentado). Descomente para reativar no teste. <<<
% alpha_f_vel = 0.3;   % no hardware a pose tem ruido -> filtra mais (menor alpha)

% --- Joystick ---
BTN_STOP = 1; % botao de parada de emergencia

%% ------------------------------------------------------------------ %%
%% 2) INICIALIZACAO DO ROS + OptiTrack + DECOLAGEM                    %%
%% ------------------------------------------------------------------ %%
rosshutdown;
rosinit('192.168.0.100');

% --- LIMO (namespace L1) ---
pub_L  = rospublisher('/L1/cmd_vel','geometry_msgs/Twist'); 
msg_L = rosmessage(pub_L);
% ponte OptiTrack: vrpn_client_node  (alternativa do lab: /natnet_ros/L1/pose)
pose_L = rossubscriber('/vrpn_client_node/L1/pose','geometry_msgs/PoseStamped');

% --- Bebop (namespace B1) ---
pub_B  = rospublisher('/B1/cmd_vel','geometry_msgs/Twist'); 
msg_B  = rosmessage(pub_B);
pub_TO = rospublisher('/B1/takeoff','std_msgs/Empty');      
msg_TO = rosmessage(pub_TO);
pub_LD = rospublisher('/B1/land','std_msgs/Empty');         
msg_LD = rosmessage(pub_LD);
% ponte OptiTrack: vrpn_client_node  (alternativa do lab: /natnet_ros/B1/pose)
pose_B = rossubscriber('/vrpn_client_node/B1/pose','geometry_msgs/PoseStamped');

% --- Joystick (parada de emergencia) ---
J = vrjoystick(1);

% --- Aguarda a primeira pose de cada robo (timeout 10 s) ---
fprintf('Aguardando poses do OptiTrack (L1 e B1)...\n');
receive(pose_L,10);
%receive(pose_B,10);
fprintf('Poses OK.\n');

% --- DECOLAGEM do drone e estabilizacao em hover ---
%fprintf('Decolando o Bebop... (Ctrl+C aborta)\n');
%send(pub_TO,msg_TO);
%pause(5);

% --- Le pose inicial (com o drone ja em hover) e inicializa memorias ---
[x1,y1,z1,psi1] = ler_pose(pose_L);

% para testar o LIMO sem o drone
x2=x1; y2=y1; z2=1.5; psi2=0;%[x2,y2,z2,psi2] = ler_pose(pose_B);


poseL_ant = [x1;y1];
poseL_psi_ant = psi1;
poseB_ant = [x2;y2;z2];
poseB_psi_ant = psi2;
vLmeas_f = [0;0];
vBmeas_f = [0;0;0;0];
vd_L_ant = [0;0];
vd_B_ant = [0;0;0;0]; % para dvd (feedforward interno)

%% ------------------------------------------------------------------ %%
%% 3) Histórico para os graficos                                      %%
%% ------------------------------------------------------------------ %%
H.t = zeros(1,N);
H.q = zeros(6,N);
H.qd = zeros(6,N);
H.p1 = zeros(2,N);
H.p2 = zeros(3,N);
H.dobs = zeros(1,N);
H.cmdL = zeros(2,N);
H.cmdB = zeros(4,N);

%% ------------------------------------------------------------------ %%
%% 4) LOOP DE CONTROLE                                                %%
%% ------------------------------------------------------------------ %%
fprintf('Iniciando formacao. Botao %d do joystick = PARAR.\n', BTN_STOP);
t0 = tic;   kf = 0;
try
for k = 1:N
    tloop = tic;
    t = toc(t0);

    % ---- Parada de emergencia (joystick) ----
    btns = button(J);
    if numel(btns) >= BTN_STOP && btns(BTN_STOP)
        fprintf('Parada solicitada pelo joystick.\n'); break;
    end

    % ============ (0) LEITURA DE POSE (OptiTrack) ============
    [x1,y1,z1,psi1] = ler_pose(pose_L);
    x2=x1; y2=y1; z2=1.5; psi2=0;%[x2,y2,z2,psi2] = ler_pose(pose_B);

    % Matrizes de cinematica inversa de cada robo
    A1inv = [ cos(psi1)        sin(psi1);          % LIMO (Eq. 2.16, versao 2x2)
             -sin(psi1)/a      cos(psi1)/a ];
    A2inv = [ cos(psi2)  sin(psi2)  0;             % Bebop (Eq. 2.26, rotacao de psi)
             -sin(psi2)  cos(psi2)  0;
              0          0          1 ];

    % ============ (1) ESTIMATIVA DA VELOCIDADE ATUAL (corpo) ============
    velWL = estimar_vel([x1;y1], poseL_ant, T); % [xdot1; ydot1]
    velWB = estimar_vel([x2;y2;z2], poseB_ant, T); % [xdot2; ydot2; zdot2]
    psidot2 = wrap_pi(psi2-poseB_psi_ant)/T;
    vL_meas = A1inv*velWL; % [u ; w]
    vB_meas = [A2inv*velWB; psidot2]; % [vx ; vy ; vz ; psidot]

    % ---- FILTRO passa-baixa DESATIVADO (comentado); usando velocidade CRUA ----
    % Para REATIVAR: descomente as 2 linhas do filtro, comente as 2 linhas cruas
    % e descomente 'alpha_f_vel' na secao de parametros.
    % vLmeas_f = filtro_pb(vL_meas, vLmeas_f, alpha_f_vel);
    % vBmeas_f = filtro_pb(vB_meas, vBmeas_f, alpha_f_vel);

    vLmeas_f = vL_meas;
    vBmeas_f = vB_meas;

    % ============ (2) ESTADO DA FORMACAO:  x -> q   (Eq. 5.5a) ============
    dx=x2-x1; 
    dy=y2-y1; 
    dz=z2-z1;
    q = [ x1;
          y1;
          z1;
          sqrt(dx^2+dy^2+dz^2);
          atan2(dy,dx);
          atan2(dz, sqrt(dx^2+dy^2)) ];

    % ============ lemniscata de Bernoulli ============
    xd  = 0.75*sin(2*pi*t/40);
    dxd = 0.75*(2*pi/40)*cos(2*pi*t/40);
    yd  = 0.75*sin(4*pi*t/40);
    dyd = 0.75*(4*pi/40)*cos(4*pi*t/40);
    qd  = [xd; yd; 0; rho_f; alpha_f; beta_f];
    dqd = [dxd; dyd; 0; 0; 0; 0];

    % ============ CONTROLADOR CINEMATICO DA FORMACAO (Eq. 5.7) ============
    qtil = qd - q;
    qtil(5) = wrap_pi(qtil(5));   qtil(6) = wrap_pi(qtil(6));   % erros angulares
    dqr = dqd + L*tanh(L\(K*qtil));

    % ============ (5) FORMACAO -> ROBOS (mundo):  Jacobiano (Eq. 5.6c) ============
    ca=cos(q(5)); sa=sin(q(5)); cb=cos(q(6)); sb=sin(q(6)); rf=q(4);
    % Jacobiano da Eq. (5.6c), escrito na convencao do enunciado (alpha=azimute,
% beta=elevacao) — trocada em relacao ao livro (alpha=elevacao, beta=azimute).

    Jinv = [ 1 0 0     0            0            0;
             0 1 0     0            0            0;
             0 0 1     0            0            0;
             1 0 0   ca*cb   -rf*sa*cb   -rf*ca*sb;
             0 1 0   sa*cb    rf*ca*cb   -rf*sa*sb;
             0 0 1     sb          0        rf*cb ];
    dx_form = Jinv*dqr;

    % ============ (6) DESVIO DE OBSTACULO por ESPACO NULO (NSB) ============
    dobs = hypot(x1-xo, y1-yo); % distancia do LIMO ao obstaculo
    if dobs < Rinf
        V  = exp( -((x1-xo)/aexp)^nexp - ((y1-yo)/bexp)^nexp );     % Eq. 5.20
        Jo = zeros(1,6);
        Jo(1) = -nexp*((x1-xo)^(nexp-1)/aexp^nexp)*V;   % dV/dx1
        Jo(2) = -nexp*((y1-yo)^(nexp-1)/bexp^nexp)*V;   % dV/dy1
        Jo_pinv = Jo'/(Jo*Jo' + 1e-9);
        dx_obs = Jo_pinv*( 0 + kobs*(Vd - V) );                     % Eq. 5.22
        dx_r   = dx_obs + (eye(6) - Jo_pinv*Jo)*dx_form;            % Eq. 5.9
    else
        dx_r = dx_form;
    end
    dx1 = dx_r(1:2);       % LIMO -> [xdot1; ydot1] (mundo, ja com desvio NSB)

    % ============ (6b) DRONE: controle CARTESIANO relativo ============
    % JUSTIFICATIVA (nao consta no livro; adaptacao de implementacao):
    %   Em beta=90 (drone na vertical) o Jacobiano esferico (Eq. 5.6c) e singular
    %   no azimute alpha. Como o enunciado inicia o drone deslocado lateralmente,
    %   surge grande erro de azimute no transitorio, que faz o mapeamento por
    %   Jacobiano "girar" o drone. No exemplo do livro (Sec. 5.6) isso nao ocorre
    %   porque o drone parte pousado sobre o robo terrestre e sobe reto.
    %   Solucao: controlar a POSICAO do drone diretamente, mantendo a estrutura
    %   proporcional+feedforward+tanh da Eq. (5.7), com o alvo p2d dado pela
    %   transformacao inversa g(q) da Eq. (5.5b). Com beta=90 e alpha=0,
    %   p2d = (x1, y1, 1.5): drone na vertical do ponto de controle do LIMO.
    p2d = [ x1 + rho_f*cos(alpha_f)*cos(beta_f);
            y1 + rho_f*sin(alpha_f)*cos(beta_f);
            z1 + rho_f*sin(beta_f) ];
    p2  = [x2; y2; z2];
    ff2 = [dx1; 0];                 % feedforward: drone acompanha o LIMO em XY; Z constante
    dx2 = ff2 + Ls_B*tanh(Ls_B\(Kp_B*(p2d - p2)));

    % ============ (7) MUNDO -> CORPO: velocidades desejadas de cada robo ============
    vd_L = A1inv*dx1; % [u_d ; w_d]
    vd_B = [A2inv*dx2; 0]; % [vx_d ; vy_d ; vz_d ; psidot_d(=0)]

    if k==1, dvd_L=[0;0]; dvd_B=[0;0;0;0];
    else,    dvd_L=(vd_L-vd_L_ant)/T;  dvd_B=(vd_B-vd_B_ant)/T;  end

    % ============ (8) COMPENSADOR DINAMICO DO LIMO (Eq. 4.44) ============
    w  = vLmeas_f(2);
    Hm = [th(1) 0; 0 th(2)];
    Cm = [th(4) -th(3)*w; th(5)*w th(6)];
    cmdL = Hm*(dvd_L + KD_L*(vd_L - vLmeas_f)) + Cm*vLmeas_f;   % [u_ref; w_ref]
    cmdL = [saturar(cmdL(1),umax_L); saturar(cmdL(2),wmax_L)];

    % ============ (9) COMPENSADOR DINAMICO DO BEBOP (Eq. 4.47) ============
    cmdB = f1\(dvd_B + KD_B*(vd_B - vBmeas_f) + f2*vBmeas_f);   % [uvx;uvy;uvz;upsi]
    %cmdB = saturar(cmdB, umax_B);

    % ============ (10) ENVIO DE COMANDOS via ROS ============
    msg_L.Linear.X = cmdL(1); 
    msg_L.Linear.Y = 0; 
    msg_L.Linear.Z = 0;
    msg_L.Angular.Z = cmdL(2);                    
    send(pub_L,msg_L);

    %msg_B.Linear.X = cmdB(1); 
    %msg_B.Linear.Y = cmdB(2);
    %msg_B.Linear.Z = cmdB(3); 
    %msg_B.Angular.Z = cmdB(4);   
    %send(pub_B,msg_B);

    % ============ HISTORICO + atualiza memorias ============
    H.t(k)=t; H.q(:,k)=q; H.qd(:,k)=qd; H.p1(:,k)=[x1;y1]; H.p2(:,k)=[x2;y2;z2];
    H.dobs(k)=dobs; H.cmdL(:,k)=cmdL; H.cmdB(:,k)=cmdB;   kf = k;
    poseL_ant=[x1;y1]; poseL_psi_ant=psi1;
    poseB_ant=[x2;y2;z2]; poseB_psi_ant=psi2;
    vd_L_ant=vd_L; vd_B_ant=vd_B;

    % ---- monitor (1x por segundo) ----
    if mod(k,30)==0
        fprintf('t=%5.1fs | LIMO=(%+.2f,%+.2f) rho=%.2f beta=%4.0f deg | dobs=%.2f\n',...
                t, x1, y1, q(4), rad2deg(q(6)), dobs);
    end

    % ---- fecha o loop em ~30 Hz ----
    pause(max(0, T - toc(tloop)));
end
catch ME
    fprintf(2,'ERRO no loop: %s\n', ME.message);
end

%% ------------------------------------------------------------------ %%
%% 5) ENCERRAMENTO SEGURO: para o LIMO e POUSA o drone                %%
%% ------------------------------------------------------------------ %%
fprintf('Encerrando: parando LIMO e pousando o Bebop...\n');
msg_L.Linear.X = 0; 
msg_L.Linear.Y = 0; 
msg_L.Linear.Z = 0; 
msg_L.Angular.Z = 0;
send(pub_L,msg_L);
msg_B.Linear.X = 0; msg_B.Linear.Y = 0; msg_B.Linear.Z = 0; msg_B.Angular.Z = 0;
%send(pub_B,msg_B);
%send(pub_LD,msg_LD); % LAND
pause(0.5);
rosshutdown;
fprintf('Finalizado.\n');

%% ------------------------------------------------------------------ %%
%% 6) GRAFICOS (usa apenas os dados efetivamente coletados)           %%
%% ------------------------------------------------------------------ %%
if kf > 1
    idx = 1:kf;
    Ht=H.t(idx); Hq=H.q(:,idx); Hqd=H.qd(:,idx);
    Hp1=H.p1(:,idx); Hp2=H.p2(:,idx); Hdobs=H.dobs(idx);

    figure('Name','Trajetorias XY','Color','w'); hold on; axis equal; grid on;
    td = linspace(0,max(Ht),600);
    plot(0.75*sin(2*pi*td/40), 0.75*sin(4*pi*td/40),'k--','DisplayName','Lemniscata desejada');
    plot(Hp1(1,:),Hp1(2,:),'b','LineWidth',1.5,'DisplayName','LIMO');
    plot(Hp2(1,:),Hp2(2,:),'r','LineWidth',1.2,'DisplayName','Bebop (proj. XY)');
    plot(Hp1(1,1),Hp1(2,1),'bo','MarkerFaceColor','b','DisplayName','LIMO inicio');
    desenhar_circulo(xo,yo,Robs,'k-');
    desenhar_circulo(xo,yo,Rinf,'k:');
    xlabel('x [m]'); ylabel('y [m]'); title('Formacao seguindo a lemniscata + desvio');
    legend('Location','bestoutside');

    rot = {'x_f [m]','y_f [m]','z_f [m]','\rho_f [m]','\alpha_f [rad]','\beta_f [rad]'};
    figure('Name','Variaveis da formacao','Color','w');
    for i=1:6
        subplot(3,2,i); hold on; grid on;
        plot(Ht,Hqd(i,:),'k--','LineWidth',1.2);
        plot(Ht,Hq(i,:),'b','LineWidth',1.2);
        ylabel(rot{i}); if i>=5, xlabel('t [s]'); end
        if i==1, legend('desejado','real','Location','best'); end
    end
    sgtitle('Variaveis da formacao: desejado (--) x real (—)');

    figure('Name','Distancia ao obstaculo','Color','w'); hold on; grid on;
    plot(Ht,Hdobs,'b','LineWidth',1.4);
    yline(Rinf,'k:','zona de influencia'); yline(Robs,'r--','raio do obstaculo');
    xlabel('t [s]'); ylabel('distancia LIMO-obstaculo [m]');
    title('Distancia do LIMO ao obstaculo');
end

%% ================================================================== %%
%% ================        FUNCOES LOCAIS         =================== %%
%% ================================================================== %%

function [x,y,z,psi] = ler_pose(sub)
    p    = sub.LatestMessage;
    quat = [p.Pose.Orientation.W p.Pose.Orientation.X ...
            p.Pose.Orientation.Y p.Pose.Orientation.Z];
    eul  = quat2eul(quat);          % [yaw pitch roll] (sequencia ZYX)
    x   = p.Pose.Position.X;
    y   = p.Pose.Position.Y;
    z   = p.Pose.Position.Z;
    psi = eul(1);                   % yaw
end

function vw = estimar_vel(pos, pos_ant, T)
    vw = (pos - pos_ant)/T;
end

function vf = filtro_pb(v, vf_ant, alpha)
    vf = (1-alpha)*vf_ant + alpha*v;
end

function y = saturar(u, umax)
    y = max(min(u, umax), -umax);
end

function ang = wrap_pi(ang)
    ang = atan2(sin(ang), cos(ang));
end

function desenhar_circulo(xc,yc,r,estilo)
    th = linspace(0,2*pi,100);
    plot(xc+r*cos(th), yc+r*sin(th), estilo, 'HandleVisibility','off');
end