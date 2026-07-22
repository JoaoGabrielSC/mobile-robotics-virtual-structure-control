% ---------------------------------------------------------------------------
% VIRTUAL STRUCTURE FORMATION CONTROL
% LIMO (L1) + BEBOP (B1)
% GROUP LAB PRACTICAL MATLAB SCRIPT
% MATLAB + ROS + OPTITRACK
% ---------------------------------------------------------------------------

clc
clear
close all

% ---------------------------------------------------------------------------
% 1. ROS INITIALIZATION

rosshutdown
rosinit('192.168.0.100')


% ---------------------------------------------------------------------------
% 2. JOYSTICK INITIALIZATION

J = vrjoystick(1);
manual_mode = 0;
disp('Joystick Connected')


% ---------------------------------------------------------------------------
% 3. PARAMETERS

T=1/30;
tf=120;
time=0:T:tf;
a=0.10;
rho_f=1.5;
obs=[-0.2 0.425];
obs_radius=0.15;
influence=0.25;
vz_max=0.4;
yaw_max=100;
vmax=1;
wmax=1;

alpha_f = 0;                       % desired azimuth (rad)
beta_f  = pi/2;                    % desired elevation (rad) -> drone above LIMO
ku = 4; kw = 4; lu = 1; lw = 1;    % LIMO inner-loop gains / saturation
Lx = 0.8; Ly = 0.8;                % outer-loop (kinematic) saturation levels

% Gaussian potential field for obstacle avoidance (book eq. 5.20-5.22)
pot_a = 0.30;   % spread in x  (tunes how far the potential reaches)
pot_b = 0.30;   % spread in y
pot_n = 4;      % positive EVEN integer
kobs  = 0.5;    % avoidance gain
Vd    = 0.1;    % desired (low) potential value the robot is driven to

% ------------------
% TEST SWITCHES (set 1 = on, 0 = off) for isolated tests before joining all
USE_LIMO     = 1;         % 1 = command the LIMO      / 0 = LIMO off (no send)
USE_DRONE    = 1;         % 1 = command the CrazyFly  / 0 = drone off (no takeoff/send/land)
USE_OBSTACLE = 1;         % 1 = obstacle avoidance on / 0 = ignore the obstacle
TRAJ_MODE    = 1;         % 1 = full lemniscate       / 0 = go to a fixed point (pos_des)
pos_des      = [0 0 1];   % target [x y z] used when TRAJ_MODE = 0
                          % (z is the drone hover height; use 0 only for LIMO-only / ground)


% ---------------------------------------------------------------------------
% 4. LIMO PARAMETERS

theta1=0.1521;
theta2=0.0953;
theta3=0.0031;
theta4=0.9840;
theta5=-0.0451;
theta6=1.6422;


% ---------------------------------------------------------------------------
% 5. BEBOP PARAMETERS

f1=[...
0.8417 0 0 0
0 0.8354 0 0
0 0 3.9660 0
0 0 0 9.8524];

f2=[...
0.18227 0 0 0
0 0.17095 0 0
0 0 4.0010 0
0 0 0 4.7295];


% ---------------------------------------------------------------------------
% 6. PUBLISHERS

pubL=rospublisher(...
'/L1/cmd_vel',...
'geometry_msgs/Twist');
msgL=rosmessage(pubL);

% Crazy Fly

% pubB = rospublisher(['/cf7/cmd_vel'],'geometry_msgs/Twist');
% msg_cmdvel = rosmessage(pubB);
% msgB=rosmessage(pubB);

% pubTake=rossvcclient(['/cf7/takeoff'], 'std_srvs/Trigger');
% msgTake=rosmessage(pubTake);

% pubLand=rossvcclient(['/cf7/land'], 'std_srvs/Trigger');
% msgLand=rosmessage(pubLand);

% BEPOP

pubB=rospublisher('/B1/cmd_vel', 'geometry_msgs/Twist');
msgB=rosmessage(pubB);

pubTake=rospublisher('/B1/takeoff', 'std_msgs/Empty');
msgTake=rosmessage(pubTake);

pubLand=rospublisher('/B1/land', 'std_msgs/Empty');
msgLand=rosmessage(pubLand);

% ---------------------------------------------------------------------------
% 7. SUBSCRIBERS (OPTITRACK)

poseL=rossubscriber(...
'/natnet_ros/L1/pose');
poseB=rossubscriber(...
'/natnet_ros/B1/pose');
pause(3)

% Wait until the bodies IN USE are publishing before the first read.
while (USE_LIMO && isempty(poseL.LatestMessage)) || (USE_DRONE && isempty(poseB.LatestMessage))
    pause(0.05);
end


% ---------------------------------------------------------------------------
% 8. INITIAL POSITION CHECK

xL0 = 0.4;
yL0 = -0.25;
psi0 = 0;
xB0 = 0.4;
yB0 = 0.05;
zB0 = 0.03;

disp('Move robots to initial positions')
tol = 0.20;
ready = 0;
while ~ready
    if USE_LIMO
    PL = poseL.LatestMessage.Pose;
    xL = PL.Position.X;
    yL = PL.Position.Y;
    else
    xL = xL0; yL = yL0;
    end
    if USE_DRONE
    PB = poseB.LatestMessage.Pose;
    xB = PB.Position.X;
    yB = PB.Position.Y;
    zB = PB.Position.Z;
    else
    xB = xB0; yB = yB0; zB = zB0;
    end
    cond1 = ...
        abs(xL-xL0)<tol;
    cond2 = ...
        abs(yL-yL0)<tol;
    cond3 = ...
        abs(xB-xB0)<tol;
    cond4 = ...
        abs(yB-yB0)<tol;
    cond5 = ...
        abs(zB-zB0)<tol;
    ready = ...
        cond1 && ...
        cond2 && ...
        cond3 && ...
        cond4 && ...
        cond5;
    pause(0.2)
end

disp('Initial positions confirmed')


% ---------------------------------------------------------------------------
% 9. PRE-FLIGHT TEST      [SAFETY - Item 5 & 6]
% Fly the drone BY HAND to check OptiTrack reading and controller output.
% The drone does NOT take off here and NOTHING is sent to the robots.

TEST_MODE = 0;                 % 1 = run test and STOP ; 0 = skip test and fly
if TEST_MODE
    disp('TEST MODE: move the drone BY HAND to check OptiTrack + controller.')
    xdes = [0; 0; 1];
    kx=1.2; ky=1.2; kz=1.5;
    for it = 1:300             % ~10 s at 30 Hz
        PBt = poseB.LatestMessage.Pose;
        xbt = PBt.Position.X; ybt = PBt.Position.Y; zbt = PBt.Position.Z;
        et  = [xdes(1)-xbt; xdes(2)-ybt; xdes(3)-zbt];
        vt  = [kx*et(1); ky*et(2); kz*et(3); 0];
        uBt = f1\([0;0;0;0] + f2*vt);           % control signal (NOT sent)
        fprintf('pos=[% .2f % .2f % .2f]  err=[% .2f % .2f % .2f]  uB=[% .2f % .2f % .2f]\n',...
                xbt,ybt,zbt, et(1),et(2),et(3), uBt(1),uBt(2),uBt(3));
        pause(1/30);
    end
    disp('End of test. If readings/signals are OK, call a LAB-AIR member to')
    disp('validate (Item 6), then set TEST_MODE = 0 and run again to fly.')
    rosshutdown; return
end


% ---------------------------------------------------------------------------
% 10. TAKEOFF

if USE_DRONE

%     takeoffResponse = call(pubTake, msgTake, 'Timeout', 5); % Crazy Fly
    send(pubTake,msgTake)   % BEBOP
    pause(10)
end


% ---------------------------------------------------------------------------
% 11. DATA STORAGE

traj=[];   % append (robust to an early stop; preallocation optional)

% [SAFETY - Item 2,3,4] state used by the abort/land routines
lastStampB=[]; tLostB=0;
lastStampL=[]; tLostL=0;
aborted=false;


% ---------------------------------------------------------------------------
% 12. CONTROL LOOP (30 Hz)

rate = rosrate(30); reset(rate);   % enforce the real 30 Hz sample rate

try   % [SAFETY - Item 2] protect the whole control loop
for k=1:length(time)
t=time(k);

% ------------------
% 12.1. READ LIMO

if USE_LIMO
PL=poseL.LatestMessage.Pose;
x1=PL.Position.X;
y1=PL.Position.Y;
quat=[...
PL.Orientation.W ...
PL.Orientation.X ...
PL.Orientation.Y ...
PL.Orientation.Z];
eul=quat2eul(quat);
psi=eul(1);
else
x1=0; y1=0; psi=0;
end

% LIMO actual velocities (low-pass filtered) for the inner-loop feedback (12.7).
if k==1, x1p=x1; y1p=y1; psip=psi; v_meas=0; w_meas=0; end
vx1=(x1-x1p)/T;  vy1=(y1-y1p)/T;
v_raw = cos(psi)*vx1 + sin(psi)*vy1;
w_raw = atan2(sin(psi-psip),cos(psi-psip))/T;
lam = 0.3;
v_meas = (1-lam)*v_meas + lam*v_raw;
w_meas = (1-lam)*w_meas + lam*w_raw;
x1p=x1; y1p=y1; psip=psi;

% ------------------
% 12.2. CONTROL POINT

xf=x1+a*cos(psi);
yf=y1+a*sin(psi);

% ------------------
% 12.3. READ DRONE

if USE_DRONE
PB=poseB.LatestMessage.Pose;
xb=PB.Position.X;
yb=PB.Position.Y;
zb=PB.Position.Z;
% Drone yaw, used to rotate the global velocity into the body frame (12.8).
quatB=[PB.Orientation.W PB.Orientation.X PB.Orientation.Y PB.Orientation.Z];
eulB=quat2eul(quatB); psi_b=eulB(1);
else
xb=0; yb=0; zb=0; psi_b=0;
end

% [SAFETY - Item 4] Land if OptiTrack loses a body for more than 0.5 s.
% (If a body is lost, vrpn stops publishing and the message timestamp freezes.)

if USE_DRONE
sB=poseB.LatestMessage.Header.Stamp; stampB=double(sB.Sec)+double(sB.Nsec)*1e-9;
if isempty(lastStampB), lastStampB=stampB; end
if stampB==lastStampB, tLostB=tLostB+T; else, tLostB=0; lastStampB=stampB; end
end
if USE_LIMO
sL=poseL.LatestMessage.Header.Stamp; stampL=double(sL.Sec)+double(sL.Nsec)*1e-9;
if isempty(lastStampL), lastStampL=stampL; end
if stampL==lastStampL, tLostL=tLostL+T; else, tLostL=0; lastStampL=stampL; end
end
if tLostB>0.5 || tLostL>0.5
    disp('OptiTrack lost a body (>0.5 s) - aborting')
    abortAndLand(pubL,msgL,pubB,msgB,pubLand,msgLand,USE_LIMO,USE_DRONE);
    aborted=true; break
end

% [SAFETY - Item 3] Virtual wall: land if the drone leaves the safe box.
if USE_DRONE && (abs(xb)>2 || abs(yb)>2 || zb>1.8)
    disp('Virtual wall exceeded - aborting')
    abortAndLand(pubL,msgL,pubB,msgB,pubLand,msgLand,USE_LIMO,USE_DRONE);
    aborted=true; break
end


% ------------------
% 12.4. TRAJECTORY (LEMNISCATE)

if TRAJ_MODE   % 1 = full lemniscate
    xd  = 0.75*sin(2*pi*t/40);
    yd  = 0.75*sin(4*pi*t/40);
    dxd = 0.75*(2*pi/40)*cos(2*pi*t/40);
    dyd = 0.75*(4*pi/40)*cos(4*pi*t/40);
else           % 0 = go to a fixed point (pos_des)
    xd  = pos_des(1);
    yd  = pos_des(2);
    dxd = 0;
    dyd = 0;
end

% ------------------
% 12.5. FORMATION CONTROL

kx=1;
ky=1;
kz=1;

% Drone-only test: control the drone's OWN position error (not the LIMO point).
if ~USE_LIMO
    xf = xb; yf = yb;
end

% Position error
ex=xd-xf;
ey=yd-yf;

% Formation geometry (virtual-structure forward map, global frame).
xB_des = xf + rho_f*cos(beta_f)*cos(alpha_f);   % = xf
yB_des = yf + rho_f*cos(beta_f)*sin(alpha_f);   % = yf
zB_des = rho_f*sin(beta_f);                     % = 1.5
if ~TRAJ_MODE
    zB_des = pos_des(3);
end

% Altitude error
ez=zB_des-zb;

% Outer-loop law: feedforward + tanh. vx,vy = desired control-point velocity.
vx = dxd + Lx*tanh((kx/Lx)*ex);
vy = dyd + Ly*tanh((ky/Ly)*ey);
vz = kz*ez;


% ------------------
% 12.6. NULL SPACE OBSTACLE (Gaussian potential field, book eq. 5.20-5.22)

dx=xf-obs(1);
dy=yf-obs(2);
d=sqrt(dx^2+dy^2);

% Version without gaussian potential field
% if USE_OBSTACLE && d<influence
%     kavoid=2.0;
%     Jo=[dx/d, dy/d];                       % distance Jacobian (1x2)
%     Jp=Jo';                                % pseudo-inverse (||Jo||=1 -> Jp=Jo')
%     N=eye(2)-Jp*Jo;                        % null-space projector
%     sig=kavoid*(influence-d)/influence;    % 0 at boundary, grows inward
%     u=Jp*sig + N*[vx; vy];
%     vx=u(1);
%     vy=u(2);
% end

if USE_OBSTACLE && d<influence
    % Gaussian potential (eq. 5.20) and its gradient Jo (eq. 5.21c)
    V    = exp(-(dx/pot_a)^pot_n - (dy/pot_b)^pot_n);
    dVdx = -V*pot_n*(dx^(pot_n-1))/(pot_a^pot_n);
    dVdy = -V*pot_n*(dy^(pot_n-1))/(pot_b^pot_n);
    Jo   = [dVdx, dVdy];                    % 1x2 potential Jacobian
    if norm(Jo)>1e-4                        % guard against Jo -> 0 (d -> 0)
        Jp = pinv(Jo);                      % 2x1 pseudo-inverse
        N  = eye(2) - Jp*Jo;                % null-space projector
        % Primary task: avoid obstacle -> V -> Vd  (eq. 5.22, Vd_dot = 0)
        x_avoid = Jp*( 0 + kobs*(Vd - V) );
        % Secondary task: formation tracking, projected in the null space
        u  = x_avoid + N*[vx; vy];
        vx = u(1);
        vy = u(2);
    end
end

% ------------------
% 12.7. LIMO DYNAMIC

% (A) inverse kinematics of the control point (a=0.10, alpha=0)
vd = cos(psi)*vx + sin(psi)*vy;
wd = (-sin(psi)*vx + cos(psi)*vy)/a;
% (B) dynamic compensator with velocity feedback
if k==1, vd_prev=vd; wd_prev=wd; end
vdotd=(vd-vd_prev)/T; wdotd=(wd-wd_prev)/T;
vd_prev=vd; wd_prev=wd;
sv = vdotd + lu*tanh((ku/lu)*(vd-v_meas));
sw = wdotd + lw*tanh((kw/lw)*(wd-w_meas));
v = theta1*sv - theta3*wd*w_meas + theta4*vd;
w = theta2*sw + theta3*w_meas*vd + theta6*wd + theta5*v_meas*wd - theta3*wd*v_meas;


% ------------------
% 12.8. BEBOP DYNAMIC

% 
vxb_des = vx+Lx*tanh((kx/Lx)*(xB_des - xb));
vyb_des = vy+Ly*tanh((ky/Ly)*(yB_des - yb));

% Rotate the global vx,vy into the drone body frame (cmd_vel is body-frame).
vbx =  cos(psi_b)*vxb_des + sin(psi_b)*vyb_des;
vby = -sin(psi_b)*vxb_des + cos(psi_b)*vyb_des;
vb  = [vbx; vby; vz; 0];

% Rotate the global vx,vy into the drone body frame (cmd_vel is body-frame).
% vbx =  cos(psi_b)*vx + sin(psi_b)*vy;
% vby = -sin(psi_b)*vx + cos(psi_b)*vy;
% vb  = [vbx; vby; vz; 0];

if k==1
    vb_prev = vb;
    vb_f = vb; 
end
vb_f = 0.8*vb_f + 0.2*vb;

% Inner loop: inverse of v_dot = f1*u - f2*v  ->  u = f1\(v_dot + f2*v).
vdot=(vb_f-vb_prev)/T;
uB=f1\(vdot+f2*vb_f);

vb_prev=vb_f;


% ------------------
% 12.9. JOYSTICK CONTROL

Analog = axis(J);
Digital = button(J);
% BUTTON 1 - [SAFETY - Item 1] Emergency stop: zero cmd_vel and land the drone
if Digital(1)
    disp('Emergency Stop (joystick)')
    abortAndLand(pubL,msgL,pubB,msgB,pubLand,msgLand,USE_LIMO,USE_DRONE);
    aborted=true;
    break
end
% BUTTON 2
% Manual override
if Digital(2)
manual_mode = 1;
else
manual_mode = 0;
end
if manual_mode
% LIMO
v = Analog(2);
w = Analog(1);
% BEBOP
uB(1) = Analog(2);
uB(2) = Analog(1);
uB(3) = Analog(3);
uB(4) = 0;
end


% ------------------
% 12.10. COMMAND SAFETY SATURATION

% Soft limiting using tanh
v=vmax*tanh(v/vmax);
w=wmax*tanh(w/wmax);
uB(3)=vz_max*tanh(uB(3)/vz_max);

% Keep direct saturation for yaw
uB(4)= max(min(uB(4),yaw_max),-yaw_max);

% Final hard safety limits
v= max(min(v,vmax),-vmax);
w= max(min(w,wmax),-wmax);
uB(3)= max(min(uB(3),vz_max),-vz_max);


% Also saturate the drone horizontal velocities (indoor safety).
uB(1) = max(min(uB(1), vmax), -vmax);
uB(2) = max(min(uB(2), vmax), -vmax);


% ------------------
% 12.11. SEND LIMO

msgL.Linear.X=v;
msgL.Linear.Y=0;
msgL.Linear.Z=0;
msgL.Angular.Z=w;
if USE_LIMO, send(pubL,msgL); end


% ------------------
% 12.12. SEND DRONE

msgB.Linear.X=uB(1);
msgB.Linear.Y=uB(2);
msgB.Linear.Z=uB(3);
msgB.Angular.Z=0;   % yaw held at 0 (use uB(4) to control yaw)
if USE_DRONE, send(pubB,msgB); end

uB
% disp('uB(1): ');
% disp(uB(1))
% disp('uB(2): ')
% disp(uB(2));
% disp('uB(3): ')
% dips(uB(3));

% ------------------
% 12.13. STORE

traj=[...
    traj;
    t ...
    xf ...
    yf ...
    xb ...
    yb ...
    zb ...
    xd ...
    yd ...
    xB_des ...
    yB_des ...
    psi ...
    obs(1) ...
    obs(2) ...
    d];
% Column map:
%   1=t 2=xf 3=yf 4=xb 5=yb 6=zb 7=xd 8=yd
%   9=xB_des 10=yB_des 11=psi 12=obs1 13=obs2 14=d

waitfor(rate);   % hold the 30 Hz loop rate
end

catch ME   % [SAFETY - Item 2] any error in the loop -> zero cmd_vel and land
    warning('Control loop error: %s', ME.message);
    abortAndLand(pubL,msgL,pubB,msgB,pubLand,msgLand,USE_LIMO,USE_DRONE);
    aborted=true;
end


% ----------------------------------------------------------------------------
% 13. SAFE LANDING AND POSITION

if ~aborted
    if USE_LIMO
    % Stop LIMO first
    msgL.Linear.X = 0;
    msgL.Linear.Y = 0;
    msgL.Linear.Z = 0;
    msgL.Angular.Z = 0;
    send(pubL,msgL);
    pause(1);
    end
    if USE_DRONE
    % Move drone sideways away from LIMO
    safe_offset = 0.60;
    msgB.Linear.X = 0;
    msgB.Linear.Y = safe_offset;
    msgB.Linear.Z = 0;
    msgB.Angular.Z = 0;
    send(pubB,msgB);
    disp('Moving Drone To Safe Landing Position')
    pause(2);
    % Stop drone motion
    msgB.Linear.X = 0;
    msgB.Linear.Y = 0;
    msgB.Linear.Z = 0;
    msgB.Angular.Z = 0;
    send(pubB,msgB);
    pause(1);
    % Land drone (service send)
    send(pubLand,msgLand);
    disp('Landing Drone')
    pause(5);
    end
else
    disp('Aborted: drone already landed by a safety routine.')
end
% Final stop commands
if USE_LIMO, send(pubL,msgL); end
if USE_DRONE, send(pubB,msgB); end
pause(1);
rosshutdown
disp('Experiment Completed')


% ----------------------------------------------------------------------------
% 14. PLOT

figure
% Desired trajectory
plot(...
traj(:,7),...
traj(:,8),...
'k--',...
'LineWidth',2)
hold on
% LIMO trajectory
plot(...
    traj(:,2),...
    traj(:,3),...
    'r',...
    'LineWidth',2)
% Drone trajectory
plot(...
    traj(:,4),...
    traj(:,5),...
    'b',...
    'LineWidth',2)
% Obstacle (columns 12,13)
plot(...
    traj(1,12),...
    traj(1,13),...
    'ko',...
    'MarkerSize',12,...
    'LineWidth',3)
xlabel('X (m)')
ylabel('Y (m)')
title('Formation Tracking and Obstacle Avoidance')
legend(...
    'Desired',...
    'LIMO',...
    'Bebop',...
    'Obstacle')
axis equal
grid on

% ------------------
% Altitude
figure
plot(...
    traj(:,1),...
    traj(:,6),...
    'LineWidth',2)
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Drone Altitude')
grid on

% ------------------
% Heading (column 11)
figure
plot(...
    traj(:,1),...
    traj(:,11),...
    'LineWidth',2)
xlabel('Time (s)')
ylabel('Heading (rad)')
title('LIMO Heading')
grid on

% ------------------
% Obstacle Clearance (column 14)
figure
plot(...
    traj(:,1),...
    traj(:,14),...
    'LineWidth',2)
hold on
yline(influence,'--');     % influence zone radius (0.5 m)
yline(obs_radius,'r--');   % obstacle radius (0.15 m)
xlabel('Time (s)')
ylabel('Distance (m)')
title('Distance To Obstacle')
grid on


%----------------------------------------------------------------------------
%15. FORMATION ERROR
%3D distance between the real drone (xb,yb,zb) and the target (xf,yf,1.5).

ef=sqrt(...
    (traj(:,2)-traj(:,4)).^2+...   % xf - xb
    (traj(:,3)-traj(:,5)).^2+...   % yf - yb
    (traj(:,6)-1.5).^2);           % zb - 1.5
figure
plot(...
    traj(:,1),...
    ef,...
    'LineWidth',2)
xlabel('Time (s)')
ylabel('Formation Error (m)')
title('Drone Formation Error')
grid on


%----------------------------------------------------------------------------
%16. ANIMATION

figure
for k=1:5:length(traj)
    clf
    plot(...
        traj(:,7),...
        traj(:,8),...
        'k--')
    hold on
    plot(...
        traj(1,12),...
        traj(1,13),...
        'ko',...
        'MarkerSize',12)
    % Obstacle radius and influence zone
    th=linspace(0,2*pi,50);
    plot(obs(1)+obs_radius*cos(th), obs(2)+obs_radius*sin(th),'k');
    plot(obs(1)+influence*cos(th),  obs(2)+influence*sin(th),'k:');
    plot(...
        traj(1:k,2),...
        traj(1:k,3),...
        'r',...
        'LineWidth',2)
    plot(...
        traj(1:k,4),...
        traj(1:k,5),...
        'b',...
        'LineWidth',2)
    plot(...
        traj(k,2),...
        traj(k,3),...
        'rs',...
        'MarkerSize',12)
    plot(...
        traj(k,4),...
        traj(k,5),...
        'bo',...
        'MarkerSize',12)
    legend(...
        'Desired',...
        'Obstacle',...
        'LIMO',...
        'Bebop')
    xlabel('X (m)')
    ylabel('Y (m)')
    title('Formation Animation')
    axis equal
    grid on
    pause(0.05)
end


% ---------------------------------------------------------------------------
% 17. END OF EXPERIMENT

clear J


% ---------------------------------------------------------------------------
% LOCAL FUNCTIONS

function abortAndLand(pubL,msgL,pubB,msgB,pubLand,msgLand,USE_LIMO,USE_DRONE)
% [SAFETY - Items 1,2,3,4] Zero cmd_vel of the robots in use ([0 0 0 0]^T)
% and land the drone (only when it is in use).
    msgL.Linear.X=0; msgL.Linear.Y=0; msgL.Linear.Z=0; msgL.Angular.Z=0;
    msgB.Linear.X=0; msgB.Linear.Y=0; msgB.Linear.Z=0; msgB.Angular.Z=0;
    for i=1:5                              % resend (UDP may drop a packet)
        if USE_LIMO,  send(pubL,msgL); end
        if USE_DRONE, send(pubB,msgB); end
        pause(0.02);
    end
    if USE_DRONE
        send(pubLand,msgLand);             % land the drone (service send)
    end
end
