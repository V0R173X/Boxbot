clear, clc

L1 = 0.106;
L2 = 0.096;
L3 = 0.10;

%% Criando o robo
% L1 = Link('d',0,'a',0,'alpha',0,'modified'); % Junta REV
% L2 = Link('theta',0,'a',length1,'alpha',-pi/2,'prismatic','modified'); % Junta TRANS
% L3 = Link('theta',pi/2,'a',length2,'alpha',0,'prismatic','modified'); % Junta TRANS

Link1 = Link('d',L1,'a',0,'alpha',0); % Junta REV
Link2 = Link('theta',0,'a',L2,'alpha',-pi/2,'prismatic'); % Junta TRANS
Link3 = Link('theta',pi/2,'a',L3,'alpha',0,'prismatic'); % Junta TRANS

d2min = .107; d2max = .410;
d3min = 0.15; d3max = .216;
Link2.qlim = [d2min,d2max];
Link3.qlim = [d3min,d3max];

% Parte envolvendo a dinamica
% 
Link1.m = 1.37; % massa (kg)
Link1.r = (1e-3)*[0 0 -54]; % centro de massa
Link1.I = (1e-9)*[9854229 9801603 8358081 316 103903 896];
% [Ixx Iyy Izz Ixy Iyz Ixz]
% Link1.Jm = 0; % motor inertia
% Link1.G = 1; % gear ratio
Link2.m = 1.536; % massa (kg)
Link2.r = (1e-3)*[-86 33 -38]; % centro de massa
Link2.I = (1e-9)*[34735480 17388537 44444545 -4936502 -2215531 5675815];
% [Ixx Iyy Izz Ixy Iyz Ixz]

Link3.m = 0.191; % massa (kg)
Link3.r = (1e-3)*[-75 0 -25]; % centro de massa
Link3.I = (1e-9)*[546534 1767755 1356849 -21225 -3359 479973];
% [Ixx Iyy Izz Ixy Iyz Ixz]
RTT = SerialLink([Link1,Link2,Link3],'name','RTT');

%% Obtendo as matrizes M,C e G
q = [pi/2 d2max d3max];
qd = [0.25 .007 .042];

M = RTT.inertia(q);          % M(q)
C = RTT.coriolis(q,qd);      % C(q,qd)
G = RTT.gravload(q);         % G(q)



%% Valores maximos 
% theta1max = 0.8*pi;
% d2max = L1;
% d3min = 0.15;
% d3max = 0.5;
%% constraints
rmin = sqrt(d3min^2 + L2^2);
rmax = sqrt(d3max^2 + L2^2);
zmin = L1+d2min-L3;
zmax = L1+d2max-L3;
%% Animacao  
% t = 0:0.05:2;
% timespan = length(t);
% q = [linspace(0,pi/2, timespan);
%      linspace(d2min,d2max, timespan);
%      linspace(d3min,d3max, timespan)]';
% figure(1), clf
% zlim([0 0.6]), hold on
% xlim([-0.3 0.3]), hold on
% ylim([-0.3 0.3]), hold on
% RTT.plot(q);  % Animate over time

 


% T = RTT.fkine([theta1, d2, d3]);  % Get forward kinematics
