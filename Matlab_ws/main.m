robot_model

% Posicoes inicial e final
p0 = [0.0;0.2;0.15];
pf = [0.2;-0.1;0.4];
Tf = 2; 
% Trajetoria desejada do end-effector
[x,y,z,t] = trajectory_gen(p0,pf,Tf,rmin,rmax,zmin,zmax); 

% Trajetoria desejada no espaço das juntas
qdes = inverse_kinematics(x,y,z,L1,L2,L3);

Kp = diag([14.71,822.95,91.439]);
Ki = diag([36.688,2051.68,227.964]);
Kd = diag([1.761,98.496,10.944]);

[qm,qmd,tau] = dynamics_simulation(qdes(1,:),[0,0,0],t,qdes,...
    RTT,Kp,Ki,Kd);
%%
% path_plot
RTT.plot(qm)
% qm é uma matriz [n x dof] com as juntas em cada instante
% Exemplo: cada linha de qm é uma configuração do manipulador

% filename = 'RTT_animacao.gif';
% 
% for k = 1:3:size(qm,1)
%     % Plota o robô na configuração k
%     RTT.plot(qm(k,:));
%     drawnow;
% 
%     % Captura o frame
%     frame = getframe(gcf);
%     img = frame2im(frame);
%     [A,map] = rgb2ind(img,256);
% 
%     % Escreve no GIF
%     if k == 1
%         imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",0.1);
%     else
%         imwrite(A,map,filename,"gif","WriteMode","append","DelayTime",0.1);
%     end
% end


%% Plot de qdes e qm, JUNTA 1
subplot(3,1,1)
plot(t,qm(:,1),'LineWidth',2,'Color','b')
hold on
plot(t,qdes(:,1),'LineStyle','--','LineWidth',2,'Color','r')
legend('q_{1,med}(t)','q_{1,des}(t)','FontSize',10)
title('Junta 1, rotacional')
xlabel('Tempo t (s)')
ylabel('q_1(t)'), grid on

% Plot de qdes e qm, JUNTA 2
subplot(3,1,2)
plot(t,qm(:,2),'LineWidth',2,'Color','b')
hold on
plot(t,qdes(:,2),'LineStyle','--','LineWidth',2,'Color','r')
legend('q_{2,med}(t)','q_{2,des}(t)','FontSize',10)
title('Junta 2, prismática')
xlabel('Tempo t (s)')
ylabel('q_2(t)'), grid on

% Plot de qdes e qm, JUNTA 3
subplot(3,1,3)
plot(t,qm(:,3),'LineWidth',2,'Color','b')
hold on
plot(t,qdes(:,3),'LineStyle','--','LineWidth',2,'Color','r')
legend('q_{3,med}(t)','q_{3,des}(t)','FontSize',10)
title('Junta 3, prismática')
xlabel('Tempo t (s)')
ylabel('q_3(t)'), grid on
%% Sinais de erro(t)
figure(3)
plot(t,qdes-qm,'LineWidth',2)
legend('e_1(t)','e_2(t)','e_3(t)')
title('Sinais de erro em cada junta')
xlabel('Tempo t(s)'), ylabel('e(t)'), grid on
ylim([-0.1,0.1])

%% Sinal do atuador
figure(4), clf
subplot(3,1,1)
plot(t,tau(:,1),'LineWidth',2)
legend('\tau_1','FontSize',12)
title('Atuador 1 rotacional'), grid on
xlabel('t(s)'), ylabel('Torque (Nm)')

subplot(3,1,2)
plot(t,tau(:,2),'LineWidth',2)
legend('f_2','FontSize',12)
title('Atuador 2 translacional'), grid on
xlabel('t(s)'), ylabel('Força (N)')

subplot(3,1,3)
plot(t,tau(:,3),'LineWidth',2)
legend('f_3','FontSize',12)
title('Atuador 3 translacional'), grid on
xlabel('t(s)'), ylabel('Força (N)')