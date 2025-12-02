function path_plot(rmin,rmax,zmin,zmax)
%% Geraçao de trajetoria
% Posicoes inicial e final
% p0 = [0.2;0.3;0.15];
% pf = [0.0;-0.45;0.25];
% Tf = 2; 
% [x,y,z] = trajectory_gen(p0,pf,Tf,rmin,rmax,zmin,zmax);

% Gerando os pontos para o cilindro de fora
[xx_outer, yy_outer, zz_outer] = cylinder(rmax, 50);
zz_outer = zz_outer * (zmax - zmin) + zmin;
% Gerando os pontos para o cilindro de dentro
[xx_inner, yy_inner, zz_inner] = cylinder(rmin, 50);
zz_inner = zz_inner * (zmax - zmin) + zmin;
%%

hold on;

% Cilindro de fora (transparente)
h1 = surf(xx_outer, yy_outer, zz_outer);
set(h1, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
    'FaceColor', 'blue','HandleVisibility','off');

% Cilindro de dentro (transparente)
h2 = surf(xx_inner, yy_inner, zz_inner);
set(h2, 'FaceAlpha', 0.2, 'EdgeColor', 'none',...
    'FaceColor', 'blue','HandleVisibility','off');

% Conectando as faces de cima e de baixo
theta = linspace(0, 2*pi, 50);
x_top_outer = rmax*cos(theta);
y_top_outer = rmax*sin(theta);
x_top_inner = rmin*cos(theta);
y_top_inner = rmin*sin(theta);

% Anel do topo
fill3([x_top_outer fliplr(x_top_inner)], [y_top_outer fliplr(y_top_inner)], ...
      zmax*ones(1,100), 'cyan', 'FaceAlpha', 0.2,...
      'EdgeColor', 'black','HandleVisibility','off');

% Anel debaixo
fill3([x_top_outer fliplr(x_top_inner)], [y_top_outer fliplr(y_top_inner)], ...
      zmin*ones(1,100), 'cyan', 'FaceAlpha', 0.2,...
      'EdgeColor', 'black','HandleVisibility','off');
hold on

% plot3(x, y, z, 'b-', 'LineWidth', 2,'DisplayName','End-Effector Trajectory');
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('Trajetoria 3D');
% grid on;
xlim([-rmax,rmax]);
ylim([-rmax,rmax]);
zlim([-.3,.55]);
% hold on;
% plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 8,...
%     'DisplayName', 'Start','LineWidth',2);
% plot3(x(end), y(end), z(end), 'ro', 'MarkerSize', 8,...
%     'DisplayName', 'End','LineWidth',2);
view(3);
% legend;



