function RobotSimUserInterface
    % Create figure (fullscreen normalized)
    fig = figure('Name','Simulation UI',...
                 'Units','normalized',...
                 'Position',[0.05 0.05 0.9 0.85]);

    %% initial position
    annotation(fig,'textbox',[0.83 0.715 0.1 0.05],...
        'String','$\mathbf{p_0}$: ','Interpreter','latex','FontSize',14,...
        'EdgeColor','none');
    p0Box = uicontrol("Style","edit","Units","normalized","Position",...
        [0.85 0.72 0.1 0.05],"String",'[ 0.0 ; 0.2 ; 0.15 ]','FontSize',12);

    %% final position
    annotation(fig,'textbox',[0.83 0.675 0.1 0.05],...
        'String','$\mathbf{p_f}$: ','Interpreter','latex','FontSize',14,...
        'EdgeColor','none');
    pfBox = uicontrol("Style","edit","Units","normalized","Position",...
        [0.85 0.68 0.1 0.05],"String",'[ 0.2 ; -0.1 ; 0.4 ]','FontSize',12);

    %% KP
    annotation(fig,'textbox',[0.57 0.76 0.1 0.05],...
        'String','$\mathbf{K_P}$: ','Interpreter','latex','FontSize',14,...
        'EdgeColor','none');
    kpBox = uicontrol('Style','edit','Units','normalized','Position',...
        [0.59 0.77 0.22 0.05],...
        'String','diag([ 14.71 , 822.95 , 91.439 ])','FontSize',12);

    %% KI
    annotation(fig,'textbox',[0.57 0.71 0.1 0.05],...
        'String','$\mathbf{K_I}$: ','Interpreter','latex','FontSize',14,...
        'EdgeColor','none');
    kiBox = uicontrol('Style','edit','Units','normalized','Position',...
        [0.59 0.72 0.22 0.05],...
        'String','diag([ 36.688 , 2051.68 , 227.964 ])','FontSize',12);

    %% KD
    annotation(fig,'textbox',[0.57 0.66 0.1 0.05],...
        'String','$\mathbf{K_D}$: ','Interpreter','latex','FontSize',14,...
        'EdgeColor','none');
    kdBox = uicontrol('Style','edit','Units','normalized','Position',...
        [0.59 0.67 0.22 0.05],...
        'String','diag([ 1.761 , 98.496 , 10.944 ])','FontSize',12);

    %% RUN SIMULATION BUTTON
    uicontrol('Style','pushbutton','String','RUN','Units','normalized',...
        'Position',[0.62 0.84 0.15 0.07],...
        'Callback',@(src,event)runSim(),'FontSize',16);

    %% BETWEEN XYZ OR R THETA
    bg = uibuttongroup('Parent',fig,'Title','Coordinate System',...
        'Units','normalized','Position',[0.8 0.84 0.15 0.10],'FontSize',12);

    r1 = uicontrol(bg,'Style','radiobutton','String','XYZ (End-effector)',...
        'Units','normalized','Position',[0.05 0.55 0.9 0.35],...
        'HandleVisibility','off','FontSize',12);
    r2 = uicontrol(bg,'Style','radiobutton','String','Joint Space       ',...
        'Units','normalized','Position',[0.05 0.15 0.9 0.35],...
        'HandleVisibility','off','FontSize',12);

    %% Axes for plotting results
    axq = axes('Parent',fig,'Units','normalized','Position',[0.57 0.49 0.42 0.15]);
    ylabel(axq,'$\mathbf{q}$','FontSize',18,'Interpreter','latex');
    title(axq,'Joint space','FontSize',12);
    grid(axq,'on');

    axqd = axes('Parent',fig,'Units','normalized','Position',[0.57 0.27 0.42 0.15]);
    ylabel(axqd,'$\mathbf{\dot{q}}$','FontSize',18,'Interpreter','latex');
    title(axqd,'Joint velocities','FontSize',12);
    grid(axqd,'on');

    axtau = axes('Parent',fig,'Units','normalized','Position',[0.57 0.06 0.42 0.15]);
    ylabel(axtau,'$\tau$','FontSize',26,'Interpreter','latex');
    title(axtau,'Control input u(t)','FontSize',12);
    grid(axtau,'on');

    %% Axes for plotting the ANIMATION
    ax = axes('Parent',fig,'Units','normalized','Position',[0.03 0.05 0.5 0.9],...
              'XLim',[-0.2364,0.2364],'YLim',[-0.2364,0.2364],'ZLim',[-.3,.55]);
    view(ax,3)
    title(ax,'Animation');
    grid(ax,'on');

    %% Callback
    function runSim()
        L1 = 0.106;
        L2 = 0.096;
        L3 = 0.10;

        %% Criando o robo
      
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

        Link2.m = 1.536; % massa (kg)
        Link2.r = (1e-3)*[-86 33 -38]; % centro de massa
        Link2.I = (1e-9)*[34735480 17388537 44444545 -4936502 -2215531 5675815];

        Link3.m = 0.191; % massa (kg)
        Link3.r = (1e-3)*[-75 0 -25]; % centro de massa
        Link3.I = (1e-9)*[546534 1767755 1356849 -21225 -3359 479973];

        RTT = SerialLink([Link1,Link2,Link3],'name','RTT');

        %% constraints
        rmin = sqrt(d3min^2 + L2^2);
        rmax = sqrt(d3max^2 + L2^2);
        zmin = L1+d2min-L3;
        zmax = L1+d2max-L3;

        % Get user inputs
        kp = eval(get(kpBox,'String'));
        ki = eval(get(kiBox,'String'));
        kd = eval(get(kdBox,'String'));
        p0 = eval(get(p0Box,'String'));
        pf = eval(get(pfBox,'String'));
        selected = bg.SelectedObject.String;  
        disp(['User selected: ' selected]);
        if bg.SelectedObject.String == 'XYZ (End-effector)'
            ...
        elseif bg.SelectedObject.String == 'Joint Space       '
            p0 = pol2rect(p0);
            p0
            pf = pol2rect(pf);
            pf
        end

        Tf = 2;
        % Trajetoria desejada do end-effector
        [x,y,z,t] = trajectory_gen(p0,pf,Tf,rmin,rmax,zmin,zmax);
        % path_plot
        % Trajetoria desejada no espaço das juntas
        qdes = inverse_kinematics(x,y,z,L1,L2,L3);
        [qm,qmd,tau] = dynamics_simulation(qdes(1,:),[0,0,0],t,qdes,RTT,...
            kp,ki,kd);
        

        


        % Plot result in the same UI window
        cla(ax); % clear previous plot
        plot3(x, y, z, 'b-', 'LineWidth',2);
        xlim([-2.25*rmax 2.25*rmax]), ylim([-2.25*rmax 2.25*rmax]),...
            zlim([-.3 .5])
        hold on
        RTT.plot(qm,'workspace',...
            [-2.25*rmax 2.25*rmax -2.25*rmax 2.25*rmax -0.3 0.5]);
        

        
        cla(axq);
        plot(axq,t,qm,'LineWidth',2);
        ylabel(axq,'$\mathbf{q}$','FontSize',18,'Interpreter','latex');
        title(axq,'Joint space','FontSize',15);
        grid(axq,'on');

        cla(axqd);
        plot(axqd,t,qmd,'LineWidth',2);
        ylabel(axqd,'$\mathbf{\dot{q}}$','FontSize',18,'Interpreter','latex');
        title(axqd,'Joint velocities','FontSize',15);
        grid(axqd,'on');

        cla(axtau);
        plot(axtau,t,tau,'LineWidth',2);
        ylabel(axtau,'$\mathbf{\tau}$','FontSize',24,'Interpreter','latex');
        title(axtau,'Control input u(t)','FontSize',15);
        grid(axtau,'on');

        % xlabel(ax,'Time (s)'); ylabel(ax,'Displacement (m)');
        % title(ax,sprintf('Response (m=%.2f, c=%.2f, k=%.2f)',m,c,k));
        % grid(ax,'on');
    end

end

