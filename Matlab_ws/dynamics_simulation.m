function [qm,qdm,input] = dynamics_simulation(q0,qd0,t,qdes,robot,Kp,Ki,Kd)
    N = length(t);
    dt = t(2) - t(1);  % passo do tempo uniforme
    s = 0; % integral de e(t)
    eprev = [0,0,0]; 

    qm = zeros(N, 3);
    qdm = zeros(N, 3);
    qdd = zeros(N, 3);
    input = zeros(N,3);

    qm(1,:) = q0;
    qdm(1,:) = qd0;
    
    % Kp = diag([14.71,822.95,91.439]);
    % Ki = diag([36.688,2051.68,227.964]);
    % Kd = diag([1.761,98.496,10.944]);


    for i = 1:N-1
        q = qm(i,:);
        qd = qdm(i,:);
        qdes_i = qdes(i,:);
        e = qdes_i-q;
        dedt = (e-eprev)/dt;
        s = s + (e+eprev)*dt/2;
        tau = Kp*e' + Ki*s' + Kd*dedt';
        input(i,:) = tau';
        % calculo das matrizes da dinamica do robo pra cada posiçao
        M = robot.inertia(q);
        C = robot.coriolis(q, qd);
        G = robot.gravload(q);
        % calcula a nova posiçao
        qdd_i = M \ (tau - C * qd' - G');
        % size(qdd_i)
        qdd_i = qdd_i';
        
        
        qdm(i+1,:) = (qd + qdd_i*dt);
        qm(i+1,:) = (q + qd * dt + 0.5 * qdd_i * dt^2)';

        qdd(i,:) = qdd_i';
        eprev = e;
    end
end


