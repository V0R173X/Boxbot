function [x,y,z,t] = trajectory_gen(p0,pf,Tf,rmin,rmax,zmin,zmax)

t = linspace(0,Tf,20)';
% TRansformando p0 e pf para coordenadas cilindricas
r0 = norm(p0(1:2));
theta0 = atan2(p0(2), p0(1));
z0 = p0(3);
rf = norm(pf(1:2));
thetaf = atan2(pf(2), pf(1));
zf = pf(3);
%% Verifica se p0 e pf estao na Workspace
if r0 < rmax && r0 > rmin && z0 < zmax && z0 > zmin
    disp('p0 está dentro');
    p0_dentro = 1;
else
    disp('p0 está fora')
    p0_dentro = 0;
end

if rf < rmax && rf > rmin && zf < zmax && zf > zmin
    disp('pf está dentro');
    pf_dentro = 1;
else
    disp('pf está fora')
    pf_dentro = 0;
end

if p0_dentro && pf_dentro
    % Interpolando usando o polinomio quintico
    s = 10*(t/Tf).^3 - 15*(t/Tf).^4 + 6*(t/Tf).^5;
    r = r0 + (rf - r0) * s;
    theta = theta0 + (thetaf - theta0) * s;
    z = z0 + (zf - z0) * s;
    % De volta pras coordendadas retangulares
    x = r.*cos(theta);
    y = r.*sin(theta);
else
    return
end

end

