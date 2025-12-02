function qdes = inverse_kinematics(x,y,z,L1,L2,L3)
    % x, y, z: column vectors or scalars
    % L1, L2, L3: link lengths
    % qdes: Nx3 matrix [q1, q2, q3]

    N = length(x);
    qdes = zeros(N,3);

    for i = 1:N
        r = sqrt(x(i)^2 + y(i)^2);
        
        q2 = z(i) + L3 - L1;
        q3 = sqrt(r^2 - L2^2);
        q1 = atan2(y(i), x(i))-atan2(q3,L2) + 2*pi;

        % % enforce joint limits if needed
        % q2 = max(0, q2);   % assuming prismatic joints can't retract
        % q3 = max(0, q3);   

        qdes(i,:) = [q1, q2, q3];
    end
end
