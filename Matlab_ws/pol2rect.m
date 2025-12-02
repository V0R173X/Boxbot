function p = pol2rect(v)
r = v(1);
theta = v(2);
zin = v(3);
p(1) = r*cosd(theta);
p(2) = r*sind(theta);
p(3) = zin;
end

