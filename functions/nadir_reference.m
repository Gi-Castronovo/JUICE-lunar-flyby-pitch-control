function [theta_g, thetadot_g] = nadir_reference(S, mu)
%NADIR_REFERENCE Pitch angle (and rate) that points the body Z-axis at the Moon.
%   theta is measured from the velocity vector to the body X-axis, so nadir
%   pointing means theta_g = acos(r.v/(|r||v|)) - pi/2, i.e. minus the
%   flight-path angle. The rate is the exact time derivative of this
%   expression along the Keplerian motion. S has the states on the rows
%   (x, y, u, v first).

x = S(1,:); y = S(2,:); u = S(3,:); v = S(4,:);
r  = sqrt(x.^2 + y.^2);
V  = sqrt(u.^2 + v.^2);
ud = -mu*x./r.^3;                          % gravitational acceleration
vd = -mu*y./r.^3;

rv = x.*u + y.*v;                          % r . v
s  = rv./(r.*V);                           % cosine of the angle between r and v
sdot = (V.^2 + x.*ud + y.*vd)./(r.*V) - s.*(rv./r.^2 + (u.*ud + v.*vd)./V.^2);

theta_g    = acos(s) - pi/2;
thetadot_g = -sdot./sqrt(1 - s.^2);
end
