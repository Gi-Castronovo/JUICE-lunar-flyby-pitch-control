function [tCA, rp] = closest_approach(P)
%CLOSEST_APPROACH Time from the epoch to the pericentre of the hyperbola, and its radius.

x = P.X0(1); y = P.X0(2); u = P.X0(3); v = P.X0(4);
r = hypot(x, y);
assert(x*u + y*v < 0, 'The spacecraft must be approaching the Moon at the epoch.');

a = 1/(2/r - (u^2 + v^2)/P.mu);            % semi-major axis (negative)
h = x*v - y*u;                             % specific angular momentum
e = sqrt(1 - h^2/(P.mu*a));                % eccentricity
F = acosh((1 - r/a)/e);                    % hyperbolic anomaly at the epoch
n = sqrt(P.mu/(-a)^3);

tCA = (e*sinh(F) - F)/n;                   % hyperbolic Kepler equation
rp  = a*(1 - e);
end
