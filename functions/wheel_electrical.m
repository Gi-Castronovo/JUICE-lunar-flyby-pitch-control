function [i, Pw] = wheel_electrical(Tm, w, P)
%WHEEL_ELECTRICAL Motor current and electrical power of the wheel.
%   Tm is the torque the motor applies to the wheel, so that the back-EMF
%   term Kw*w*i is the mechanical power. Negative Pw means regeneration.

i  = Tm/P.Km;
Pw = (P.Kw*w + P.R*i).*i;
end
