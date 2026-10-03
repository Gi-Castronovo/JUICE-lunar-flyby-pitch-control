function [e, edot] = pointing_offset(S, P)
%POINTING_OFFSET Offset from nadir pointing, e = theta_goal - theta, and its rate.

[theta_g, thetadot_g] = nadir_reference(S, P.mu);
e    = theta_g - S(5,:);
edot = thetadot_g - S(6,:);
end
