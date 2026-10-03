function D = desaturation_manoeuvre(t0, x0, P, opts, E_trig, T_pulse, w_end, T_slew)
%DESATURATION_MANOEUVRE Wheel spin-up to the opposite limit with thruster pulses.
%
%   Phase A  The PID is switched off and the motor drives the wheel at maximum
%            torque from the negative limit towards +w_end. The reaction torque
%            makes the offset e = theta_goal - theta grow with constant
%            acceleration. Every time e reaches E_trig, the positive thruster
%            pair fires for T_pulse and pushes the offset back.
%   Phase B  At +w_end the wheel torque is removed and a two-pulse slew brings
%            offset and rate back to zero, so that the PID can take over again.
%            These pulses are multiples of the minimum pulse P.tpulse.
%
%   Histories are returned as row vectors (t, Tm, Tt) and an 8xN matrix (S).

t = t0;
x = x0(:);
D = struct('t', [], 'S', [], 'Tm', [], 'Tt', [], 'pulses', zeros(0, 3));

% ---------------- Phase A: free spin-up, thruster pulses at the trigger
Tm  = P.Twmax;
evA = odeset(opts, 'Events', @(tt, s) events_spinup(tt, s, P, E_trig, w_end));
while x(7) < w_end
    [tt, S, ~, ~, ie] = ode113(@(tt, s) pitch_open_dynamics(tt, s, Tm, 0, P), ...
                               [t, t + 5000], x, evA);
    D = append_segment(D, tt, S, Tm, 0);
    t = tt(end);  x = S(end, :).';
    if any(ie == 2)
        break                              % wheel at the target speed
    end
    assert(any(ie == 1), 'Phase A: no event detected.');
    [D, t, x] = fire(D, t, x, +1, T_pulse, Tm, P, opts);
end
D.t_spinup_end = t;

% ---------------- Phase B: two-pulse slew back to nadir (wheel torque off)
[e, edot] = pointing_offset(x, P);
dthetadot = edot + e/T_slew;               % rate change giving edot = -e/T_slew
[D, t, x] = fire(D, t, x, sign(dthetadot), quantised(dthetadot, P), 0, P, opts);

evB = odeset(opts, 'Events', @(tt, s) event_zero_offset(tt, s, P));
[tt, S] = ode113(@(tt, s) pitch_open_dynamics(tt, s, 0, 0, P), [t, t + 10*T_slew], x, evB);
D = append_segment(D, tt, S, 0, 0);
t = tt(end);  x = S(end, :).';

[~, edot] = pointing_offset(x, P);         % stop the slew: edot -> 0
[D, t, x] = fire(D, t, x, sign(edot), quantised(edot, P), 0, P, opts);

D.t_end = t;
D.x_end = x;
end

% -------------------------------------------------------------------------
function dur = quantised(dthetadot, P)
% Pulse duration for a pitch-rate change, in multiples of the minimum pulse.
dur = max(P.tpulse, round(abs(dthetadot)*P.Iy/P.Mt/P.tpulse)*P.tpulse);
end

function [D, t, x] = fire(D, t, x, pair, dur, Tm, P, opts)
% Pulse of the positive (pair = +1) or negative (pair = -1) thruster pair.
Tt = pair*P.Mt;
D.pulses(end + 1, :) = [t, dur, pair];
[tt, S] = ode113(@(tt, s) pitch_open_dynamics(tt, s, Tm, Tt, P), linspace(t, t + dur, 11), x, opts);
D = append_segment(D, tt, S, Tm, Tt);
t = tt(end);  x = S(end, :).';
end

function D = append_segment(D, tt, S, Tm, Tt)
n    = numel(tt);
D.t  = [D.t,  tt.'];
D.S  = [D.S,  S.'];
D.Tm = [D.Tm, Tm*ones(1, n)];
D.Tt = [D.Tt, Tt*ones(1, n)];
end

function [value, isterminal, direction] = events_spinup(~, s, P, E_trig, w_end)
e = pointing_offset(s, P);
value      = [e - E_trig; s(7) - w_end];
isterminal = [1; 1];
direction  = [1; 1];
end

function [value, isterminal, direction] = event_zero_offset(~, s, P)
value      = pointing_offset(s, P);
isterminal = 1;
direction  = 0;
end
