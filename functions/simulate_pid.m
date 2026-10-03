function [t, S, Tc, i, Pw] = simulate_pid(K, P, tspan, s0, opts)
%SIMULATE_PID Closed-loop simulation; histories as row vectors / 8xN matrix.

[t, S] = ode113(@(t, s) pitch_pid_dynamics(t, s, K, P), tspan, s0, opts);
t = t.';
S = S.';
Tc = pid_torque(S, K, P);
[i, Pw] = wheel_electrical(-Tc, S(7,:), P);  % the motor torque on the wheel is -Tc
end
