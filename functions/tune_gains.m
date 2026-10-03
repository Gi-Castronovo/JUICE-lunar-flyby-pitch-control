function [K, n_iter] = tune_gains(P, kp_start, kp_step, s0, t_grid, t_on, opts)
%TUNE_GAINS Increases kp from kp_start until all the requirements are met:
%   |offset| <= 3 arcsec after t_on, wheel speed, torque and power within limits.

kp = kp_start;
n_iter = 0;
while true
    n_iter = n_iter + 1;
    K = pid_gains(kp, P);
    [t, S, Tc, ~, Pw] = simulate_pid(K, P, t_grid, s0, opts);
    e = pointing_offset(S, P);
    if max(abs(e(t >= t_on))) <= P.tol && max(abs(S(7,:))) <= P.wmax && ...
       max(abs(Tc)) < P.Twmax && max(Pw) <= P.Pmax
        return
    end
    assert(kp < 2000, 'No gain below kp = 2000 satisfies the requirements.');
    kp = kp + kp_step;
end
end
