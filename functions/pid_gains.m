function K = pid_gains(kp, P)
%PID_GAINS kd at 1.5 times the critical value of the PD pair, ki at 40% of the
%   Routh-Hurwitz limit of Iy*s^3 + kd*s^2 + kp*s + ki (ki < kp*kd/Iy).

K.kp = kp;
K.kd = 1.5*2*sqrt(kp*P.Iy);
K.ki = 0.4*K.kp*K.kd/P.Iy;
end
