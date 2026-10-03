function Tc = pid_torque(S, K, P)
%PID_TORQUE Control torque on the spacecraft, limited to the wheel capability.
%   S(8,:) is the integral of the pointing offset.

[e, edot] = pointing_offset(S, P);
Tc = K.kp*e + K.kd*edot + K.ki*S(8,:);
Tc = min(max(Tc, -P.Twmax), P.Twmax);
end
