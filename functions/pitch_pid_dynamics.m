function ds = pitch_pid_dynamics(~, s, K, P)
%PITCH_PID_DYNAMICS Orbit + linearised pitch dynamics with PID control on the wheel.
%   s = [x; y; u; v; theta; thetadot; w_wheel; integral of the offset]

r  = sqrt(s(1)^2 + s(2)^2);
Tc = pid_torque(s, K, P);                    % torque on the spacecraft
e  = pointing_offset(s, P);

ds = [s(3);
      s(4);
      -P.mu*s(1)/r^3;
      -P.mu*s(2)/r^3;
      s(6);
      (gravity_gradient(r, s(5), P) + Tc)/P.Iy;
      -Tc/P.Iw;                              % reaction on the wheel
      e];
end
