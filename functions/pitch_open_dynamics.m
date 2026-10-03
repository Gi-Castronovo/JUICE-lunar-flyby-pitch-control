function ds = pitch_open_dynamics(~, s, Tm, Tt, P)
%PITCH_OPEN_DYNAMICS Orbit + pitch dynamics with prescribed torques.
%   Tm: motor torque applied to the wheel (the spacecraft receives -Tm)
%   Tt: thruster torque on the spacecraft

r = hypot(s(1), s(2));

ds = [s(3);
      s(4);
      -P.mu*s(1)/r^3;
      -P.mu*s(2)/r^3;
      s(6);
      (gravity_gradient(r, s(5), P) - Tm + Tt)/P.Iy;
      Tm/P.Iw;
      0];
end
