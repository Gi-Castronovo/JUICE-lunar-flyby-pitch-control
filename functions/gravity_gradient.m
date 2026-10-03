function G = gravity_gradient(r, theta, P)
%GRAVITY_GRADIENT Gravity-gradient torque about the pitch axis (roll angle phi = 0).

G = 1.5*P.mu./r.^3*(P.Iz - P.Ix).*sin(2*theta);
end
