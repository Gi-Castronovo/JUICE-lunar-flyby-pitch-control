function P = mission_parameters()
%MISSION_PARAMETERS Data of the JUICE lunar flyby problem, in SI units.

rpm = 2*pi/60;                       % 1 rpm in rad/s
P.rpm = rpm;

% Moon
P.mu     = 4903e9;                   % gravitational parameter [m^3/s^2]
P.R_moon = 1737.4e3;                 % mean radius, only for the altitude printout [m]

% Spacecraft principal moments of inertia [kg m^2]
P.Ix = 36000;
P.Iy = 35000;                        % pitch axis, normal to the orbital plane
P.Iz = 37000;

% Reaction wheel on the pitch axis
P.Iw    = 0.11;                      % wheel inertia [kg m^2]
P.w0    = -5100*rpm;                 % initial wheel speed [rad/s]
P.wmax  = 5600*rpm;                  % maximum wheel speed [rad/s]
P.Twmax = 0.2;                       % maximum wheel torque [N m]
P.Km    = 0.118;                     % torque constant [N m/A]
P.Kw    = 0.003/rpm;                 % back-EMF constant, 0.003 V/rpm -> [V s/rad]
P.R     = 1.2;                       % winding resistance [Ohm]
P.Pmax  = 32;                        % maximum electrical power [W]

% Thrusters: each pair is a 2 x 10 N couple with 1.25 m arm
P.F      = 10;                       % [N]
P.b      = 1.25;                     % [m]
P.Mt     = 2*P.F*P.b;                % torque of one pair [N m]
P.tpulse = 0.01;                     % minimum pulse, also used as pulse quantum [s]

% Pointing requirement: 3 arcsec
P.tol = 3/3600*pi/180;               % [rad]

% Reference state w.r.t. the Moon at tCA - 30 min, and its 1-sigma uncertainty
P.X0    = [6734454.8; -2868009.5; -3805.775; 93.497];   % [m, m, m/s, m/s]
P.sigma = [0.2; 0.2; 1e-3; 1e-3];

% Initial attitude: theta is measured from the velocity vector to the body X-axis
P.theta0    = 68.33*pi/180;          % [rad]
P.thetadot0 = -0.0105*pi/180;        % [rad/s]
end
