%% Nadir-pointing pitch control of JUICE during the lunar flyby (LEGA, August 2024)
% Task 1  Gravity-gradient torque on the pitch axis during the flyby
% Task 2  PID design on the reaction wheel: 3 arcsec from tCA-25 to tCA+25 min
% Task 3  Extension of the pointing window until the wheel saturates
% Task 4  Desaturation manoeuvre that maximises the interval to the next one
%
% Girolamo Castronovo - Space Missions and Systems, Sapienza University of Rome

clear; close all; clc;
addpath('functions');

SAVE_FIGURES = false;                         % true: export the plots to figures/

P    = mission_parameters();
opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);

[tCA, rp] = closest_approach(P);
t_on  = 5*60;                                 % pointing required from tCA - 25 min
t_end = tCA + 25*60;                          % ... until tCA + 25 min
t_grid = unique([0:1:floor(t_end), t_end]).';
s0 = [P.X0; P.theta0; P.thetadot0; P.w0; 0];
to_min = @(t) (t - tCA)/60;                   % time from closest approach [min]
arcsec = 180/pi*3600;

fprintf('Closest approach: %.2f s after the epoch, r = %.1f km (altitude %.1f km)\n\n', ...
        tCA, rp/1e3, (rp - P.R_moon)/1e3);

%% ======================= TASK 1: GRAVITY GRADIENT =======================
% Free attitude (no control); the nadir-pointing attitude is shown for comparison
[~, X1] = ode113(@(t, s) pitch_open_dynamics(t, s, 0, 0, P), t_grid, s0, opts);
X1 = X1.';
r1 = sqrt(X1(1,:).^2 + X1(2,:).^2);
[theta_g, thetadot_g] = nadir_reference(X1, P.mu);
GG_free  = gravity_gradient(r1, X1(5,:), P);
GG_nadir = gravity_gradient(r1, theta_g, P);

[GGf_max, kf] = max(abs(GG_free));
[GGn_max, kn] = max(abs(GG_nadir));
fprintf('TASK 1 - Gravity-gradient torque on the pitch axis\n');
fprintf('  Initial offset from nadir: %.1f arcsec (theta_goal = %.4f deg), rate offset %.2e deg/s\n', ...
        (theta_g(1) - P.theta0)*arcsec, theta_g(1)*180/pi, (thetadot_g(1) - P.thetadot0)*180/pi);
fprintf('  Free attitude : max |G| = %.3e N m at tCA %+.1f min\n', GGf_max, to_min(t_grid(kf)));
fprintf('  Nadir pointing: max |G| = %.3e N m at tCA %+.1f min, G(tCA) ~ 0\n', GGn_max, to_min(t_grid(kn)));

% Effect of the state uncertainty on the pointing reference (finite differences)
dtheta = zeros(4, numel(t_grid));
for k = 1:4
    sp = s0;  sp(k) = sp(k) + P.sigma(k);
    [~, Xp] = ode113(@(t, s) pitch_open_dynamics(t, s, 0, 0, P), t_grid, sp, opts);
    dtheta(k,:) = nadir_reference(Xp.', P.mu) - theta_g;
end
fprintf('  1-sigma of the nadir reference from the state uncertainty: max %.3f arcsec\n\n', ...
        max(sqrt(sum(dtheta.^2, 1)))*arcsec);

fig = figure('Color', 'w', 'Position', [100 100 950 420]);
subplot(1, 2, 1);
plot(to_min(t_grid), r1/1e3, 'b', 'LineWidth', 1.5); grid on
xlabel('Time from closest approach [min]'); ylabel('Distance from the Moon [km]');
title('Spacecraft-Moon distance');
subplot(1, 2, 2);
plot(to_min(t_grid), GG_free*1e4, 'Color', [0.85 0.4 0], 'LineWidth', 1.5); hold on
plot(to_min(t_grid), GG_nadir*1e4, 'b--', 'LineWidth', 1.5); grid on
xlabel('Time from closest approach [min]'); ylabel('Torque [10^{-4} N m]');
title('Gravity-gradient torque, pitch axis');
legend('Free attitude', 'Nadir pointing', 'Location', 'northwest');
save_figure(fig, 'gravity_gradient', SAVE_FIGURES);

%% ======================= TASK 2: PID DESIGN =============================
% kp starts from the value that balances the peak gravity gradient at the
% tolerance and grows in steps of 5 until every requirement is met
kp_min = ceil(GGf_max/P.tol);
[K, n_iter] = tune_gains(P, kp_min, 5, s0, t_grid, t_on, opts);

[t2, X2, Tc2, i2, P2] = simulate_pid(K, P, t_grid, s0, opts);
[e2, edot2] = pointing_offset(X2, P);
k_last = find(abs(e2) > P.tol, 1, 'last');
ca = abs(to_min(t2)) <= 10;
poles = roots([P.Iy, K.kd, K.kp, K.ki]);
[~, kdom] = max(abs(imag(poles)));

fprintf('TASK 2 - PID on the reaction wheel\n');
fprintf('  kp search from kp_min = %d (max gravity gradient / tolerance), %d iterations\n', kp_min, n_iter);
fprintf('  kp = %.1f N m/rad, kd = %.1f N m s/rad, ki = %.4f N m/(rad s)\n', K.kp, K.kd, K.ki);
fprintf('  Closed-loop poles: %s 1/s -> dominant pair damping %.2f, period %.0f s\n', ...
        mat2str(poles.', 3), -real(poles(kdom))/abs(poles(kdom)), 2*pi/abs(imag(poles(kdom))));
fprintf('  Offset within 3 arcsec from t = %.0f s (requirement: %.0f s)\n', t2(k_last + 1), t_on);
fprintf('  Max |offset| from tCA-25 to tCA+25 min: %.3f arcsec (tCA +/- 10 min: %.3f)\n', ...
        max(abs(e2(t2 >= t_on)))*arcsec, max(abs(e2(ca)))*arcsec);
fprintf('  Wheel speed  : %.0f to %.0f rpm   (limit +/-%.0f)\n', min(X2(7,:))/P.rpm, max(X2(7,:))/P.rpm, P.wmax/P.rpm);
fprintf('  Wheel torque : max %.4f N m      (limit %.1f)\n', max(abs(Tc2)), P.Twmax);
fprintf('  Current      : max %.3f A\n', max(abs(i2)));
fprintf('  Power        : %.2f to %.2f W   (limit %.0f)\n\n', min(P2), max(P2), P.Pmax);

z1 = [-30, -25];  z2 = [-10, 10];  tm = to_min(t2);
save_figure(plot_with_zooms(tm, e2*arcsec, 'Offset \theta_{goal} - \theta [arcsec]', 'Offset from nadir pointing', ...
            [-1 1]*P.tol*arcsec, z1, z2), 'offset', SAVE_FIGURES);
save_figure(plot_with_zooms(tm, edot2*arcsec, 'Rate offset [arcsec/s]', 'Rate offset from nadir pointing', ...
            [], z1, z2), 'rate_offset', SAVE_FIGURES);
save_figure(plot_with_zooms(tm, X2(7,:)/P.rpm, 'Wheel speed [rpm]', 'Reaction wheel speed', ...
            [-1 1]*P.wmax/P.rpm, z1, z2), 'wheel_speed', SAVE_FIGURES);
save_figure(plot_with_zooms(tm, Tc2, 'Torque [N m]', 'Wheel torque on the spacecraft', ...
            [-1 1]*P.Twmax, z1, z2), 'wheel_torque', SAVE_FIGURES);
save_figure(plot_with_zooms(tm, i2, 'Current [A]', 'Motor current', ...
            [], z1, z2), 'wheel_current', SAVE_FIGURES);
save_figure(plot_with_zooms(tm, P2, 'Power [W]', 'Wheel electrical power (negative: regeneration)', ...
            P.Pmax, z1, z2), 'wheel_power', SAVE_FIGURES);

%% ======================= TASK 3: WINDOW EXTENSION =======================
% Saturation is declared at 99% of the maximum wheel speed (1% margin)
w_sat = 0.99*P.wmax;
opts3 = odeset(opts, 'Events', @(t, s) saturation_event(t, s, w_sat));
[t3, X3, t_sat, x_sat] = ode113(@(t, s) pitch_pid_dynamics(t, s, K, P), [0, 4e4], s0, opts3);
t3 = t3.';  X3 = X3.';  x_sat = x_sat(1,:).';  t_sat = t_sat(1);
e3 = pointing_offset(X3, P);
GG3 = gravity_gradient(sqrt(X3(1,:).^2 + X3(2,:).^2), X3(5,:), P);

opts100 = odeset(opts, 'Events', @(t, s) saturation_event(t, s, P.wmax));
[~, ~, t_100] = ode113(@(t, s) pitch_pid_dynamics(t, s, K, P), [0, 4e4], s0, opts100);

fprintf('TASK 3 - Extension of the nadir-pointing window with the same control law\n');
fprintf('  Wheel at 99%% of its limit (%.0f rpm) at tCA + %.2f min (%.2f min after the epoch)\n', ...
        -w_sat/P.rpm, to_min(t_sat), t_sat/60);
fprintf('  -> the window can be extended by %.2f min beyond tCA + 25 min\n', (t_sat - t_end)/60);
fprintf('  (the wheel would reach -5600 rpm at tCA + %.2f min)\n', to_min(t_100(1)));
fprintf('  Max |offset| until saturation: %.3f arcsec\n', max(abs(e3(t3 >= t_on)))*arcsec);
fprintf('  Momentum budget [N m s]: spacecraft %+.3f, wheel %+.3f, gravity gradient %+.3f\n\n', ...
        P.Iy*(x_sat(6) - P.thetadot0), P.Iw*(x_sat(7) - P.w0), trapz(t3, GG3));

fig = figure('Color', 'w', 'Position', [100 100 800 400]);
plot(to_min(t3), X3(7,:)/P.rpm, 'b', 'LineWidth', 1.5); hold on
yline(-P.wmax/P.rpm, '--r'); yline(-w_sat/P.rpm, ':r'); xline(25, ':k');
grid on; xlabel('Time from closest approach [min]'); ylabel('Wheel speed [rpm]');
title('Wheel speed until saturation');
save_figure(fig, 'saturation', SAVE_FIGURES);

%% ======================= TASK 4: DESATURATION ===========================
% The wheel is driven to the opposite limit (99%) at maximum torque; the
% thrusters fire 1 s pulses when the offset reaches 0.99 deg
E_trig  = 0.99*pi/180;
T_pulse = 1;                                  % [s]
w_end   = 0.99*P.wmax;
T_slew  = 60;                                 % duration of the final slew [s]
D = desaturation_manoeuvre(t_sat, x_sat, P, opts, E_trig, T_pulse, w_end, T_slew);
[e4, edot4] = pointing_offset(D.S, P);
[~, P4] = wheel_electrical(D.Tm, D.S(7,:), P);

fprintf('TASK 4 - Desaturation to the opposite wheel limit\n');
fprintf('  Start tCA + %.2f min, wheel spin-up %.1f s, total duration %.1f s\n', ...
        to_min(t_sat), D.t_spinup_end - t_sat, D.t_end - t_sat);
fprintf('  Wheel %+.0f -> %+.0f rpm: %.1f N m s of momentum moved to the thrusters\n', ...
        x_sat(7)/P.rpm, D.x_end(7)/P.rpm, P.Iw*(D.x_end(7) - x_sat(7)));
fprintf('  Offset between %.3f and %.3f deg (limit 1 deg); max power %.2f W (limit %.0f)\n', ...
        min(e4)*180/pi, max(e4)*180/pi, max(P4), P.Pmax);
fprintf('  %d pulses, %.2f s of firing\n', size(D.pulses, 1), sum(D.pulses(:,2)));
fprintf('     t from tCA [min]   duration [s]   pair\n');
fprintf('     %12.2f   %12.2f   %+4d\n', [to_min(D.pulses(:,1)), D.pulses(:,2), D.pulses(:,3)].');

% Back to the PID on the wheel (integrator reset) for 24 h
xc = D.x_end;  xc(8) = 0;
opts4 = odeset(opts, 'Events', @(t, s) saturation_event(t, s, P.wmax));
[tc, Xc, t_sat2] = ode113(@(t, s) pitch_pid_dynamics(t, s, K, P), [D.t_end, D.t_end + 86400], xc, opts4);
tc = tc.';  Xc = Xc.';
ec = pointing_offset(Xc, P);
k_last = find(abs(ec) > P.tol, 1, 'last');
if isempty(k_last), t_back = 0; else, t_back = tc(min(k_last + 1, numel(tc))) - D.t_end; end
fprintf('  After the hand-over the offset is within 3 arcsec in %.0f s\n', t_back);
fprintf('  Wheel speed over the next 24 h: %.0f to %.0f rpm', min(Xc(7,:))/P.rpm, max(Xc(7,:))/P.rpm);
if isempty(t_sat2)
    fprintf(' -> no further desaturation needed\n');
else
    fprintf(' -> next saturation at tCA + %.1f h\n', (t_sat2(1) - tCA)/3600);
end

fig = figure('Color', 'w', 'Position', [100 100 1000 650]);
td = D.t - t_sat;
subplot(2, 2, 1);
plot(td, e4*180/pi, 'b', 'LineWidth', 1.5); hold on
yline(1, '--r'); yline(-1, '--r');
plot(D.pulses(:,1) - t_sat, 0*D.pulses(:,1), 'kv', 'MarkerFaceColor', 'k');
grid on; xlabel('Time from start of desaturation [s]'); ylabel('Offset \theta_{goal} - \theta [deg]');
title('Offset from nadir (triangles: thruster pulses)');
subplot(2, 2, 2);
plot(e4*180/pi, edot4*180/pi*1e3, 'b', 'LineWidth', 1.2); hold on
xline(1, '--r'); xline(-1, '--r'); grid on
xlabel('Offset [deg]'); ylabel('Rate offset [mdeg/s]'); title('Phase plane');
subplot(2, 2, 3);
plot(td, D.S(7,:)/P.rpm, 'b', 'LineWidth', 1.5); hold on
yline(P.wmax/P.rpm, '--r'); yline(-P.wmax/P.rpm, '--r'); grid on
xlabel('Time from start of desaturation [s]'); ylabel('Wheel speed [rpm]'); title('Wheel speed');
subplot(2, 2, 4);
plot(td, P4, 'b', 'LineWidth', 1.5); hold on
yline(P.Pmax, '--r'); grid on
xlabel('Time from start of desaturation [s]'); ylabel('Power [W]'); title('Wheel electrical power');
save_figure(fig, 'desaturation', SAVE_FIGURES);
