clear; close all;

% constants
T = 20;
dt = 0.05;
d2r = 180 / pi;

% system parameters
I_R = 0.15;
I_W = 3.6e-3;
W_0 = 60 * (2*pi / 60);
T_max = .5;
W_max = 2800;

% controller parameters
K_P = -3;
K_I = 0;
K_D = -0.05;

T_cov = 1e-2;

so = sim("controller.slx");
so.reference = resample(so.reference, so.tout);
err = so.speed - so.reference;

fprintf("System statistics\n");
fprintf("Inertia ratio: %3.2f\n", I_R / I_W);
fprintf("Max. wheel speed: %3.0f RPM\n", max(so.wheel.Data) * 60 / (2*pi));

low = abs(err.Data) < deg2rad(10);
low_i = find_consec(low, 2 / dt);
if (low_i ~= -1)
	t_1 = err.Time(low_i);
	fprintf("Time to 10 dps: %3.2f s\n", t_1);

	trunc = resample(err, (t_1 + 2):dt:T);
	fprintf("Steady-state RMSE: %3.2f dps\n", rms(trunc.Data) * d2r);
	fprintf("Steady-state bias: %3.2f dps\n", mean(trunc.Data) * d2r);
else
	trunc = err;
end

figure("name", "Actions");
overlay_ts("Torque [N*m]", ...
	struct("ts", so.process_noise, "leg", "Turbulence", "spec", "*-k"), ...
	struct("ts", so.control, "leg", "Controller output", "spec", ".-g"));

figure("name", "Error");
overlay_ts("Speed error [dps]", ...
	struct("ts", trunc * d2r, "leg", "Error", "spec", "-k"));

figure("name", "Speed");
overlay_ts("$\omega_z$ [dps]", ...
	struct("ts", so.reference * d2r, "leg", "Reference", "spec", "*-g"), ...
	struct("ts", so.speed * d2r, "leg", "Speed", "spec", ".-k"));

function idx = find_consec(lgl_v, k)
	% mvs = movsum(lgl_v, k)
	consec = movsum(lgl_v, length(k)) >= k;

	idx = find(movsum(lgl_v, k) >= k, 1, "first"); 
	if (any(~size(idx))) % empty vector
		idx = -1;
	end
end
