close all;
% Calculate plots etc.
simtime = so.tout;

% fprintf("Inertia ratio: %3.1f\n", ratio);

deg_f = 180 / pi;

%% SECTION: Sanity checks

%%{ 
% Position
figure("name", "Position");
overlay_ts(["East [m]", "North [m]", "Up [m]"], ...
	struct("ts", openrocket.x, "leg", "OpenRocket", "spec", ".-k"), ...
	struct("ts", so.checks.pos, "leg", "Simulink", "spec", "-g"));

figure("name", "Acceleration check");
overlay_ts(["Acceleration east [m/s\^2]", ...
	"Acceleration north [m/s\^2]", "Acceleration up [m/s\^2]"], ...
	struct("ts", calcd_accel, "leg", "OpenRocket (calc.)", "spec", ".-k"), ...
	struct("ts", so.checks.accel, "leg", "Simulink", "spec", "-g"));

figure("name", "Angular rate check");
overlay_ts(["$\omega_x$ [rad/s]", "$\omega_y$ [rad/s]"], ...
	struct("ts", ts_dim(openrocket.omega, 1:2), "leg", "OpenRocket", "spec", ".-k"), ...
	struct("ts", ts_dim(so.checks.omega, 1:2), "leg", "Simulink", "spec", "-g"));

%}


%{
% Orientation
direct = so.direction.Data;
norm_byrow = @(dat) sqrt(sum(dat .^ 2, 2));
pitch_yaw_dat = [atan2d(direct(:,2), direct(:,1)), ...
	acosd(direct(:,3) ./ norm_byrow(direct))];
pitch_yaw = timeseries(pitch_yaw_dat, so.direction.Time);
figure("name", "Orientation");
%}

%{
% Angular rate
figure("name", "Angular rate");
%}


%% SECTION: Measurements

meas_legend = ["Ideal", "Noisy"];
%{
% Gyroscope
figure("name", "Gyroscope reading")
%}

%{
% Accelerometer
figure("name", "Accelerometer reading")
%}

%%{
% Magnetometer
figure("name", "Magnetometer reading")
overlay_ts(["$B_x$ [gauss]", "$B_y$ [gauss]", "$B_z$ [gauss]"], ...
	struct("ts", so.sensors.mag_ideal, "leg", "Ideal", "spec", "*-k"), ...
	struct("ts", so.sensors.mag, "leg", "Noisy", "spec", "-g"))
%}

% SECTION: Filters

%%{
% z
ref = ts_dim(so.sensors.gyro_ideal, 3) * deg_f;
figure("name", "Filters");
overlay_ts("$\omega_z$ [dps]", ...
	struct("ts", ts_dim(so.sensors.gyro, 3) * deg_f - ref, "leg", "Measured", "spec", ".r"), ...
	struct("ts", so.filters.basic * deg_f - ref, "leg", "Filtered", "spec", "-g"), ...
	struct("ts", ts_dim(so.filters.ekf_omega, 3) * deg_f - ref, "leg", "EKF", "spec", "-m"));
%}
%
%
figure("name", "Orientation quaternions");
overlay_ts(["$q_w$", "$q_i$", "$q_j$", "$q_k$"], ...
	struct("ts", ts_align(so.checks.q), "leg", "Simulated", "spec", "*-k"), ...
	struct("ts", so.filters.ekf_gyro_only, "leg", "EKF (gyro only)", "spec", "-m"));

%% Helper functions


function aoa = angle_of_attack(v1, v2)
	% this recovers small angles very well
	% aoa = atan2d(norm(cross(v1, v2)), dot(v1, v2));	
	% this is easier to understand
	aoa = acosd(dot(v1,v2) / norm(v1) / norm(v2));
end

function ts = clean(ts, threshold)
	ts.Data(abs(ts.Data) < threshold) = 0;
end

function tall = correct_dims(ts)
	dat = squeeze(ts.Data);
	s = size(dat);
	% whatever dim with the same length as time gets put to the front
	timedim = find(s == length(ts.Time)); 

	indices = 1:length(s); % permute takes dimension indices
	indices(timedim) = []; % delete the relocated dimension
	dat = permute(dat, [timedim, indices]); % correct dim first
	ts.Data = dat; % re-assign data to index-corrected data
	tall = ts;
end
