clear; clc; close all;
% add utilities and models to local path
paths();

datapath = "assets/void_flat.mat";
simpath = "rocket_environment.slx";

%% Data readin
data_table = load(datapath).table;
openrocket = load_openrocket(data_table);
simtime = openrocket.Time;
sdt = mean(diff(simtime));
dt = 1 / 100;

%% Vehicle properties
M_T = openrocket.M;
I_T = openrocket.I;
I_W = [1.02e-3 1.02e-3 3.6e-3];
ratio = (I_T(3) - I_W(3)) / I_W(3);
W0 = 2;
T_max = 0.2;

%%  Sensor properties
% 0.03 * sqrt(SR)
gyro_rms = repmat(0.03 * sqrt(100), 1, 3) * 10;
gyro_rms = deg2rad(gyro_rms);
% random guess
gyro_drift_rms = repmat(0.1, 1, 3) * 10;
gyro_drift_rms = deg2rad(gyro_drift_rms);

% 3 miligee
accel_rms = repmat(3 * 9.81e-3, 1, 3) * 10;
% mgauss
B0 = [0 0 1];
mag_field_0 = [0; 0.25; 0];
mag_rms = [3.2e-3 3.2e-3 4.1e-3] * 10;

%% Model inputs
% define zeroes for sanity checking
zero_3d = timeseries(zeros(length(simtime), 3), simtime);
zero_1d = timeseries(zeros(size(simtime)), simtime);
% Calculate force from position series
calcd_velocity = smooth_dt(openrocket.x, 10, 2, sdt);
% calcd_velocity = smooth_dt(zero_3d, 10, 2, dt);
calcd_accel = smooth_dt(calcd_velocity, 10, 2, sdt);
force_in = calcd_accel * M_T;
% Calculate moment from angular rate series
angular_accel_in = smooth_dt(openrocket.omega, 3, 1, sdt);
moment_in = angular_accel_in * diag(I_T)';
random_torque_rms = 1;
% Initial state targets
x_0 = openrocket.x.Data(1,:);
v_0 = calcd_velocity.Data(1,:);
theta_0 = [openrocket.theta.Data(1,:), 0];
omega_0 = openrocket.omega.Data(1,:) + [0 0 W0];
% Wheel input
torque_in = pulse(simtime, 10, W0 * I_T(3) / T_max * 1, T_max);
% torque_in = zero_1d;

%% Filter models
% useful variables
alpha_cov = (random_torque_rms * dt / I_T(3)) ^ 2;
atan2_rms = mag_rms(1) / norm(mag_field_0);

% Basic filter
filters.basic.X0 = [omega_0(3)];

filters.basic.A = eye(1);
filters.basic.B = [-1 / I_T(3)] * dt;
filters.basic.C = eye(1);
filters.basic.D = zeros(1,1);

filters.basic.Q = diag([alpha_cov]);
filters.basic.R = diag([gyro_rms(3)^2]);

% EKF
filters.ekf.Q = eye(7) * 1e-2;
filters.ekf.Q0 = eye(7) * 1e-1;
init_quat = [-0.999  0 -0.436 0];
filters.ekf.X0 = [omega_0, init_quat]';
filters.ekf.RG = diag(gyro_rms .^ 2);
filters.ekf.RM = diag(mag_rms .^ 2);
%% Execute simulation

so = sim(simpath);

%% Helper functions

function ts = pulse(time, t0, dur, height)
	data = zeros(size(time));
	data(find(time > t0):find(time > t0 + dur)) = height;
	ts = timeseries(data, time);
end

function dts = smooth_dt(ts, varargin)
	dts_data = zeros(size(ts.Data));
	for (dim = 1:size(ts.Data, 2))
		dts_data(:,dim) = movingslope(ts.Data(:,dim), varargin{:});
	end
	dts = timeseries(dts_data, ts.Time);
end

function dts = raw_dt(ts, varargin) % varargin for interchangability with smooth_dt
	dat = diff(ts.Data) ./ diff(ts.Time);
	dts = timeseries(dat, ts.Time(1:length(dat)));
end

function openrocket = load_openrocket(table)
	%% rocket properties
	openrocket.Time = table.Time;
	openrocket.M = table.Mass(end);
	Ix = table.LongitudinalMomentOfInertia(end);
	Iz = table.RotationalMomentOfInertia(end);
	openrocket.I = [Ix Ix Iz]; % moments as vector for Multibody parameter

	%% translational motion
	% convention: x goes east, y goes north
	openrocket.x = timeseries([table.PositionEastOfLaunch, ...
		table.PositionNorthOfLaunch, table.Altitude], ...
		table.Time);

	%% rotational motion
	openrocket.aoa = timeseries(table.AngleOfAttack, table.Time);
	openrocket.theta = timeseries(...
		[table.LateralOrientation, ...
		90 - table.VerticalOrientation], ...
		table.Time);
	openrocket.omega = timeseries([table.YawRate, table.PitchRate, table.RollRate], table.Time);
end
