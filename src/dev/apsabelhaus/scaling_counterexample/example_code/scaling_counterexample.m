% Scaling counterexample
% Drew Sabelhaus
% 7-13-14

% This basic simulation is designed to show, by counterexample,
% why use of nondimensional numbers is necessary for NTRT.
% Equivalently, this would show why the commutative property of
% multiplication does not apply to scaling of units.

clear all; clc; close all;

% This disproves the following posulate:
% 10 * (length / time ^2) == (10 * length) / time^2

% A spring-mass-damper oriented vertically.
m = 2; % kg;
y0 = 1; % meters;
%y_rest = 0.50; % meters;
y_rest = 1;
g = 9.81; % m/s^2
k = 150; % N/m
c = 10;  % N*s/m, or kg/sec

% Newton's third law for this 1-DOF system is
% m * y_ddot + c * y_dot + k * (y - y_rest) = -mg

% So, in state space, we're looking at a linear system (technically an
% affine system) which is the following, where x = [y; y_dot]
% x_dot = A * x + C;

A = [ 0, 1; ...
    -k/m, -c/m];

C = [ 0; -g + k * y_rest/m];

% Forward-integrate (Euler style) for a handful of seconds
dt = 0.01; % sec
t_final = 4; % sec
t = [0:dt:t_final];
num_timesteps = size(t,2);
y = zeros(2, num_timesteps); % because we have 2 state variables plus the affine offset. Want column vectors for each timestep.

% initialize. Particle starts at y0, with zero velocity.
y(:,1) = [ y0; 0]; 

for i=2:num_timesteps
    % simulate. note that only the Ax terms get euler integrated, we need
    % the affine term to stay == 1.
    y(:, i) = ( A * y(:, i-1) + C) * dt + y(:,i-1);
end

hold on;
plot(t, y(1,:));
plot(t, y(2,:), 'r');
title('Original, unscaled simulation');
xlabel('time, sec');
ylabel('Vertical height of particle, meters');

% Now, for comparison, scale gravity. This should be different!
g_scaled = 10 * g;
% A does not change, but C does:
C = [ 0; -g_scaled + k * y_rest/m];
% We can use the same timestepping, but need to collect new data points:
y_new = zeros(2, num_timesteps);
y_new(:,1) = [ y0; 0];

for i=2:num_timesteps
    % simulate. note that only the Ax terms get euler integrated, we need
    % the affine term to stay == 1.
    y_new(:, i) = ( A * y_new(:, i-1) + C) * dt + y_new(:,i-1);
end

figure;
hold on;
plot(t, y_new(1,:));
plot(t, y_new(2,:), 'r');
title('With only gravity scaled');
xlabel('time, sec');
ylabel('Vertical height of particle, meters');

% Here's the fun part: do a third simulation, and scale length,
% as we'd normally do in NTRT. Our length constants are y0 and y_rest.
length_scaling_factor = g_scaled/g;
y0_new = length_scaling_factor * y0;
y_rest_new = length_scaling_factor * y_rest;

% Recalculate C, again, and initialize.
C = [ 0; -g_scaled + k * y_rest_new/m];
y_new_length_scaled = zeros(2, num_timesteps);
y_new_length_scaled(:,1) = [y0_new; 0];

for i=2:num_timesteps
    % simulate. note that only the Ax terms get euler integrated, we need
    % the affine term to stay == 1.
    y_new_length_scaled(:, i) = ( A * y_new_length_scaled(:, i-1) + C) * dt + y_new_length_scaled(:,i-1);
end

% Now, re-adjust the length scale as we'd do in NTRT.
y_new_length_scaled = y_new_length_scaled ./ length_scaling_factor;

figure;
hold on;
plot(t, y_new_length_scaled(1,:));
plot(t, y_new_length_scaled(2,:), 'r');
title('With both gravity and initial length constants scaled');
xlabel('time, sec');
ylabel('Vertical height of particle, meters');

% These plots are the same, and that makes sense by laws of superposition.
% Since scaling g is equivalent to scaling a step input in the system, this
% can be thought of as an increase in the magnitude of the input, and we
% expect a proportional increase in the magnitude of the output via linear
% systems laws.

% Now, let's do something explicitly nonlinear. 



