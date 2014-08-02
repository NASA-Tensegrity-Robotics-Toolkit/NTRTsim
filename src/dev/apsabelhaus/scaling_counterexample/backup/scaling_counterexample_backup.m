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

% A simple particle in a projectile motion
m = 2; % kg. Note, this doesn't affect the simulation, obv.
v0 = 10; % m/s
g = 9.81; % m/s^2
theta = pi/2; % rad

% For a projectile, newton's 3rd law is
% F_x = 0 (no forces in x)
% F_y = -mg (one force is gravity)

% Recalling position vs. time equation, which is
% x(t) = x0 + v0 * t + 0.5 * a * t^2
% Here, a_y = g, a_x = 0.

% For fun, let's simulate until the particle hits the ground again
% That will occur at tf = 2 * v0 * sin(theta) / g

tf = 2 * v0 * sin(theta) / g;

% Plot points along time from 0 to tf.
t = linspace(0, tf, 100);

y = v0 * sin(theta) .* t - 0.5 * g * t.^2;
x = v0 * sin(theta) .* t;

hold on;
plot(x,y);
title('Original, unscaled simulation');
xlabel('Horizontal length, meters');
ylabel('Vertical length, meters');

% Now, for comparison, scale gravity.
% Recalculate when the projectile hits the ground.
g_scaled = 10 * g;
tf_new = 2 * v0 * sin(theta) / g_scaled;
t_new = linspace(0, tf_new, 100);
y_new = v0 * sin(theta) .* t_new - 0.5 * g_scaled * t_new.^2;
x_new = v0 * sin(theta) .* t_new;

figure;
plot(x_new, y_new);
title('With only gravity scaled');
xlabel('Horizontal length, meters');
ylabel('Vertical length, meters');

% Here's the fun part: take that last simulation, and scale length,
% as we'd normally do in NTRT. 
length_scaling_factor = g_scaled/g;
y_new_scaled = length_scaling_factor * y_new;
x_new_scaled = length_scaling_factor * x_new;

figure;
plot(x_new_scaled, y_new_scaled);
title('With both gravity and length scaled, post-hoc');
xlabel('Horizontal length, units of meters * scaling const');
ylabel('Vertical length, units of meters * scaling const');

% HOWEVER! It should be clear here that there is an issue