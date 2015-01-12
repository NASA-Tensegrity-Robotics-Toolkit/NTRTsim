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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A spring-mass-damper attached horizontally to a rod, hinged at the
% origin.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p = {};
p.m = 1.5; % kg;
p.x_r = 1; % meters; NOTE THIS IS NOT USED FOR THE ANGULAR CASE.
p.l = 2; % meters;
p.g = 9.81; % m/s^2
p.k = 100; % N/m
p.c = 10;  % N*s/m, or kg/sec
p.theta_0 = pi/4; % radians
p.theta_r = pi/3; % radians. This is for the rotational spring model.


% Forward-integrate (Euler style) for a handful of seconds
dt = 0.0001; % sec
t_final = 10; % sec
t = [0:dt:t_final];
num_timesteps = size(t,2);
y = zeros(2, num_timesteps); % because we have 2 state variables.

% initialize. rod starts at angle theta_0, with zero velocity.
y(:,1) = [ p.theta_0; 0]; 

for i=2:num_timesteps
    % simulate.
    theta = y(1,i-1);
    theta_dot = y(2,i-1);
    y(:, i) = ( rod_hinge_dynamics(theta, theta_dot, p)) * dt + y(:,i-1);
end

hold on;
plot(t, y(1,:));
%plot(t, y(2,:), 'r');
axis([0 t_final 0.5 1.5]);
title('Original, unscaled simulation');
xlabel('time, sec');
ylabel('Radial position of rod, radians');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now, for comparison, scale gravity. This should show different results!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scaling_factor = 2;
g_scaled = scaling_factor * p.g;
p.g = g_scaled;
% We can use the same timestepping, but need to collect new data points:
y_new = zeros(2, num_timesteps);
y_new(:,1) = [ p.theta_0; 0];

for i=2:num_timesteps
    % simulate.
    theta = y_new(1,i-1);
    theta_dot = y_new(2,i-1);
    y_new(:, i) = ( rod_hinge_dynamics(theta, theta_dot, p)) * dt + y_new(:,i-1);
end

figure;
hold on;
plot(t, y_new(1,:));
%plot(t, y_new(2,:), 'r');
axis([0 t_final 0.5 1.5]);
title('With only gravity scaled');
xlabel('time, sec');
ylabel('Angular position of rod, radians');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Here's the fun part: do a third simulation, and scale length,
% as we'd normally do in NTRT. Our length constants are the length of the
% rod, and the rest length of the spring.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

l_new = (1/scaling_factor) * p.l;
%l_new = scaling_factor * p.l;
%l_new = sqrt(scaling_factor) * p.l;
%x_r_new = scaling_factor * p.x_r;
%c_new = sqrt(scaling_factor) * p.c;
%c_new = (1/scaling_factor) * p.c;
p.l = l_new;
%p.x_r = x_r_new;
%p.c = c_new;

%k_new = scaling_factor * p.k;
%m_new = (1/sqrt(scaling_factor)) * p.m;
%m_new = (1/scaling_factor) * p.m;
%p.k = k_new;
%p.m = m_new;

% initialize.
y_new_length_scaled = zeros(2, num_timesteps);
y_new_length_scaled(:,1) = [p.theta_0; 0];

for i=2:num_timesteps
    % simulate.
    theta = y_new_length_scaled(1,i-1);
    theta_dot = y_new_length_scaled(2,i-1);
    y_new_length_scaled(:, i) = ( rod_hinge_dynamics(theta, theta_dot, p)) * dt + y_new_length_scaled(:,i-1);
end


figure;
hold on;
plot(t, y_new_length_scaled(1,:));
%plot(t, y_new_length_scaled(2,:), 'r');

%Adjust the limits of the plot to match our new scaling
%t_final_new = floor(t_final * (1/sqrt(scaling_factor)));
%axis([0 t_final 0.5 1.5]);
title('Gravity and length scale changed');
xlabel('time, sec');
ylabel('Angular position of rod, radians');
axis([0 t_final 0.5 1.5]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Finally, scale according to our pi terms: both length and angular damping chang along with gravity.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%c_new = sqrt(scaling_factor) * p.c;
c_new = (1/scaling_factor) * p.c;
p.c = c_new;

%k_new = scaling_factor * p.k;
%m_new = (1/sqrt(scaling_factor)) * p.m;
%m_new = (1/scaling_factor) * p.m;
%p.k = k_new;
%p.m = m_new;

% initialize.
y_new_length_spring_scaled = zeros(2, num_timesteps);
y_new_length_spring_scaled(:,1) = [p.theta_0; 0];

for i=2:num_timesteps
    % simulate.
    theta = y_new_length_spring_scaled(1,i-1);
    theta_dot = y_new_length_spring_scaled(2,i-1);
    y_new_length_spring_scaled(:, i) = ( rod_hinge_dynamics(theta, theta_dot, p)) * dt + y_new_length_spring_scaled(:,i-1);
end


figure;
hold on;
plot(t, y_new_length_spring_scaled(1,:));
title('Gravity, length, and angular damping constant scale changed');
xlabel('time, sec');
ylabel('Angular position of rod, radians');
axis([0 t_final 0.5 1.5]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% But wait! We forgot to do the time term. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% initialize.
y_new_length_spring_time_scaled = zeros(2, num_timesteps);
y_new_length_spring_time_scaled(:,1) = [p.theta_0; 0];

for i=2:num_timesteps
    % simulate.
    theta = y_new_length_spring_time_scaled(1,i-1);
    theta_dot = y_new_length_spring_time_scaled(2,i-1);
    y_new_length_spring_time_scaled(:, i) = ( rod_hinge_dynamics(theta, theta_dot, p)) * dt + y_new_length_spring_time_scaled(:,i-1);
end

% Adjust time for the g, c, k angular scaling case
%t_new = t.* (1/scaling_factor);
t_new = t.* (scaling_factor);

% Adjust the time scale, from our pi term analysis:
%t = t * scaling_factor;

figure;
hold on;
plot(t_new, y_new_length_spring_time_scaled(1,:));
title('Gravity, length, angular damping constant, and time scale changed');
xlabel('time, sec');
ylabel('Angular position of rod, radians');
axis([0 t_final 0.5 1.5]);


