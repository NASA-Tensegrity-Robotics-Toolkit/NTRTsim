function theta_result = rod_hinge_dynamics(theta_t, theta_dot_t, p)

m = p.m;
x_r = p.x_r;
l = p.l;
g = p.g;
k = p.k;
c = p.c;
theta_r = p.theta_r;

theta_dot_tp1 = theta_dot_t;

% For the longitudinal sping and damper:

%theta_ddot_tp1 = (3 / (m*l^3)) * ( ((m*g*l)/2)*cos(theta_t) - c*l*theta_dot_t*(sin(theta_t)^2) + k*l^2*sin(theta_t)*cos(theta_t) - k*x_r*l*sin(theta_t));

% For the longitudinal spring, and rotational damper:
%theta_ddot_tp1 = (3 / (m*l^3)) * ( ((m*g*l)/2)*cos(theta_t) - c*theta_dot_t + k*l^2*sin(theta_t)*cos(theta_t) - k*x_r*l*sin(theta_t));

% For the rotational spring and rotational damper:
% Note that x_r is not used here, though it's passed in as a variable.
theta_ddot_tp1 = (3 / (m*l^2)) * ( ((m*g*l)/2)*cos(theta_t) - c*theta_dot_t - k*theta_t + k*theta_r);

theta_result = [theta_dot_tp1; theta_ddot_tp1];

end