% Generates the input trajectory(ies) for Belka's walking in NTRT.
% (c) Andrew Sabelhaus 2020

% prep the workspace
clear all;
close all;
clc;

%% (1) Specify the timepoints

% All inputs will use the same dt,
dt = 0.01;
% Specify the max time manually, for ease. 
% We'll apply a hold on the last value for each input until this max time.
t_max = 2;

% We'll specify pairs of [timepoint, value] that will be used to
% interpolate.
% This will be individually for EACH control input!

% Remember that we have to "double up" any of the flat sections, so that
% the curve interpolation will have zero slope. I.e., each value should
% have a start and an end.

% FOR THE ACBD GAIT:

% Leg A, degrees
u1 = [0, 0;
    0.5, 0;
    0.6, 20;
    0.8, 20];

% Leg B
u2 = [0, 0];

% Leg C
u3 = [0, 0];

% Leg D
u4 = [0, 0];

% Spine Horizontal Left/Right, as a percent retraction.
u5 = [0, 0;
    0.2, 0;
    0.4, -0.08];

% Spine Rotation CW/CCW, as a percent retraction.
u6 = [0, 0;
    0.2, 0;
    0.4, -0.065;
    1.0, -0.065;
    1.2, 0.055];

% To iterate nicer, later, put these all in a cell array
u_pts = {u1, u2, u3, u4, u5, u6};

% Add the "final timepoint" to each for the hold.
for i = 1:size(u_pts,2)
    % the value to hold is...
    hold_val = u_pts{i}(end, 2);
    % add in the last point
    u_pts{i}(end+1, :) = [t_max, hold_val];
end

%% (2) Generate the interpolated timepoints for each.

% Grid out the times
t = [0:dt:t_max]';

% Preallocate the result. One time index plus six inputs is seven columns
t_u = zeros(size(t,1), 7);
t_u(:,1) = t;

% For each of the actual inputs...
for i = 1:size(u_pts,2)
    % we're interested in...
    u_i = u_pts{i};
    % We'll work in "pairs" of points for interpolated segments, so iterate
    % over the specified points. That's rows of u_i up until the last row
    for j = 1:size(u_i,1)-1
        disp('i,j:');
        i
        j
        % Calculate the slope for this j-th segment for this i-th input.
        t0_ij = u_i(j,1);
        tf_ij = u_i(j+1, 1);
        v0_ij = u_i(j,2);
        vf_ij = u_i(j+1,2);
        % linear interpolation = finite difference
        m_ij = (vf_ij - v0_ij) / (tf_ij - t0_ij);
        % Then, insert points at each time along this segment, from start
        % until JUST BEFORE END, assume next segment will wrap up the end
        % point.
        % The slice of times will then be
        t_ij = [t0_ij : dt : (tf_ij-dt)]';
        % just a line. Easiest way is to do an offset to
        % the time indices, with intercept as previous value
        v_ij = m_ij * (t_ij - t0_ij) + v0_ij;
        disp('vij:');
        v_ij
        % Insert into the appropriate place in the trajectory. 
        % ASSUMING THE TIMEPOINTS ARE IN INTERVALS OF DT, the index that
        % should start this segment is
        idx0 = t0_ij/dt + 1;
        idxf = idx0 + size(v_ij,1)-1;
        % (since there's a first row of zeros.) Since first column is time,
        t_u(idx0 : idxf, i+1) = v_ij;
    end
end









    



    