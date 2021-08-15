% Plots the input trajectory(ies) for Belka's walking in NTRT.
% (c) Andrew Sabelhaus 2021

% prep the workspace
clear all;
close all;
clc;

%% (1) Specify the timepoints

% All inputs will use the same dt,
dt = 0.01;
% Apply a short hold at the end of the gait, this makes it so we don't have
% to "double up" the final timepoint
holdtime = dt;

% We'll specify pairs of [timepoint, value] that will be used to
% interpolate.
% This will be individually for EACH control input!

% Remember that we have to "double up" any of the flat sections, so that
% the curve interpolation will have zero slope. I.e., each value should
% have a start and an end.

% FOR THE ACBD / BDAC GAIT:

% Leg A, degrees
% Step rightwards:
% u1 = [0, 0;
%     0.5, 0;
%     0.6, 20;
%     2.8, 20;
%     2.9, 0];
% Step leftwards: switches with u2
u1 = [0, 0;
    1.8, 0;
    1.9, 20;
    2.8, 20;
    2.9, 0;
    4, 0];
% Combined:
% u1 = [0, 0;
%     0.5, 0;
%     0.6, 20;
%     2.8, 20;
%     2.9, 0;
%     3.0, 0;
%     4.8, 0;
%     4.9, 20;
%     5.8, 20;
%     5.9, 0];

% Leg B
% Step rightwards:
% u2 = [0, 0;
%     1.8, 0;
%     1.9, 20;
%     2.8, 20;
%     2.9, 0];
% Step leftwards: switches with u1. ADJUSTED
u2 = [0, 0;
    0.6, 0;
    0.7, 20;
    2.8, 20;
    2.9, 0;
    4, 0];
% Combined:
% u2 = [0, 0;
%     1.8, 0;
%     1.9, 20;
%     2.8, 20;
%     2.9, 0
%     3.0, 0;
%     3.6, 0;
%     3.7, 20;
%     5.8, 20;
%     5.9, 0];

% Leg C
% Step rightwards:
% u3 = [0, 0;
%     1.0, 0;
%     1.1, 20;
%     2.8, 20;
%     2.9, 0];
% Step leftwards: switches with u4
u3 = [0, 0;
    2.4, 0;
    2.5, 20;
    2.8, 20;
    2.9, 0;
    4, 0];
% Combined:
% u3 = [0, 0;
%     1.0, 0;
%     1.1, 20;
%     2.8, 20;
%     2.9, 0;
%     3.0, 0;
%     5.4, 0;
%     5.5, 20;
%     5.8, 20;
%     5.9, 0];

% Leg D
% Step rightwards:
% u4 = [0, 0;
%     2.4, 0;
%     2.5, 20;
%     2.8, 20;
%     2.9, 0];
% Step leftwards: switches with u3
u4 = [0, 0;
    1.0, 0;
    1.1, 20;
    2.8, 20;
    2.9, 0;
    4, 0];
% Combined:
% u4 = [0, 0;
%     2.4, 0;
%     2.5, 20;
%     2.8, 20;
%     2.9, 0;
%     3.0, 0;
%     4.0, 0;
%     4.1, 20;
%     5.8, 20;
%     5.9, 0];

% Spine Horizontal Left/Right, as a percent retraction.
% Step rightwards:
% u5 = [0, 0;
%     0.2, 0;
%     0.4, -0.08;
%     1.2, -0.08;
%     1.6, 0.18;
%     2.6, 0.18;
%     2.8, 0];
% Step leftwards: negative direction. ADJUSTED
u5 = [0, 0;
    0.2, 0;
    0.4, 0.09;
    1.2, 0.09;
    1.6, -0.18;
    2.6, -0.18;
    2.8, 0;
    4, 0];
% Combined:
% u5 = [0, 0;
%     0.2, 0;
%     0.4, -0.08;
%     1.2, -0.08;
%     1.6, 0.18;
%     2.6, 0.18;
%     2.8, 0;
%     3.0, 0;
%     3.2, 0;
%     3.4, 0.09;
%     4.2, 0.09;
%     4.6, -0.18;
%     5.6, -0.18;
%     5.8, 0];

% Spine Rotation CW/CCW, as a percent retraction.
% Step rightwards:
% u6 = [0, 0;
%     0.2, 0;
%     0.4, -0.065;
%     0.8, -0.065;
%     1.0, 0.055;
%     1.2, 0.055;
%     1.3, 0.040;
%     2.0, 0.040;
%     2.2, -0.065;
%     2.6, -0.065;
%     2.8, 0];
% Step leftwards: negative direction. ADJUSTED
u6 = [0, 0;
    0.2, 0;
    0.6, 0.045;
    0.8, 0.045;
    1.0, -0.075;
    1.2, -0.075;
    1.3, -0.065;
    2.0, -0.065;
    2.2, 0.045;
    2.6, 0.045;
    2.8, 0;
    4, 0];
% Combined:
% u6 = [0, 0;
%     0.2, 0;
%     0.4, -0.065;
%     0.8, -0.065;
%     1.0, 0.055;
%     1.2, 0.055;
%     1.3, 0.040;
%     2.0, 0.040;
%     2.2, -0.065;
%     2.6, -0.065;
%     2.8, 0;
%     3.0, 0;
%     3.2, 0;
%     3.6, 0.045;
%     3.8, 0.045;
%     4.0, -0.075;
%     4.2, -0.075;
%     4.3, -0.065;
%     5.0, -0.065;
%     5.2, 0.045;
%     5.6, 0.045;
%     5.8, 0];

% To iterate nicer, later, put these all in a cell array
u_pts = {u1, u2, u3, u4, u5, u6};

% Some adjustments to the gait, now:
% Startup delay
waittime = 0.1; % was 5
% Speedup factor
speedup = 1; % 1.5 is a nice number for the actual simulation
% speedup = 1;
% To make the gait symmetric: insert a mirrored left/right step after each
% of the original.
mirror_gait = 0;
% Let the robot settle a bit before switching to mirrored step
% mirror_delay = 0.7;
% Number of steps / repeats of the gait. If mirror_gait, total steps will
% end up being 2*nsteps.
nstep = 0; % 5 gives 30sec for the mirrored gait % was 11
% nstep = 0;

% First, apply the speedup: needed for calibrating the total time for one
% step of the gait
for i = 1:size(u_pts,2)
    % speedup
    u_pts{i}(:,1) = u_pts{i}(:,1) / speedup;
end

% For repeating the steps, find the maximum end time for the whole gait
allpts_beforeadjust = cell2mat(u_pts');
t_max_beforeadjust = max(allpts_beforeadjust(:,1));

% If selected, add a mirrored step to the end of this single step.
if mirror_gait == 1
    % preallocate
    u_pts_mirror = u_pts;
    % Legs: A is opposite B, and C is opposite D.
    u_pts_mirror{1} = u_pts{2};
    u_pts_mirror{2} = u_pts{1};
    u_pts_mirror{3} = u_pts{4};
    u_pts_mirror{4} = u_pts{3};
    % For horizonal left/right and rotation, these are defined vs. 0 as
    % neutral position, so make them negative to flip direction
    u_pts_mirror{5}(:,2) = -u_pts{5}(:,2);
    u_pts_mirror{6}(:,2) = -u_pts{6}(:,2);
    % we want mirrored step to begin after the end of the regular one plus
    % the delay
    for i=1:size(u_pts, 2)
        u_pts_mirror{i}(:,1) = u_pts_mirror{i}(:,1) + t_max_beforeadjust + mirror_delay;
        u_pts{i} = [u_pts{i}; u_pts_mirror{i}];
    end
    t_max_beforeadjust = t_max_beforeadjust*2 + mirror_delay;
end

% Apply the other two adjustments
for i = 1:size(u_pts,2)
    % Number of steps: pattern the gait and adjust by intervals of t_max.
    % Get the base pattern
    u_pts_i_onestep = u_pts{i};
    for n = 1 : nstep
        % Adjust the times only
        nextgait = [u_pts_i_onestep(:,1) + t_max_beforeadjust * n, u_pts_i_onestep(:,2)];
        % add on to the end
        u_pts{i} = [u_pts{i}; nextgait];
    end
    % startup delay: adjust all timepoints forward EXCEPT THE FIRST, since
    % we still want a zero at the beginning
    u_pts{i}(2:end,1) = u_pts{i}(2:end,1) + waittime;
end

% For preallocation, find the maximum end time for the whole gait AGAIN now
% after adjustments
allpts = cell2mat(u_pts');
t_max = max(allpts(:,1));

% Add the "final timepoint" to each for the hold.
for i = 1:size(u_pts,2)
    % the value to hold is...
    hold_val = u_pts{i}(end, 2);
    % add in the last point
    u_pts{i}(end+1, :) = [t_max + holdtime, hold_val];
end


%% (2) Generate the interpolated timepoints for each.

% Grid out the times
t = [0:dt:t_max+holdtime]';

% Preallocate the result. One time index plus six inputs is seven columnsSave
t_u = zeros(size(t,1), 7);
t_u(:,1) = t;

% For each of the actual inputs...
for i = 1:size(u_pts,2)
    % we're interested in...
    u_i = u_pts{i};
    % We'll work in "pairs" of points for interpolated segments, so iterate
    % over the specified points. That's rows of u_i up until the last row
    for j = 1:size(u_i,1)-1
        %disp('i,j:');
        %i
        %j
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
        %disp('vij:');
        %v_ij
        % Insert into the appropriate place in the trajectory. 
        % ASSUMING THE TIMEPOINTS ARE IN INTERVALS OF DT, the index that
        % should start this segment is
        idx0 = floor(t0_ij/dt + 1);
        idxf = floor(idx0 + size(v_ij,1)-1);
        % (since there's a first row of zeros.) Since first column is time,
        t_u(idx0 : idxf, i+1) = v_ij;
    end
    % We have an off-by-one error on the last line because we did the
    % indexing "up until" a timepoint not including that timepoint.
    t_u(end, 2:end) = t_u(end-1, 2:end);
end

%% (3) Plot results

% filename = 'belka_withmirror_20deg_2020-12-4.csv';
% filename = 'belka_acbd_longer_20deg_2020-12-31.csv';
% filename = 'belka_bdac_longer_20deg_2020-12-31.csv';
% disp('Saving Belka trajectory file...');

% manually specify 

figure;
hold on;
subplot(3,1,1);
hold on;
title('Walking Gait (BDAC) Input Sequence');

% Note: the u6 and u7 elements are percents +/- for each direction,
% mirrored, so we'll want to plot left/right and cw/ccw as mirrors of each
% other with this percent.

% "Horizontal Left Bend" is positive retraction in u5, right bend is
% negative,
% "CCW rotation" is positive u7.

limx = [0, 3.8];
lw = 2;
fontsize = 10;

% Spine bending left/right first
plot(t_u(:,1), 100*t_u(:,6), 'LineWidth', lw);
plot(t_u(:,1), -100*t_u(:,6), 'LineWidth', lw);
ylabel({'% Retraction';'(Horiz.)'});
xlim(limx);
legend('L', 'R');
set(gca,'FontSize',fontsize);

% Then rotation
subplot(3,1,2);
hold on;
plot(t_u(:,1), 100*t_u(:,7), 'LineWidth', lw);
plot(t_u(:,1), -100*t_u(:,7), 'LineWidth', lw);
% ylabel('% Retract (Rot.)');
ylabel({'% Retraction';'(Rotational)'});
xlim(limx);
legend('CW', 'CCW');
set(gca,'FontSize',fontsize);

% Then leg angles
subplot(3,1,3);
hold on;
% Note: bug in the above code somehow. Not going to fix, so just manually
% do this:
t_u(290, 3) = 20.0;

plot(t_u(:,1), t_u(:,2), 'LineWidth', lw);
plot(t_u(:,1), t_u(:,3), 'LineWidth', lw);
plot(t_u(:,1), t_u(:,4), 'LineWidth', lw);
plot(t_u(:,1), t_u(:,5), 'LineWidth', lw);
ylabel({'Leg Angle';'(\theta, deg.)'});
xlabel('Time (sec)');
xlim(limx);
legend('A', 'B', 'C', 'D', 'location', 'NE');
set(gca,'FontSize',fontsize);

% % Write the header
% hdr = {};
% hdr{1} = 'Belka Open-Loop Gait Timepoints Trajectory';
% hdr{2} = 'Usage: times and control inputs';
% hdr{3} = 'There are 6 inputs.';
% hdr{4} = 'Simulation time (sec),legA,legB,legC,legD,spineHoriz,spineRot,';
% fid = fopen(filename, 'w');
% for k=1:size(hdr,2)
%     fprintf(fid, '%s\n', string(hdr{k}));
% end
% fclose(fid);

% Write the data table
% Frustratingly enough, MATLAB doesn't have an option for appending an
% extra comma at the end, so we go row by row with fprintf... ugh
% for r = 1:size(t_u, 1)
%     row_r = t_u(r,:);
%     for c = 1:size(row_r,2)
%         % dlmwrite(filename, t_u, '-append');
% %         fprintf(fid, '%f,', row_r(c));
%     end
%     % append a newline
%     fprintf(fid, '\n');
% end
% fclose(fid);
% 
% disp('Done.');










    



    