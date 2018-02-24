function timeseries = motor_data_ramp(sampling_time, total_time, max, filename)
%motor_data_example
%   This code generates an example of time series data that may
%   come from NTRT for motor input tracking. It outputs a time series
%   of linearly increasing datapoints, from time 0 to total_time,
%   of amplitude 0 to max (at total time.)
%
%   Arguments:
%       sampling_time = timestep for the tracking. This correlates to what
%           timestep we choose for NTRT, and is also then the dt for the
%           discrete time controller. Will be the first column of output.
%       total_time = end time for the data. Rounded down to the nearest
%           interval of sampling_time.
%       max = maximum value, at the final timestep.
%       filename = name of the .CSV file that will be output of the
%           example data. Should not contain the .csv extension. File is
%           placed in the current folder.

% Generate the timesteps, save as the first column of example data.
timeseries = [0:sampling_time:total_time]';
% Generate the ramp data. If we multiply each timepoint
% by the maximum amplitude, then re-scale by the max time, we
% get a ramp.
ramp = (max * timeseries) ./ total_time;
% assign it to the second column of our output
timeseries(:,2) = ramp;

% Output the CSV file. This will be what's provided by NTRT.
% Creat the full filename:
filename = strcat(filename, '.csv');
% Write the csv file.
disp(strcat('Writing csv file: ', filename));
csvwrite(filename, timeseries);

end

