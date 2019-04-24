% Create a big vector of whatever. We will resize to 24 rows, 5 col
example = 0.35*(1:(24*5));
% Reshape the vector into a matrix that looks like the same shape as we want
example = reshape(example, 24, 5);
% csvwrite takes two arguments
csvwrite('matlab_example_csv_output.csv', example);
diary off
