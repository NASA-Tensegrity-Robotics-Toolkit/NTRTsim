% Load the actual length/tension file
lenten = csvread('hex_pack_len_ten_prestress.csv');
totsz = size(lenten, 1);
norm_idx = 0.2;
lenten_i = interp1(linspace(0,1,totsz), lenten, norm_idx);

% Vector of lengths and tensions:
actuallengths_i = lenten_i(1:24);
tensions_i = lenten_i(25:end);