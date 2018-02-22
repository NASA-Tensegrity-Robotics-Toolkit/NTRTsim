% calculate YAML parameters for IROS 2018 paper
% Andrew P. Sabelhaus, Feb 2018

clear all;
close all;
clc;

% We need to do the following calculations for the YAML file,
% based on the physical parameters of the robot:

% (1) Density of rigid bodies (needs volume of each)
% (2) Cable parameters (pretension, etc)
% (3) Assembly positions (spine translations, etc

% I'm assuming that we've already done the math to create the robot's
% leg, foot, shoulder, hip models.

% Physical parameters of the models. Lengths in cm.
% YAML uses half-extents for the box dimensions! These are*full* extents.

% Legs:
leg_height = 17.85;
leg_width = 0.3; 
leg_length_Y = 2.5; % the left-right dimension, with respect to Laika
foot_radius = 1.25;

% Shoulders/hips:
% the base box
hip_length_X = 12.6;
hip_height = 6;
hip_length_Y = 7;
% the two "ends" of the "T"
hip_T_height = 6;
hip_T_width = 0.3;
hip_T_length_Y = 17.6;

% Vertebrae:
rod_r = 0.64; % cm. THis is roughly 1/4 inch.
% For the standard vertebrae, let's declare each node location,
% then get the total length vector.
% We'll name them top, bottom, right, left (tbrl).
v_t = [6.5, 0, 9.6];
v_b = [6.5, 0, -9.6];
v_r = [-6.5, 9.5, 0];
v_l = [-6.5, -9.5, 0];
% For the rotating vertebra: same thing.
v_rot_t = [5.5, 0, 9.2];
v_rot_b = [5.5, 0, -9.2];
v_rot_r = [-5.5, 9.2, 0];
v_rot_l = [-5.5, -9.2, 0];

%% (1) Density of rigid bodies

% For the front and rear (symmetric!), let's calculate the total volume.
% That's going to be, in cm^3, the volume of
% 2 x foot
% 2 x leg box
% 1 x hips base
% 1 x hips "T" top

leg_vol = (leg_height * leg_width * leg_length_Y) + ...
          ( (4/3) * pi * foot_radius^3);
      
rear_vol_cm = 2 * leg_vol + ...
           hip_length_X * hip_height * hip_length_Y + ...
           hip_T_height * hip_T_width * hip_T_length_Y;

% Convert volume in cm^3 to m^3: that's (1e-2)^3.
rear_vol_m = rear_vol_cm * 1e-6;
       
% Now, for density. We want the total force to be in N, for F = m*g.
% Here, however, we're using g' = 100 * g, so total mass (in kg) should be
% adjusted down by a factor of 100. (Our g = 981 in simulation.)
% E.g., we want our mass to be in deci-grams (kg / 100, not kg / 1000.)

% NEED TO DO: look at the dimensional analysis / scaling analysis again,
% I really don't think we're doing this correctly. Are we really supposed
% to scale mass also, like this? Or is density different? I don't think
% it's this easy as just saying "100" ...

% Measured mass of one full rear:
rear_mass_kg = 0.181; % was 181 grams
% So we want the mass to be
%read_mass_adj = rear_mass_kg * 100;

% ...actually, Drew thinks this should be different.
% Really need to look up that scaling stuff. Let's leave it for now,
% and just calculate the mass as if we were not scaling anything,
% using the m^3 volume above.

% From older analysis: seems we should do 1 / s^3 (where 's' is scaling
% factor.) So for example, this would be 1 / (100^3) = 1e-6, again.

rear_dens_si = rear_mass_kg / rear_vol_m;
rear_dens_adj = rear_dens_si * 1e-6;

% Let's do the vertebrae. The length of each rod is just the distance
% (vector norm).
% Just a quick calculation right now does show that all rods are the same
% length, within some small millimeter tolerance that we don't care about.
rod_l = norm(v_r);
rot_rod_l = norm(v_rot_r);

% So, the total volume, in cm^3, for each vertebra is four times the volume
% of one rod.
v_vol_cm = 4 * ( pi * rod_r^2 * rod_l);
v_rot_vol_cm = 4 * (pi * rod_r^2 * rot_rod_l);
% converted to m^3:
v_vol_m = v_vol_cm * 1e-6;
v_rot_vol_m = v_rot_vol_cm * 1e-6;

% one vertebra weighs:
v_mass_kg = 0.145;

% density is then:
v_dens_si = v_mass_kg / v_vol_m;
v_dens_adj = v_dens_si * 1e-6;
v_rot_dens_si = v_mass_kg / v_rot_vol_m;
v_rot_dens_adj = v_rot_dens_si * 1e-6;








