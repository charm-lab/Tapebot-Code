% Define the variables
E = 2.1e11; % Elastic modulus of the tape material (in Pa)
R0 = 0.05; % Radius of curvature (in meters)
t = 0.001; % Tape thickness (in meters)
alpha = linspace(0, 60, 100); % Bending angle (in degrees)

% Convert bending angle to radians
alpha_rad = deg2rad(alpha);

% Calculate the second moment of area of the cross-section (assuming a rectangular tape)
% For a rectangular cross-section, I = (t * (R0 + t/2)^3) / 3
I = (t * (R0 + t/2)^3) / 3;

% Calculate the longitudinal beam curvature (kappa) using the formula: kappa = 1 / R0
kappa = 1 / R0;

% Calculate the bending moment (M) for each bending angle
M = E * I * kappa * ((alpha_rad.^3) / 12);

% Plot the bending moment vs. bending angle
figure;
plot(alpha, M,'Color', 'r', 'LineWidth', 2);
xlabel('Bending Angle (degrees)');
ylabel('Bending Moment (N*m)');
title('Bending Moment vs. Bending Angle');
grid on;




