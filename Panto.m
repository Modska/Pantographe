% Robot dimensions
l_or_mg = 15; % Distance between origin and left motor
l_or_md = 120; % Distance between origin and right motor
l1 = 240; % Length from motor to passive joints
l2 = 220; % Length from passive joints to end effector

% Parameters for the circular trajectory
center_x = 0; % X-coordinate of the center of the circle
center_y = 300; % Y-coordinate of the center of the circle
radius = 100; % Radius of the circle
num_points = 50; % Number of points to define the circle

% Generate angles for the circular trajectory
angles = linspace(0, 2*pi, num_points);

% Calculate x and y coordinates for the circular trajectory
x_corners = center_x + radius * cos(angles);
y_corners = center_y + radius * sin(angles);

% Calculate angles for each corner of the square trajectory
for k = 1:length(x_corners)
    x = x_corners(k);
    y = y_corners(k);

    % Calculate angles
    beta1 = atan2(y, (l_or_mg));
    beta2 = atan2(y, l_or_md);

    alpha1_calc = (l1^2 + (l_or_mg^2 + y^2) - l2^2) / (2 * l1 * sqrt(l_or_mg^2 + y^2));
    alpha2_calc = (l1^2 + (l_or_md^2 + y^2) - l2^2) / (2 * l1 * sqrt(l_or_md^2 + y^2));

    if alpha1_calc > 1 || alpha2_calc > 1
        warning('Unreachable coordinates');
        continue;
    end

    alpha1 = acos(alpha1_calc);
    alpha2 = acos(alpha2_calc);

    shoulder1 = beta1 + alpha1;
    shoulder2 = -(pi - beta2 - alpha2);

    % Display the angles
    fprintf('Corner (%f, %f): Shoulder 1 = %f degrees, Shoulder 2 = %f degrees\n', x, y, rad2deg(shoulder1), rad2deg(shoulder2));
end
