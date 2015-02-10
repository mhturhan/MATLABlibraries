% Code to move the IMU world-relative angular velocities to the robot coordinates
clc
clear all
close all

% Rotation order: yaw-pitch-roll
syms IMUx IMUy IMUz dYaw dPitch dRoll yaw pitch roll real

% Rotation matricies
Rx = [1 0          0;
      0 cos(roll) -sin(roll);
      0 sin(roll)  cos(roll)];

Ry = [cos(pitch) 0 sin(pitch);
      0          1 0;
     -sin(pitch) 0 cos(pitch)];

Rz = [cos(yaw) -sin(yaw) 0;
      sin(yaw)  cos(yaw) 0;
      0         0        1];

% Move from the world frame to the robot frame
% The world has zero angular velocity
z_f0 = [0;
        0;
        0];

% Spatial velocity of yaw "link"
z_f1 = z_f0 + [0; 0; dYaw];
% Body velocity of yaw "link"
z_f1 = Rz.'*z_f1;

% Spatial velocity of pitch "link"
z_f2 = z_f1 + [0; dPitch; 0];
% Body velocity of pitch "link"
z_f2 = Ry.'*z_f2;

% Spatial velocity of roll "link"
z_f3 = z_f2 + [dRoll; 0; 0];
% Body velocity of roll "link"
z_f3 = Rx.'*z_f3;

% Move the body velocity of the roll "link" to world coordinates
z_f4 = Rz*Ry*Rx*z_f3;

% Equate the IMU data to these velocities
solutions = solve(z_f4(1) == IMUx,...
                  z_f4(2) == IMUy,...
                  z_f4(3) == IMUz,...
                  [dYaw dPitch dRoll]);

% Simplify and print the equations
display(simplify(solutions.dYaw))
display(simplify(solutions.dPitch))
display(simplify(solutions.dRoll))
