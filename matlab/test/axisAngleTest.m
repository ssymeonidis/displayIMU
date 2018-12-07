% This file is part of quaternion-based displayIMU C++/QT code base
% (https://github.com/ssymeonidis/displayIMU.git)
% Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
% Time (SMRT) Processing Solutions)
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, version 2.
%
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
% General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.

% initialize simulation
clear all; close all;
addpath('..');

% use axisAngle to "divide" quaternion by 5
q_full        = quat("rand");
axisAngle     = q_full.axisAngle;
axisAngle(1)  = axisAngle(1) / 5;
q_part        = quat("axisAngle", axisAngle);

% rotate vector by original quaternion 
v             = rand(1,3);
v_full        = q_full / v

% rotate vector by "divide" quaternion 5 times
for i=1:5
  v           = q_part / v;
end
v_full        = v
