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

% initialize environment
clear all; close all;
addpath('..');
addpath('../utils');

% create test vector
f    = 256 * rand(1,3)
mag  = sqrt(sum(f.^2));

% create quaternion and verify up vector
q1   = quat("frwd",     f);
out1 = mag*q1.frwd
q2   = quat("frwdFast", f);
out2 = mag*q2.frwd

% display the state
deg1 = q1.deg
deg2 = q2.deg
figure(1); plotState(q1);
figure(2); plotState(q2);