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
u    = 256 * rand(1,3)
fOrg = 256 * rand(1,3);
D1   = dot(fOrg, u);
D2   = dot(u, u);
f    = fOrg - (D1/D2)*u
uMag = sqrt(sum(u.^2));
fMag = sqrt(sum(f.^2));

% create quaternion and verify up vector
q    = quat("upFrwd", u, fOrg);
u    = uMag*q.up
f    = fMag*q.frwd

% display the state
deg  = q.deg
figure(1); plotState(q);