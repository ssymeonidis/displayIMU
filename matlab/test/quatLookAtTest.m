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

% simple forward/up test
q       = quat("deg", 125, 5, 10)
u       = q.up;
f       = q.frwd;
q1_out  = quat("upFrwd", u, f)

% partial update test #1
q_init  = quat;
u       = q.up;
f       = q_init.frwd;
q_temp  = quat("upFrwd", u, f);
f       = q.frwd;
u       = q_temp.up;
q2_out  = quat("upFrwd", u, f)

% partial update test #2
u       = q.up;
q_temp  = quat("up", u);
f       = q.frwd;
u       = q_temp.up;
q3_out  = quat("upFrwd", u, f)