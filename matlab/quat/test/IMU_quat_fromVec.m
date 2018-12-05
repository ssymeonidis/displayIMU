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

%% matlab reference
addpath('../..');
v1   = 50 * rand(1,3) - 25;
v2   = 50 * rand(1,3) - 25;
q    = quat('vec', v1, v2)

%% optimized C lib
loadlibrary('../IMU_quat.so', '../IMU_quat.c', 'alias', 'IMU_quat');
v1   = single(v1);
v2   = single(v2);
q    = single(zeros(1,4));
[~, ~, ~, q] = calllib('IMU_quat', 'IMU_quat_fromVec', v1, v2, q)
unloadlibrary IMU_quat;