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
q1   = quat('rand');
q2   = quat('rand');
out  = q1 * q2'

%% optimized matlab
addpath('../optimized');
out  = multConj(q1.val, q2.val)

%% optimized C lib
loadlibrary('../IMU_quat.so', '../IMU_quat.c', 'alias', 'IMU_quat');
q1   = single(q1.val);
q2   = single(q2.val);
out  = single(zeros(1,4));
[~, ~, ~, out] = calllib('IMU_quat', 'IMU_quat_multConj', q1, q2, out)
unloadlibrary IMU_quat;