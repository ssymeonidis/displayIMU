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
q    = quat('rand');
out  = q.rght

%% optimized matlab
addpath('../optimized');
out  = toRght(q.val)

%% optimized C lib
loadlibrary('../IMU_quat.so', '../IMU_quat.c', 'alias', 'IMU_quat');
q    = single(q.val);
out  = single(zeros(1,3));
[~, ~, out] = calllib('IMU_quat', 'IMU_quat_toRght', q, out)
unloadlibrary IMU_quat;