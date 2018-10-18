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

function q = applyGyro(q, gyro, dt)

tmp(1) = - 0.5*q(2)*gyro(1) - 0.5*q(3)*gyro(2) - 0.5*q(4)*gyro(3);
tmp(2) =   0.5*q(1)*gyro(1) + 0.5*q(3)*gyro(3) - 0.5*q(4)*gyro(2);
tmp(3) =   0.5*q(1)*gyro(2) - 0.5*q(2)*gyro(3) + 0.5*q(4)*gyro(1);
tmp(4) =   0.5*q(1)*gyro(3) + 0.5*q(2)*gyro(2) - 0.5*q(3)*gyro(1);
q      =   q + dt*tmp;

end

