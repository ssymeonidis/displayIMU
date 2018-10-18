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

function out = quatToEuler(q)

% roll (x-axis rotation)
sinr_cosp = 2.0 * (q(1)*q(2) + q(3)*q(4));
cosr_cosp = 1.0 - 2.0 * (q(2)*q(2) + q(3)*q(3));
out(3)    = atan2(sinr_cosp, cosr_cosp);

% pitch (y-axis rotation)
sinp      = 2.0 * (q(1)*q(3) - q(4)*q(2));
if (abs(sinp) >= 1)
  out(2)  = sign(sinp) * pi / 2;
else
  out(2)  = asin(sinp);
end

% yaw (z-axis rotation)
siny_cosp = 2.0 * (q(1)*q(4) + q(2)*q(3));
cosy_cosp = 1.0 - 2.0 * (q(3)*q(3) + q(4)*q(4));
out(1)    = atan2(siny_cosp, cosy_cosp);

end
