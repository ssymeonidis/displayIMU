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

%%
% This function sets the quaternion to the vector specified by the
% Euler angles
function q = eulerToQuat(angles)
  cos_z                 = cos(angles(1)/2);
  cos_y                 = cos(angles(2)/2);
  cos_x                 = cos(angles(3)/2);
  sin_z                 = sin(angles(1)/2);
  sin_y                 = sin(angles(2)/2);
  sin_x                 = sin(angles(3)/2);
    
  q(1)                  = cos_z*cos_y*cos_x + sin_z*sin_y*sin_x;
  q(2)                  = cos_z*cos_y*sin_x - sin_z*sin_y*cos_x;
  q(3)                  = cos_z*sin_y*cos_x + sin_z*cos_y*sin_x;
  q(4)                  = sin_z*cos_y*cos_y - cos_z*sin_y*sin_x;
end
