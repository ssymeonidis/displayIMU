% This file is part of quaternion-based displayIMU C++/QT code base
% (https://github.com/ssymeonidis/displayIMU.git)
% Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
% Time (SMRT) Processing Solutions
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
% This function reports the state of the quaternion as Euler angles
% isHomogenous was in the original implementation; it is unclear if
% this is needed
function angles = quatToEuler(q, isHomogenous)
  if nargin < 2
    isHomogenous      = true;
  end
    
  w                     = q(1);
  x                     = q(2);
  y                     = q(3);
  z                     = q(4);
  sqw                   = w*w;
  sqx                   = x*x;
  sqy                   = y*y;
  sqz                   = z*z;
    
  if isHomogenous
    angles(1)           = atan2(2*(x*y+z*w), sqx-sqy-sqz+sqw);
    angles(2)           = asin(-2*(x*z-y*w));
    angles(3)           = atan2(2*(y*z+x*w), -sqx-sqy+sqz+sqw);
  else
    angles(1)           = atan2f(2*(z*y+x*w), 1-2*(sqx+sqy));
    angles(2)           = asinf(-2*(x*z-y*w));
    angles(2)           = atan2f(2*(x*y+z*w), 1-2*(sqy+sqz));
  end
end
