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
% This function will convert a quaternion to a rotation matrix 
% based on http://www.euclideanspace.com/maths/geometry/rotations/
% conversions/matrixToQuaternion/

function M = eulerToMatrix(angles) 

  ch = cos(angles(1));
  sh = sin(angles(1));
  ca = cos(angles(2));
  sa = sin(angles(2));
  cb = cos(angles(3));
  sb = sin(angles(3));

  M  = [ ca*ch,   -cb*sh + sb*sa*ch,    sb*sh + cb*sa*ch;
         ca*sh,    cb*ch + sb*sa*sh,   -sb*ch + cb*sa*sh;
        -sa,       sb*ca,               cb*ca]; 


