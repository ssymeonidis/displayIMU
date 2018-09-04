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
% This function will convert a roation matrix to a quaternion
% based on http://www.euclideanspace.com/maths/geometry/rotations/
% conversions/matrixToQuaternion/
function a = matrixToEuler(M)

  a(3) = atan2(M(3,2),M(3,3));
  a(2) = atan2(-M(3,1),sqrt(M(3,2)*M(3,2)+M(3,3)*M(3,3)));
  a(1) = atan2(M(2,1),M(1,1));
