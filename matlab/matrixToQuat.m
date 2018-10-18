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
% This function will convert a roation matrix to a quaternion
% based on http://www.euclideanspace.com/maths/geometry/rotations/
% conversions/matrixToQuaternion/
function q = matrixToQuat(M)

tr = M(1,1) + M(2,2) + M(3,3);

if (tr > 0) 
  S    = sqrt(tr+1.0) * 2; 
  q(1) = 0.25 * S;
  q(2) = (M(3,2) - M(2,3)) / S;
  q(3) = (M(1,3) - M(3,1)) / S; 
  q(4) = (M(2,1) - M(1,2)) / S; 
elseif ((M(1,1) > M(2,2)) && (M(1,1) > M(2,2))) 
  S    = sqrt(1.0 + M(1,1) - M(2,2) - M(3,3)) * 2; 
  q(1) = (M(3,2) - M(2,3)) / S;
  q(2) = 0.25 * S;
  q(3) = (M(1,2) + M(2,1)) / S; 
  q(4) = (M(1,3) + M(3,1)) / S; 
elseif (M(2,2) > M(3,3))  
  S    = sqrt(1.0 + M(2,2) - M(1,1) - M(3,3)) * 2;
  q(1) = (M(1,3) - M(3,1)) / S;
  q(2) = (M(1,2) + M(2,1)) / S; 
  q(3) = 0.25 * S;
  q(4) = (M(2,3) + M(3,2)) / S; 
else 
  S    = sqrt(1.0 + M(3,3) - M(1,1) - M(2,2)) * 2;
  q(1) = (M(2,1) - M(1,2)) / S;
  q(2) = (M(1,3) + M(3,1)) / S;
  q(3) = (M(2,3) + M(3,2)) / S;
  q(4) = 0.25 * S;
end

end