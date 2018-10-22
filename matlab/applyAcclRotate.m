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

% This function is based off the work performed by Sebastian O.H. Madgwick,
% documented in the paper "An efficient orientation Filter for inertial and
% inertial/magnetic sensor arrays. 

%%
% ASSUMPTION - normalized input quaternion (use quatNormalize function)

function [q, FOM] = applyAcclRotate(q, accl, alpha, method)

% normalize the acceleration vector
accl       = accl / sqrt(sum(accl.^2));

% check number of arguments
if (nargin < 4)
  method = "full";
end

if (method == "full")
  
  % normalize the acceleration vector
  mag        = sqrt(sum(accl.^2));
  if (mag > 0.001)
    accl     = accl / mag;
  end

  % find the rotational difference
  q_delta    = quatRotateDiff(accl, q);
  q          = qRotate(q, q_delta);
end

end