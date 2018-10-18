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
% inertial/magnetic sensor arrays. Changes were made to turn on/off sensors
% and to adapted the filter for human activity recoginition.  Significant
% changes were made for adding hooks and increasing readabilty.

function [q, FOM] = applyAcclGradient(q, accl, alpha)

% normalize the acceleration vector
accl       = accl / sqrt(sum(accl.^2));

% compute the objective function 
f_1        = 2*q(2)*q(4) - 2*q(1)*q(3) - accl(1);
f_2        = 2*q(1)*q(2) + 2*q(3)*q(4) - accl(2);
f_3        = 1 - 2*q(2)*q(2) - 2*q(3)*q(3) - accl(3);
 
% compute the Jacobian
J_11or24   = 2*q(3);
J_12or23   = 2*q(4);
J_13or22   = 2*q(1);
J_14or21   = 2*q(2);
J_32       = 2.0*J_14or21;
J_33       = 2.0*J_11or24;

% calculate the gradient
qHatDot    = [J_14or21*f_2 - J_11or24*f_1,              ...
              J_12or23*f_1 + J_13or22*f_2 - J_32*f_3,   ...
              J_12or23*f_2 - J_33*f_3 - J_13or22*f_1,   ...
              J_14or21*f_1 + J_11or24 * f_2];
mag        = sqrt(sum(qHatDot.^2));
if (mag > 1)
  qHatDot  = qHatDot / mag;
end
q          = q - alpha*qHatDot;
q          = q / sqrt(sum(q.^2));
FOM        = qHatDot(1);

end