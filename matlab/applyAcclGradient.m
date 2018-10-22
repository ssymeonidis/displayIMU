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
% inertial/magnetic sensor arrays".

%%
% ASSUMPTION - normalized input quaternion (use quatNormalize function)

function [q, FOM] = applyAcclGradient(q, accl, alpha)

% normalize the acceleration vector
mag        = sqrt(sum(accl.^2));
if (mag > 0.001)
  accl     = accl / mag;
end

% compute the objective function 
two_q      = 2 * q;
f_1        = two_q(2)*q(4) - two_q(1)*q(3) - accl(1);
f_2        = two_q(1)*q(2) + two_q(3)*q(4) - accl(2);
f_3        = 1 - two_q(2)*q(2) - two_q(3)*q(3) - accl(3);

% compute the Jacobian
J_11       = two_q(3);
J_12       = two_q(4);
J_13       = two_q(1);
J_14       = two_q(2);
J_32       = 2*J_14;
J_33       = 2*J_11;

% calculate the gradient
qHatDot    = [J_14*f_2 - J_11*f_1,              ...
              J_12*f_1 + J_13*f_2 - J_32*f_3,   ...
              J_12*f_2 - J_33*f_3 - J_13*f_1,   ...
              J_14*f_1 + J_11 * f_2];

% apply the gradient to q
mag        = sqrt(sum(qHatDot.^2));
if (mag > 0.001)
  qHatDot  = qHatDot / mag;
end
q          = q - alpha*qHatDot;

% normalize q prior to return
q          = q / sqrt(sum(q.^2));
FOM        = qHatDot(1);

end