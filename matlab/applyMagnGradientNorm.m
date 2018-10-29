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

function [q, FOM] = applyMagnGradientNorm(q, magn, alpha)

% normalize the magnetometer vector
magn        = magn / sqrt(sum(magn.^2));
u           = quatToUp(q);
D           = dot(u, magn);
magn        = magn - D*u;
magn        = magn / sqrt(sum(magn.^2));

% compute the objective function
two_q       = 2 * q;
f_4         = 1 - two_q(3)*q(3) - two_q(4)*q(4) - magn(1);
f_5         = two_q(2)*q(3) - two_q(1)*q(4) - magn(2);
f_6         = two_q(1)*q(3) + two_q(2)*q(4) - magn(3);

% calculate the gradient
qHatDot     = [-two_q(4)*f_5 + two_q(3)*f_6,      ...
                two_q(3)*f_5 + two_q(4)*f_6,      ...
               -2*two_q(3)*f_4 + two_q(2)*f_5 + two_q(1)*f_6,      ...
               -2*two_q(4)*f_4 - two_q(1)*f_5 + two_q(2)*f_6];

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
