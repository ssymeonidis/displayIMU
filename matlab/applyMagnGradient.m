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

function [q, FOM] = applyMagnGradient(q, magn, ref, alpha)

% normalize the magnetometer vector
magn        = magn / sqrt(sum(magn.^2));

% compute the objective function 
twox        = 2 * ref(1);
twoz        = 2 * ref(3);
q1_q3       = q(1) * q(3);
q2_q4       = q(2) * q(4);
q3_q3       = q(3) * q(3);
f_4         = twox * (0.5 - q3_q3 - q(4)*q(4)) +           ...
              twoz * (q2_q4 - q1_q3) - magn(1);
f_5         = twox * (q(2)*q(3) - q(1)*q(4)) +             ...
              twoz * (q(1)*q(2) + q(3)*q(4)) - magn(2);
f_6         = twox * (q1_q3 + q2_q4) +                     ...
              twoz * (0.5 - q(2)*q(2) - q3_q3) - magn(3);

% compute the Jacobian
twox_q      = twox * q;
twoz_q      = twoz * q;
J_41        = twoz_q(3);
J_42        = twoz_q(4);
J_43        = 2*twox_q(3) + twoz_q(1); 
J_44        = 2*twox_q(4) - twoz_q(2);
J_51        = twox_q(4) - twoz_q(2);
J_52        = twox_q(3) + twoz_q(1);
J_53        = twox_q(2) + twoz_q(4);
J_54        = twox_q(1) - twoz_q(3);
J_61        = twox_q(3);
J_62        = twox_q(4) - 2*twoz_q(2);
J_63        = twox_q(1) - 2*twoz_q(3);
J_64        = twox_q(2);

% calculate the gradient
qHatDot     = [-J_41*f_4 - J_51*f_5 + J_61*f_6,      ...
                J_42*f_4 + J_52*f_5 + J_62*f_6,      ...
               -J_43*f_4 + J_53*f_5 + J_63*f_6,      ...
               -J_44*f_4 - J_54*f_5 + J_64*f_6];

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
