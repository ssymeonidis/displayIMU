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

classdef imuGradient < handle
    
properties                % config structure
  aAlpha                  % accelerometer weight
  mAlpha                  % magnetometer weight
end

properties                % state structure
  t                       % last sample time
  q                       % quaternion value
  aReset                  % acceleromter reset
  mReset                  % magnetometer weight
end


methods


%% constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj   = imuGradient()
  % initialize state
  obj.q        = quat;
  obj.aReset   = true;
  obj.mReset   = true;
end


%% apply gyroscope rate (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj   = estmGyro(obj, gyro, dt)
  q            = obj.q.val;
  tmp(1)       = -0.5 * (q(2)*gyro(1) + q(3)*gyro(2) + q(4)*gyro(3));
  tmp(2)       =  0.5 * (q(1)*gyro(1) + q(3)*gyro(3) - q(4)*gyro(2));
  tmp(3)       =  0.5 * (q(1)*gyro(2) - q(2)*gyro(3) + q(4)*gyro(1));
  tmp(4)       =  0.5 * (q(1)*gyro(3) + q(2)*gyro(2) - q(3)*gyro(1));
  obj.q        =  obj.q + dt*tmp;
end


%% apply acclerometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM = estmAccl(obj, accl)
  % normalize the acceleration vector
  mag        = sqrt(sum(accl.^2));
  if (mag > 0.001)
    accl     = accl / mag;
  end

  % compute the objective function 
  q          = obj.q.val;
  two_q      = 2 * q;
  f_1        = two_q(2)*q(4) - two_q(1)*q(3) - accl(1);
  f_2        = two_q(1)*q(2) + two_q(3)*q(4) - accl(2);
  f_3        = 1 - two_q(2)*q(2) - two_q(3)*q(3) - accl(3);

  % calculate the gradient
  qHatDot    = [two_q(2)*f_2 - two_q(3)*f_1,                    ...
                two_q(4)*f_1 + two_q(1)*f_2 - 2*two_q(2)*f_3,   ...
                two_q(4)*f_2 - 2*two_q(3)*f_3 - two_q(1)*f_1,   ...
                two_q(2)*f_1 + two_q(3)*f_2];

  % apply the gradient to q
  qHatDot    = ~quat(qHatDot);
  obj.q      = obj.q - obj.mAlpha*qHatDot.val;
  obj.q      = ~obj.q;
  FOM        = qHatDot(1);
end


%% apply magnetometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM  = estmMagn(obj, magn)
  % normalize the magnetometer vector
  magn        = magn / sqrt(sum(magn.^2));
  u           = obj.q.up;
  D           = dot(u, magn);
  magn        = magn - D*u;
  magn        = magn / sqrt(sum(magn.^2));

  % compute the objective function
  q           = obj.q.val;
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
  qHatDot    = ~quat(qHatDot);
  obj.q      = obj.q - obj.mAlpha*qHatDot.val;
  obj.q      = ~obj.q;
  FOM        = qHatDot(1);
end


end % methods

end % classdef