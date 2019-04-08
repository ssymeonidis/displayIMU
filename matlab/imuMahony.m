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

% https://github.com/PaulStoffregen/MahonyAHRS/blob/master/src/MahonyAHRS.cpp

classdef imuMahony < handle
    
properties                % config structure
  aAlpha                  % accelerometer weight
  mAlpha                  % magnetometer weight
end

properties                % state structure
  t                       % last sample time
  qSys                    % quaternion value
  aReset                  % acceleromter reset
  mReset                  % magnetometer weight
end


methods


%% constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj   = imuMahony()
  % initialize state
  obj.qSys     = quat;
  obj.aAlpha   = 0.2;
  obj.mAlpha   = 1.0;
  obj.aReset   = true;
  obj.mReset   = true;
end


%% apply gyroscope rate (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj = updateGyro(obj, gyro, dt)
  q          =  obj.qSys.val;
  tmp(1)     = -0.5 * (q(2)*gyro(1) + q(3)*gyro(2) + q(4)*gyro(3));
  tmp(2)     =  0.5 * (q(1)*gyro(1) + q(3)*gyro(3) - q(4)*gyro(2));
  tmp(3)     =  0.5 * (q(1)*gyro(2) - q(2)*gyro(3) + q(4)*gyro(1));
  tmp(4)     =  0.5 * (q(1)*gyro(3) + q(2)*gyro(2) - q(3)*gyro(1));
  obj.qSys   =  obj.qSys + dt*tmp;
end


%% apply acclerometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM = updateAccl(obj, accl, t)

  % normalize acceleration vector
  mag        = sqrt(sum(accl.^2));
  if (mag > 0.001)
    accl     = accl / mag;
  end

  % estimate direction of gravity
  q          =  obj.qSys.val;
  halfv      = [q(2)*q(4) - q(1)*q(3),                ...
                q(1)*q(2) + q(3)*q(4),                ...
                q(1)*q(1) + q(4)*q(4) - 0.5];

  % calculate the error (cross product)
  halfe      = [accl(2)*halfv(3) - accl(3)*halfv(2),  ...
                accl(3)*halfv(1) - accl(1)*halfv(3),  ...
                accl(1)*halfv(2) - accl(2)*halfv(1)]; 

  % apply proportional feedback
  obj.qSys   = obj.qSys + obj.aAlpha*[0, halfe];
  obj.qSys   = ~obj.qSys;
  FOM        = 0;
end


%% apply magnetometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM = updateMagn(obj, magn, t)

  % normalize magnetometer vector
  mag        = sqrt(sum(magn.^2));
  if (mag > 0.001)
    magn     = magn / mag;
  end

  % reference direction on Earth's magnetic field
  q          = obj.qSys.val;
  hx         = 2 * (magn(1) * (0.5 - q(3)*q(3) - q(4)*q(4)) + ...
                    magn(2) * (q(2)*q(3) - q(1)*q(4))       + ...
                    magn(3) * (q(2)*q(4) + q(1)*q(3)));
  hy         = 2 * (magn(1) * (q(2)*q(3) + q(1)*q(4))       + ...
                    magn(2) * (0.5 - q(2)*q(2) - q(4)*q(4)) + ...
                    magn(3) * (q(3)*q(4) - q(1)*q(2)));
  bx         = sqrt(hx*hx + hy*hy);
  bz         = 2 * (magn(1) * (q(2)*q(4) - q(1)*q(3))       + ...
                    magn(2) * (q(3)*q(4) + q(1)*q(2))       + ...
                    magn(3) * (0.5 - q(2)*q(2) - q(3)*q(3)));
  
  % estimate direction of magnetic field
  halfw      = [bx * (0.5 - q(3)*q(3) - q(4)*q(4))          + ...
                bz * (q(2)*q(4) - q(1)*q(3)),                 ...
                bx * (q(2)*q(3) - q(1)*q(4))                + ...
                bz * (q(1)*q(2) + q(3)*q(4)),                 ...
                bx * (q(1)*q(3) + q(2)*q(4))                + ...
                bz * (0.5 - q(2)*q(2) - q(3)*q(3))];
                
  % calculate the error (cross product)
  halfe      = [magn(2)*halfw(3) - magn(3)*halfw(2),  ...
                magn(3)*halfw(1) - magn(1)*halfw(3),  ...
                magn(1)*halfw(2) - magn(2)*halfw(1)]; 
                
  % apply proportional feedback
  obj.qSys   = obj.qSys + obj.mAlpha*[0, halfe];
  obj.qSys   = ~obj.qSys;
  FOM        = 0;
end


end % methods

end % classdef