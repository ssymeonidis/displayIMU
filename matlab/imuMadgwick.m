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
% along with this program. If not, see <http://www.gnu.org/liWcenses/>.


classdef imuMadgwick < handle
    
properties       % config structure
  gAlpha1        % gyro weight (cur state)
  gAlpha2        % gyro weight (est state)
  aAlpha         % accl weight
  mAlpha         % magn weight
  mRefX          % magn reference (x-axis)
  mRefZ          % magn reference (z-axis)
end

properties       % state structure
  gFltr1         % filtered gyro data (cur state)
  gFltr2         % filtered gyro data (est state)
  gReset         % gyro reset flag
  aReset         % accl reset flag
  mReset         % magn reset flag
  gTime          % last gyro sample time
  aTime          % last accl sample time
  mTime          % last magn sample time
  time           % last sample time
  qSys           % quaternion value
end

properties (Constant)
  isOrtho        = true;
end


methods


%% object constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj     = imuMadgwick()
  obj.gAlpha1    = 1.0;
  obj.gAlpha2    = 1.0;
  obj.aAlpha     = 0.2;
  obj.mAlpha     = 0.1;
  obj.gFltr1     = [0.0; 0.0; 0.0];
  obj.gFltr2     = [0.0; 0.0; 0.0];
  obj.gReset     = true;
  obj.aReset     = true;
  obj.mReset     = true;
  obj.qSys       = quat;
end


%% apply gyroscope rate (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM     = updateGyro(obj, t, g, weight)
  
  % initialize state after reset
  if obj.gReset == true
    obj.gFltr1   = g(:);
    obj.gFltr2   = g(:);
    obj.gReset   = false;
    obj.gTime    = t;
    obj.time     = t;
    FOM          = NaN;
    return
  end
  
  % assign default value to alpha
  if nargin < 4
    weight       = 1.0;
  end
  
  % filter gyroscope data
  obj.gFltr1     = obj.gAlpha1 * g(:) + (1 - obj.gAlpha1) * obj.gFltr1;
  obj.gFltr2     = obj.gAlpha2 * g(:) + (1 - obj.gAlpha2) * obj.gFltr2;
  
  % apply gyroscope rates
  obj.qSys       = obj.qSys.addRate(obj.gFltr1, weight * (t - obj.gTime));
  obj.gTime      = t;
  obj.time       = t;
  FOM            = NaN;
end


%% apply acclerometer datum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM     = updateAccl(obj, t, a, weight)
    
  % initialize state after reset
  if obj.aReset == true
    a            = a(:);
    if obj.mReset == true
      obj.qSys   = quat("up", a);
    else
      m          = obj.qSys.frwd;
      obj.qSys   = quat("upFrwd", a, m);
    end
    obj.aReset   = false;
    obj.aTime    = t;
    obj.time     = t;
    FOM          = NaN;
    return
  end

  % assign default value to alpha
  if nargin < 4
    weight      = 1;
  end
  
  % compute the objective function
  a              = a(:) / sqrt(sum(a(:).^2));
  q              = obj.qSys.val;
  q2             = 2.*q;
  f1             = -a(1) - q2(1)*q(3) + q2(2)*q(4);
  f2             = -a(2) + q2(1)*q(2) + q2(3)*q(4);
  f3             = -a(3) - q2(2)*q(2) - q2(3)*q(3) + 1;

  % calculate the gradient
  qHatDot        = [-f1*q2(3) + f2*q2(2),                ...
                     f1*q2(4) + f2*q2(1) - 2*f3*q2(2),   ...
                    -f1*q2(1) + f2*q2(4) - 2*f3*q2(3),   ...
                     f1*q2(2) + f2*q2(3)];
  qHatDot        = ~quat(qHatDot);

  % apply the gradient to q
  alpha          = weight * obj.aAlpha * (t - obj.aTime);
  obj.qSys       = obj.qSys - alpha * qHatDot.val;
  obj.qSys       = ~obj.qSys;
  obj.aTime      = t;
  obj.time       = t;
  FOM            = qHatDot(1);
end


%% apply magnetometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM     = updateMagn(obj, t, m, weight)

  % initialize state after reset
  if obj.mReset == true
    m            = m(:);
    a            = obj.qSys.up;
    obj.qSys     = quat("upFrwd", a, m);
    obj.mReset   = false;
    obj.mTime    = t;
    obj.time     = t;
    FOM          = NaN;
    return
  end

  % assign default value to alpha
  if nargin < 4
    weight        = 1;
  end
  
  % normalize the magnetometer vector
  m              = m(:);
  m              = m / sqrt(sum(m.^2));
  if obj.isOrtho == true
    u            = obj.qSys.up;
    D            = dot(u, m);
    m            = m - D*u;
    m            = m / sqrt(sum(m.^2));
  end

  % compute the objective function
  q              = obj.qSys.val;
  q2             = 2.*q;
  if obj.isOrtho == true
    f1           = -m(1) - q2(3)*q(3) - q2(4)*q(4) + 1;
    f2           = -m(2) + q2(2)*q(3) - q2(1)*q(4);
    f3           = -m(3) + q2(1)*q(3) + q2(2)*q(4);
  else
    vX           = obj.RefX;
    vZ           = obj.RefZ;
    f1           = vX * (1 - q2(3)*q(3) - q2(4)*q(4)) +                 ...
                   vZ * (q2(2)*q(4) - q2(1)*q(3)) - m(1);
    f2           = vX * (q2(2)*q(3) - q2(1)*q(4)) +                     ...
                   vZ * (q2(1)*q(2) + q2(3)*q(4)) - m(2);
    f3           = vX * (q2(1)*q(3) + q2(2)*q(4)) +                     ...
                   vZ * (1 - q2(2)*q(2) - q2(3)*q(3)) - m(3);
  end
    
  % calculate the gradient
  if obj.isOrtho == true
    qHatDot      = [            - f2*q2(4) + f3*q2(3),                  ...
                                + f2*q2(3) + f3*q2(4),                  ...
                    -2*f1*q2(3) + f2*q2(2) + f3*q2(1),                  ...
                    -2*f1*q2(4) - f2*q2(1) + f3*q2(2)];
  else
    qHatDot(1)   =  - f1*vZ*q2(3) - f2*(vX*q2(4)-vZ*q2(2)) + f3*vX*q2(3);
    qHatDot(2)   =  + f1*vZ*q2(4) + f2*(vX*q2(3)+vZ*q2(1))              ...
                    + f3*(vX*q2(4)-2*vZ*q2(2));
    qHatDot(3)   =  - f1*(2*vX*q2(3)+vZ*q2(1)) + f2*(vX*q2(2)+vZ*q2(4)) ...
                    + f3*(vX*q2(1)-2*vZ*q2(3));
    qHatDot(4)   =  - f1*(2*vX*q2(4)-vZ*q2(2)) - f2*(vX*q2(1)-vZ*q2(3)) ...
                    + f3*vX*q2(2);     
  end
  qHatDot        = ~quat(qHatDot);

  % apply the gradient to q
  alpha          = weight * obj.mAlpha * (t - obj.mTime);
  obj.qSys       = obj.qSys - alpha * qHatDot.val;
  obj.qSys       = ~obj.qSys;
  obj.mTime      = t;
  obj.time       = t;
  FOM            = qHatDot(1);
end


%% estimate current state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function qEstm   = estmQuat(obj, t)
  g              = obj.gFltr2;
  q              = obj.qSys.val;
  dq             = [-q(2)*g(1) - q(3)*g(2) - q(4)*g(3),     ...
                     q(1)*g(1) + q(3)*g(3) - q(4)*g(2),     ...
                     q(1)*g(2) - q(2)*g(3) + q(4)*g(1),     ...
                     q(1)*g(3) + q(2)*g(2) - q(3)*g(1)];
  delta          = 0.5 * (t - obj.time);
  qEstm          = obj.qSys + delta .* dq;
end


end % methods

end % classdef