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

classdef imuSLERP < handle
    
properties         % config structure
  gAlpha           % gyro weight
  aAlpha           % accl weight
  mAlpha           % magn weight
  vAlpha           % velocity weight
end

properties         % state structure
  qSys             % last system state
  qGyro            % last gyro state
  qAccl            % last accl state
  qMagn            % last magn state
  vFltr            % filtered rate vals
  tSys             % last system time
  tGyro            % last gyro sample time
  tAccl            % last accl sample time
  tMagn            % last magn sample time
  aReset           % accl reset
  mReset           % magn reset
  gReset           % gyro reset
  vReset           % velocity reset
end

properties (Constant)
  isGyroFltr       = false;
end


methods


%% constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj       = imuSLERP()
  obj.qSys         = quat;
  obj.qGyro        = quat;
  obj.qAccl        = quat;
  obj.qMagn        = quat;
  obj.vFltr        = [0, 0, 0];
  obj.tSys         = 0;
  obj.tGyro        = 0;
  obj.tAccl        = 0;
  obj.tMagn        = 0;
  obj.gAlpha       = 1.0;
  obj.aAlpha       = 0.1;
  obj.mAlpha       = 0.1;
  obj.vAlpha       = 0.1;
  obj.aReset       = true;
  obj.mReset       = true;
  obj.gReset       = true;
  obj.vReset       = true;
end


%% apply gyroscope rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM       = updateGyro(obj, t, g, weight)

  % initialize state after reset
  if obj.gReset
    obj.qGyro      = obj.qSys;
    obj.vFltr      = g(:);
    obj.tGyro      = t;
    obj.tSys       = t;
    obj.gReset     = false;
    obj.vReset     = false;
    FOM            = NaN;
    return;
  end
  
  % calculate datum weight (function needs to asymptoic to one)
  if obj.isGyroFltr
    alpha          = obj.Alpha * (t - obj.tGyro);
    alpha          = alpha / (1 + alpha);
  else
    alpha          = 1.0;
  end
  if nargin > 3
    alpha          = weight * alpha;
  end

  % update system orientation
  qEstm            = obj.estmQuat(t);
  qMeas            = obj.qGyro.addRate(g(:), (t - obj.tGyro));
  qDiff            = qEstm'*qMeas;
  vDiff            = qDiff.rate();
  obj.qSys         = qEstm.addRate(vDiff(:), alpha);
  obj.qGyro        = obj.qSys;
  obj.tSys         = t;
  obj.tGyro        = t;
  FOM              = qDiff.dist();
  
  % update velocity estimate
  alpha            = alpha * obj.vAlpha;
  obj.vFltr        = alpha * g(:) + (1-alpha) * obj.vFltr;
end


%% apply acclerometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM       = updateAccl(obj, t, a, weight)

   % check for reset condition
  if obj.aReset
    if obj.mReset 
      obj.qSys     = quat("up", a(:));
    else
      obj.qSys     = quat("upFrwd", a(:), obj.qSys.frwd);
    end
    obj.qAccl      = obj.qSys;
    obj.qGyro      = obj.qSys;
    obj.tSys       = t;
    obj.tAccl      = t;
    obj.aReset     = false;
    FOM            = NaN;
    return
  end
      
  % calculate datum weight (function needs to asymptoic to one)
  alpha            = obj.aAlpha * (t - obj.tAccl);
  alpha            = alpha / (1 + alpha);
  if nargin > 3
    alpha          = weight * alpha;
  end
  
  % estimate current state
  qEstm            = obj.estmQuat(t);
  qMeas            = quat("upFrwd", a, qEstm.frwd);
  qDiff            = qEstm'*qMeas;
  vDiff            = qDiff.rate();
  obj.qSys         = qEstm.addRate(vDiff, alpha);
  obj.tSys         = t;
  FOM              = qDiff.dist();

  % update current velocity
  qDist            = obj.qAccl'*qMeas;
  vDist            = qDist.rate(t - obj.tAccl);
  if obj.vReset
    obj.vFltr      = vDist;
    obj.qAccl      = obj.qAccl;
    obj.qGyro      = obj.qSys;
    obj.tAccl      = t;
    obj.tGyro      = t;
    obj.vReset     = false;
  else
    alpha          = alpha * obj.vAlpha;
    obj.vFltr      = alpha * vDist + (1-alpha) * obj.vFltr;
    obj.qAccl      = obj.qSys;
    obj.tAccl      = t;
  end
end



%% apply magn datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM       = updateMagn(obj, t, m, weight)
  
   % check for reset condition
  if obj.mReset
    obj.qSys       = quat("upFrwd", obj.qSys.up, m(:));
    obj.qMagn      = obj.qSys;
    obj.qGyro      = obj.qSys;
    obj.tSys       = t;
    obj.tMagn      = t;
    obj.mReset     = false;
    FOM            = NaN;
    return
  end
  
  % calculate datum weight (function needs to asymptoic to one)
  alpha            = obj.mAlpha * (t - obj.aTime);
  alpha            = alpha / (1 + alpha);
  if nargin > 3
    alpha          = weight * alpha;
  end
  
  % estimate current state
  qEstm            = obj.estmQuat(t);
  qMeas            = quat("upFrwd", qEstm.up, m);
  vDiff            = qEstm'*qMeas;
  gDiff            = vDiff.rate();
  obj.qSys         = qEstm.addRate(gDiff(:), alpha);
  obj.tSys         = t;

  % update current velocity
  qDist            = obj.qAccl'*qMeas;
  vDist            = qDist.rate(t - obj.aTime);
  if obj.vReset
    obj.vFltr      = vDist;
    obj.qAccl      = obj.qAccl;
    obj.qGyro      = obj.qSys;
    obj.aTime      = t;
    obj.gTime      = t;
    obj.vReset     = false;
  else
    alpha          = alpha * obj.vFltr;
    obj.vFltr      = alpha * vDist + (1-alpha) * obj.vFltr;
    obj.qAccl      = obj.qSys;
    obj.aTime      = t;
  end
end


%% estimate current orientation state (quaternion)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function qEstm     = estmQuat(obj, t)
  qEstm            = obj.qSys.addRate(obj.vFltr, (t - obj.tSys));
end

end % methods

end % classdef