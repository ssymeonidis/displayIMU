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
  gFstEn           % fast gyro enable
  aAlpha           % accl weight
  mAlpha           % magn weight
  mDot             % magn dot product
  mNrmEn           % ortho-normalize enable
  vAlpha           % velocity weight
  vEn              % velocity enable
end

properties         % state structure
  tSys             % last system time
  tGyro            % last gyro sample time
  tAccl            % last accl sample time
  tMagn            % last magn sample time
  qSys             % last system state
  qGyro            % last gyro state
  qAccl            % last accl state
  qMagn            % last magn state
  qVel             % last velocity state
  gFltr            % filtered gyro vals
  aReset           % accl reset
  mReset           % magn reset
  gReset           % gyro reset
  vReset           % velocity reset
end


methods


%% constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj       = imuSLERP()
  obj.aAlpha       = 0.1;
  obj.mAlpha       = 0.1;
  obj.gAlpha       = 0.5;
  obj.mDot         = -0.5547;
  obj.gFstEn       = false;
  obj.vAlpha       = 0.5;
  obj.vEn          = true;
  obj.qSys         = quat;
  obj.qVel         = quat;
  obj.tSys         = 0;
  obj.tGyro        = 0;
  obj.qGyro        = quat;
  obj.aReset       = true;
  obj.mReset       = true;
  obj.gReset       = true;
  obj.vReset       = true;
end


%% apply gyroscope rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM       = updateGyro(obj, gyro, t)

  % inialize figure-of-merit
  FOM              = 0;

  % check for velocity reset
  if obj.vReset
    if obj.gFstEn
      obj.gFltr    = gyro;
    else
      obj.qVel     = quat(0.5 * gyro);
      obj.qVel(1)  = sqrt(1.0 - sum(obj.qVel(2:4).^2));
    end
    obj.vReset     = false;
  end
    
  % check for reset conditions
  if obj.gReset
    obj.tSys       = t;
    obj.tGyro      = t;
    obj.qGyro      = obj.qSys;
    obj.gReset     = false;
    return
  end
  
  % fast gyroscope update
  if obj.gFstEn
  
    % calcuate time delta and update time state
    dt             = t - obj.tSys;
    obj.tSys       = t;
    
    % update angular velocity (alpha filter)
    if obj.vAlpha < 1
      gyro         = (1 - obj.vAlpha) * obj.gFltr + obj.vAlpha * gyro;
      obj.gFltr    = gyro;
    end
   
    % update state by applying gyroscope rate
    q              =  obj.qSys.val;
    qVel(1)        = -0.5 * (q(2)*gyro(1) + q(3)*gyro(2) + q(4)*gyro(3));
    qVel(2)        =  0.5 * (q(1)*gyro(1) + q(3)*gyro(3) - q(4)*gyro(2));
    qVel(3)        =  0.5 * (q(1)*gyro(2) - q(2)*gyro(3) + q(4)*gyro(1));
    qVel(4)        =  0.5 * (q(1)*gyro(3) + q(2)*gyro(2) - q(3)*gyro(1));
    obj.qSys       =  obj.qSys + dt * qVel;
 
  else
     
    % update quaternion based on current datum
    dt             = t - obj.tGyro;
    qShift         = quat(0.5 * gyro * dt);
    qShift(1)      = sqrt(1.0 - sum(qShift(2:4).^2));
    qDatum         = obj.qGyro * qShift;
    
    % calculate angular difference between estm
    qEstm          = obj.estmState(t);
    qDiff          = qEstm / qDatum;
    xDiff          = qDiff.axisAngle;
    FOM            = xDiff(1);
 
    % update system quaternion
    xShift         = [obj.gAlpha * xDiff(1), xDiff(2:4)];
    qShift         = quat("axisAngle", xShift);
    obj.qSys       = qEstm * qShift;
    
    % update internal state
    obj.qGyro      = obj.qSys;
    obj.tSys       = t;
    obj.tGyro      = t;
  end
end


%% apply acclerometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM       = updateAccl(obj, accl, t, weight)

  % used defaults for unspecified params
  if nargin < 4
    weight         = 1;
  end

  % check for reset condition
  if obj.aReset
    obj.qSys       = quat("up", accl);
    obj.qAccl      = obj.qSys;
    obj.tSys       = t;
    obj.tAccl      = t;
    obj.aReset     = false;
    FOM            = 0;
    return
  end
      
  % estimate current state and reference vector
  qEstm            = obj.estmState(t);
  aEstm            = qEstm.up;
 
  % calcuate axis-angle shift
  qDiff            = quat("vec", aEstm, accl);
  xDiff            = qDiff.axisAngle;
  FOM              = xDiff(1);
    
  % update system quaternion
  xShift           = [weight * obj.aAlpha * xDiff(1), xDiff(2:4)];
  qShift           = quat("axisAngle", xShift);
  obj.qSys         = obj.qSys * qShift;
  obj.tSys         = t;

  % check for accl velocity enable
  if obj.gFstEn || ~obj.vEn 
    return
  end

  % calcuate rotational difference
  dt               = t - obj.tAccl;
  aEstm            = obj.qAccl.up;
  qDiff            = quat("vec", aEstm, accl);
  xDiff            = qDiff.axisAngle;
  
  % initialize velocity estimate
  if obj.vReset
    xVel           = [xDiff(1) / dt, xDiff(2:4)];
    obj.qVel       = quat("axisAngle", xVel);
    obj.vReset     = false;
    
  % update velocity estimate
  else
    % TBD
  end
  
  % udpate system quaternion
  obj.qAccl        = obj.qSys;
  obj.tAccl        = t;

end



%% apply magn datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM       = updateMagn(obj, magn, t, weight)
  
  % used defaults for unspecified params
  if nargin < 4
    weight         = 1;
  end
  
  % check for reset condition
  if obj.mReset
    obj.qSys       = quat;
    obj.qMagn      = obj.qSys;
    obj.tSys       = t;
    obj.tMagn      = t;
    obj.mReset     = false;
    FOM            = 0;
    return
  end

  % estimate current state and refernce vector
  qEstm            = obj.estmState(t);
  mEstm            = obj.estmMagn(qEstm);
 
  % normalize magnetometer (if configured so)
  if obj.mNrmEn
    aEstm          = obj.qSys.up;
    n              = dot(magn, aEstm);
    magn           = magn - n * aEstm;
  end
  
  % calcuate axis-angle shift
  qDiff            = quat("vec", mEstm, magn);
  xDiff            = qDiff.axisAngle;
  FOM              = xDiff(1);
    
  % update system quaternion
  xShift           = [weight * obj.mAlpha * xDiff(1), xDiff(2:4)];
  qShfit           = quat("axisAngle", xShift);
  obj.qSys         = obj.qSys * qShfit;
  obj.tSys         = t;

  % check for accl velocity enable
  if obj.gFstEn || ~obj.vEn 
    return
  end

  % calcuate rotational difference
  dt               = t - obj.tMagn;
  mEstm            = obj.estmMagn(obj.qMagn);
  qDiff            = quat("vec", mEstm, magn);
  xDiff            = qDiff.axisAngle;
  
  % initialize velocity estimate
  if obj.vReset
    xVel           = [xDiff(1) / dt, xDiff(2:4)];
    obj.qVel       = quat("axisAngle", xVel);
    obj.vReset     = false;
    
  % update velocity estimate
  else
    % TBD
  end
  
  % udpate system quaternion
  obj.qMagn        = obj.qSys;
  obj.tMagn        = t;
end


%% estimate current orientation state (quaternion)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function qEstm     = estmState(obj, t)

  % check for velocity disable
  if ~obj.vEn
    qEstm          = obj.qSys;
    return
  end

  % estimate state (apply velocity)
  dt               = t - obj.tSys;
  qDist            = dt * obj.qVel;
  qEstm            = obj.qSys * qDist;
end


%% estimate current orientation state (quaternion)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mEstm     = estmMagn(obj, q)

  % create reference vector (non-orthonormalized)
  if ~obj.mNrmEn
    mRef           = [sqrt(1 - obj.mDot.^2), 0, obj.mDot];
    mEstm          = q / mRef;
    
  % create reference vector (orthonormalized)
  else
    mEstm          = q.frwd;
  end
end


end % methods

end % classdef