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


classdef imuCore < handle

properties                % config structure
  imuType                 % enumerated imu type
end

properties                % state structure
  imu                     % imu object
end

properties (Constant)
  tScale     = 0.00001;   % measurement to seconds
  gScale     = 0.001;     % measurement to rad/sec
end

methods

    
%% constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj = imuCore(imuType)

   % see default imu type
   if (nargin < 1)
     obj.imuType = 'madgwick';
     obj.imu     = imuMadgwick;
   
   % madgwick imu type
   elseif strcmp(imuType, 'madgwick')
     obj.imuType = 'madgwick';
     obj.imu     = imuMadgwick;
   
   % mahony imu type
   elseif strcmp(imuType, 'mahony')
     obj.imuType = 'mahony';
     obj.imu     = imuMahony;
   
   % SLERP imu type
   elseif strcmp(imuType, 'SLERP')
     obj.imuType = 'SLERP';
     obj.imu     = imuSLERP;
     
   % unidentified imu type
   else
     error('unsupported imuType');
   end
end


%% update datum
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function FOM = update(obj, datum)
  if     strcmp(datum.type, "gyro")
    FOM = obj.imu.updateGyro(obj.tScale*datum.t, obj.gScale*datum.val);
  elseif strcmp(datum.type, "accl")
    FOM = obj.imu.updateAccl(obj.tScale*datum.t, datum.val);
  elseif strcmp(datum.type, "magn")
    FOM = obj.imu.updateMagn(obj.tScale*datum.t, datum.val);
  end
end


%% estimate line-of-sight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function qEstm = estmQuat(obj, t)
  qEstm = obj.imu.estmQuat(obj.tScale*t);
end


end % methods

end % classdef