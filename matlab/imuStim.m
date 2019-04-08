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

classdef imuStim < handle
    
properties         % config structure
  mDeg
end

properties         % state structure
  qCur
  figure
end


methods


%% constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj       = imuStim()
  % add plot utilities to path
  addpath('./utils');
    
  % initialize config structure
  obj.mDeg         = 135;

  % initialize state structure
  obj.qCur         = quat;
  obj.figure       = [];
end


%% apply gyroscope rate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function datum     = genDatum(obj)
    
  % create vectors
  datum.accl       = obj.genAccl();
  datum.magn       = obj.genMagn();
  
  % display vectors
  if ~isempty(obj.figure)
    figure(obj.figure);
    plotVector(datum.accl, datum.magn);
  end
end


%% apply acclerometer datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function accl      = genAccl(obj)
  accl             = obj.qCur / [0, 0, 1];
end


%% apply magn datum (lowest level function)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function magn      = genMagn(obj)
  mRad             = pi * obj.mDeg / 180;
  mDot             = cos(mRad);
  magn             = [sqrt(1-mDot), 0, mDot];
  magn             = obj.qCur / magn;
end

end % methods

end % classdef