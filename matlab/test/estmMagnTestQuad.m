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

% initialize simulation
clear all; close all;
addpath('..');
addpath('../utils');

% test 0deg-yaw
euler             = [25, 15, 15];
magn              = [1, 0, 0];
run_sim(euler, magn)

% test 90deg-yaw
euler             = [65, 15, 15];
magn              = [0, -1, 0];
run_sim(euler, magn)

% test neg90deg-yaw
euler             = [-65, 15, 15];
magn              = [0, 1, 0];
run_sim(euler, magn)

% test 180deg-yaw
euler             = [-205, 15, 15];
magn              = [-1, 0, 0];
run_sim(euler, magn)


%% run simulation given specified inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler    = run_sim(euler, magn)

  % define local constants
  imu             = imuCore("madgwick");
  imu.imu.qSys    = quat("deg", euler);
  imu.imu.mTime   = 0;
  imu.imu.time    = 0;
  imu.imu.mReset  = false;
  imu.imu.mAlpha  = 0.075;
  iter            = 100;
  dt              = 0.1;

  % plot the initial state
  q               = imu.estmQuat(0);
  display_state(q);
  
  % main processing loop
  datum.type      = 'magn';
  datum.t         = dt / imu.tScale;
  datum.val       = magn;
  for i=1:iter
    FOM(i)        = imu.update(datum);
    q             = imu.estmQuat(datum.t);
    datum.t       = datum.t + dt/imu.tScale;
    display_state(q);
  end
  
  % print results
  euler           = q.deg;
end


%% main function (performs conversion and generates plot)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  plotState(q);
  title('estmMagnTest');
  drawnow;
end