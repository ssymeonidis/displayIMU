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

% initialize environment
clear all; close all;
addpath('..');
addpath('../utils');
global csv_enable csv_file imu datum dt

% define simulation parameterse
csv_filename     = '../../stim/applyGyroTest.csv';
csv_enable       = false;
csv_file         = [];
imu              = imuCore("SLERP");
datum.type       = 'gyro';
datum.t          = 0;
datum.val        = [0, 0, 0];
dt               = 0.1;

% create csv file (used to create stimulus)
if csv_enable
  csv_file       = fopen(csv_filename, 'w');
end

% rotate test #1
axis             = [1, 0, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(axis, range, speed);

% rotate test #2
axis             = [-1, 0, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(axis, range, speed);

% rotate test #3
axis             = [0, 1, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(axis, range, speed);

% rotate test #4
axis             = [0, -1, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(axis, range, speed);

% rotate test #5
axis             = [0, 0, 1];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(axis, range, speed);

% rotate test #6
axis             = [0, 0, -1];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(axis, range, speed);

% create csv file (used to create stimulus)
if csv_enable
  fclose(csv_file);
end


%% run simulation given specified inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler   = run_sim(axis, range, speed)

  % import global csv parameters
  global csv_enable csv_file imu datum dt

  % define local constants
  time           = range / speed;
  iter           = time  / dt;
  speed_rad      = pi * speed / 180;
  datum.val      = round(axis * speed_rad / imu.gScale);
  if csv_enable
    val          = datum.val;
    gyro_str     = sprintf("%d, %d, %d", val(1), val(2), val(3));
  end
  
  % main processing loop
  for i=1:iter
    FOM(i)       = imu.update(datum);
    q            = imu.estmQuat(datum.t);
    datum.t      = round(datum.t + dt / imu.tScale);
    display_state(q);
    if csv_enable
      fprintf(csv_file, "1, %d, %s\n", datum.t, gyro_str);
    end
  end

  % return final state
  euler          = q.deg;
end


%% update the display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  plotState(q);
  title('estmGyroTest');
  drawnow;
end