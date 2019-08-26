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
global csv_enable csv_magn_scale csv_file imu datum dt iter

% define simulation parameterse
csv_filename     = '../../stim/applyMagnTest.csv';
csv_enable       = false;
csv_magn_scale   = 255;
csv_file         = [];
imu              = imuCore("madgwick");
imu.imu.mAlpha   = 0.075;
datum.type       = 'magn';
dt               = 0.1;
iter             = 250;

% create csv file (used to create stimulus)
if csv_enable
  csv_file       = fopen(csv_filename, 'w');
end

% initialize imu
datum.t          = 0;
datum.val        = [1, 0, 0];
imu.update(datum);
datum.t          = round(datum.t + dt / imu.tScale);
if csv_enable
  fprintf(csv_file, "3, 0, %d, 0, 0", csv_magn_scale);
end


% test 90-deg roll
magn             = [0, -1, 0];
euler            = run_sim(magn)

% test 180-deg roll
magn             = [-1, 0, 0];
euler            = run_sim(magn)

% test neg90-deg roll
magn             = [0, 1, 0];
euler            = run_sim(magn)

% test zero-pitch, zero-roll
magn             = [1, 0, 0];
euler            = run_sim(magn)

% create csv file (used to create stimulus)
if csv_enable
  fclose(csv_file);
end


%% run simulation given specified inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler   = run_sim(magn)

  % import global csv parameters
  global csv_enable csv_magn_scale csv_file imu datum dt iter

  % define local constants
  if csv_enable
    val          = round(magn*csv_magn_scale);
    magn_str     = sprintf("%d, %d, %d", val(1), val(2), val(3));
  end

  % main processing loop
  datum.val      = magn;
  for i=1:iter
    FOM(i)       = imu.update(datum);
    q            = imu.estmQuat(datum.t);
    datum.t      = round(datum.t + dt / imu.tScale);
    display_state(q);
    if csv_enable
      fprintf(csv_file, "3, %d, %s\n", datum.t, magn_str);
    end
  end

  % return final state
  euler          = q.deg;
end


%% update the display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  plotState(q);
  title('applyMagnTest');
  drawnow;
end
