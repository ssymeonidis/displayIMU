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
global imu csv_enable csv_magn_scale csv_time_scale csv_file csv_time
addpath('..');

% define simulation parameterse
csv_enable       = true;
csv_filename     = '../../stim/applyMagnTest.csv';
csv_magn_scale   = 255;
csv_time_scale   = 0.00001;
csv_time         = 0;
imu              = imuGradient;
imu.mAlpha       = 0.005;

% create csv file (used to create stimulus)
if csv_enable
  csv_file       = fopen(csv_filename, 'w');
  fprintf(csv_file, "3, 0, %d, 0, 0\n", csv_magn_scale);
end

% test 90-deg roll
magn   = [0, -1, 0];
euler  = run_sim(magn)

% test 180-deg roll
magn   = [-1, 0, 0];
euler  = run_sim(magn)

% test neg90-deg roll
magn   = [0, 1, 0];
euler  = run_sim(magn)

% test zero-pitch, zero-roll
magn   = [1, 0, 0];
euler  = run_sim(magn)

% create csv file (used to create stimulus)
if csv_enable
  fclose(csv_file);
end


%% run simulation given specified inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler = run_sim(magn)

  % import global csv parameters
  global imu csv_enable csv_magn_scale csv_time_scale csv_file csv_time

  % define local constants
  iter       = 300;
  if csv_enable
    dt       = 0.1;
    val      = round(magn*csv_magn_scale);
    accl_str = sprintf("%d, %d, %d", val(1), val(2), val(3));
  end

  % main processing loop
  FOM    = [];
  for i=1:iter
    FOM(i)  = imu.estmMagn(magn);
    display_state(imu.q);
    if csv_enable
      csv_time    = csv_time + dt;
      val         = round(csv_time/csv_time_scale);
      fprintf(csv_file, "3, %d, %s\n", val, accl_str);
    end
  end

  % return final state
  euler           = imu.q.deg;
end


%% update the display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  plotState(q);
  title('applyMagnTest');
  drawnow;
end
