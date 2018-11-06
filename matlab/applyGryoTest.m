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
global csv_enable csv_gyro_scale csv_time_scale csv_file csv_time

% define simulation parameterse
csv_enable       = true;
csv_filename     = '../stim/applyGyroTest.csv';
csv_gyro_scale   = 0.001;
csv_time_scale   = 0.00001;
csv_time         = 0;

% create csv file (used to create stimulus)
if csv_enable
  csv_file       = fopen(csv_filename, 'w');
end

% rotate test #1
euler            = [0, 0, 0];
axis             = [1, 0, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(euler, axis, range, speed);

% rotate test #2
axis             = [-1, 0, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(euler, axis, range, speed);

% rotate test #3
axis             = [0, 1, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(euler, axis, range, speed);

% rotate test #4
axis             = [0, -1, 0];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(euler, axis, range, speed);

% rotate test #5
axis             = [0, 0, 1];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(euler, axis, range, speed);

% rotate test #6
axis             = [0, 0, -1];
range            = 90;              % deg
speed            = 15;              % deg/sec
euler            = run_sim(euler, axis, range, speed);

% create csv file (used to create stimulus)
if csv_enable
  fclose(csv_file);
end


%% run simulation given specified inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler   = run_sim(euler, axis, range, speed)

  % import global csv parameters
  global csv_enable csv_gyro_scale csv_time_scale csv_file csv_time

  % define local constants
  dt             = 0.1;
  time           = range / speed;
  iter           = time  / dt;
  speed_rad      = pi * speed / 180;
  gyro           = axis * speed_rad;
  if csv_enable
    val          = round(gyro/csv_gyro_scale);
    gyro_str     = sprintf("%d, %d, %d", val(1), val(2), val(3));
  end
  
  % main processing loop
  euler_rad      = pi * euler / 180;
  q              = eulerToQuat(euler_rad);
  for i=1:iter
    q            = applyGyroIntegrate(q, gyro, dt);
    display_state(q);
    if csv_enable
      csv_time    = csv_time + dt;
      val         = round(csv_time/csv_time_scale);
      fprintf(csv_file, "1, %d, %s\n", val, gyro_str);
    end
  end

  % return final state
  euler_rad    = quatToEuler(q);
  euler        = 180 * euler_rad / pi;
end


%% update the display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_state(q)
  plotState(q);
  title('applyGyroTest');
  drawnow;
end