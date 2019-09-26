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

% last run on 11/25/19

% initialize simulation
clear all; close all;
addpath('..');
disp(' ');

% define environment variables
nRand = 10;

% test reversability using different inputs
for i=1:nRand
  a1(1) = 360*rand(1)-180;
  a1(2) = 180*rand(1)-90;
  a1(3) = 360*rand(1)-180;
  a1    = round(a1, 1);
  disp(['in  = ', num2str(a1(1)), ', ', num2str(a1(2)), ', ' num2str(a1(3))]);
  q     = quat("eulerDeg", a1);
  a2    = q.eulerDeg;
  disp(['out = ', num2str(a2(1)), ', ', num2str(a2(2)), ', ' num2str(a2(3))]);
  disp(' ');
end