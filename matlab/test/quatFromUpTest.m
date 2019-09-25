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

% initialize environment
clear all; % close all;
addpath('..');
addpath('../utils');

% define envrionment variables
nRand = 10;

% create point list
u     = {[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]};

% add random vectors
for i=1:nRand
  v   = rand(1,3) - 0.5;
  v   = v / norm(v);
  u   = [u, v];
end

% test quaternion generation
for i=1:length(u)
  q1  = quat("up",     u{i});
  q2  = quat("upFast", u{i});
  figure(1); plotVector(u{i}); title('reference');
  figure(2); plotState(q1);    title('up safe');
  figure(3); plotState(q2);    title('up fast');
  v1  = round(u{i},  2);
  v2  = round(q1.up, 2);
  v3  = round(q2.up, 2);
  a1  = round(q1.eulerDeg, 1);
  a2  = round(q2.eulerDeg, 1);
  display(['in   = ',num2str(v1(1)),',',num2str(v1(2)),',',num2str(v1(3))]);
  display(['out1 = ',num2str(v2(1)),',',num2str(v2(2)),',',num2str(v2(3))]);
  display(['out2 = ',num2str(v3(1)),',',num2str(v3(2)),',',num2str(v3(3))]);
  display(['ang1 = ',num2str(a1(1)),',',num2str(a1(2)),',',num2str(a1(3))]);
  display(['ang2 = ',num2str(a2(1)),',',num2str(a2(2)),',',num2str(a2(3))]);
  display(' ');
  pause(3);
end