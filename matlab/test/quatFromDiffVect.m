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

% define environment variables
iter  = 100;
alpha = 0.05;
nRand = 10;

% create corner list
v1a   = {[ 1,  0, 0], [ 1,  0,  0], [ 0,  1,  0], [ 0,  1,  0], [ 0,  0,  1], [0,  0,  1]};
v2a   = {[ 0,  1, 0], [ 0,  0,  1], [ 1,  0,  0], [ 0,  0,  1], [ 1,  0,  0], [0,  1,  0]};
v1b   = {[-1,  0, 0], [-1,  0,  0], [ 0, -1,  0], [ 0, -1,  0], [ 0,  0, -1], [0,  0, -1]};
v2b   = {[ 0,  1, 0], [ 0,  0,  1], [ 1,  0,  0], [ 0,  0,  1], [ 1,  0,  0], [0,  1,  0]};
v1c   = {[ 1,  0, 0], [ 1,  0,  0], [ 0,  1,  0], [ 0,  1,  0], [ 0,  0,  1], [0,  0,  1]};
v2c   = {[ 0, -1, 0], [ 0,  0, -1], [-1,  0,  0], [ 0,  0, -1], [-1,  0,  0], [0, -1,  0]};
v1d   = {[-1,  0, 0], [-1,  0,  0], [ 0, -1,  0], [ 0, -1,  0], [ 0,  0, -1], [0,  0, -1]};
v2d   = {[ 0, -1, 0], [ 0,  0, -1], [-1,  0,  0], [ 0,  0, -1], [-1,  0,  0], [0, -1,  0]};
v1e   = {[ 1,  0, 0], [ 0,  1,  0], [ 0,  0,  1], [-1,  0,  0], [ 0, -1,  0], [0,  0, -1]};
v2e   = {[-1,  0, 0], [ 0, -1,  0], [ 0,  0, -1], [ 1,  0,  0], [ 0,  1,  0], [0,  0,  1]};
v1    = [v1a, v1b, v1c, v1d, v1e];
v2    = [v2a, v2b, v2c, v2d, v2e];
clear    v1a  v1b  v1c  v1d  v1e
clear    v2a  v2b  v2c  v2d  v2e

% add random vectors
for i=1:nRand
  t1      = 512 * rand(1,3) - 256;
  t2      = 512 * rand(1,3) - 256;
  t1      = t1 / sqrt(sum(t1.^2));
  t2      = t2 / sqrt(sum(t2.^2));
  v1      = [v1, t1];
  v2      = [v2, t2];
end
clear t1 t2

% test quaternion function
for i=1:length(v1)
  q1{i}   = quat("vec", v1{i}, v2{i});
  v3{i}   = q1{i} / v1{i};
  tmp     = v3{i} - v2{i};
  df(i)   = sqrt(sum((tmp).^2));
  if df(i) > 0.01
    error('accuracy failure');
  end
end

% test axis-angle compliment
for i=1:length(v1)
  v       = v1{i};
  for j=1:iter
    q     = quat("vec", v, v2{i});
    a     = q.axisAngle();
    a(1)  = a(1) * alpha;
    q     = quat("axisAngle", a);
    v     = q / v;
    plotVector(v1{i}, v2{i}, v);
    drawnow;
  end 
end
