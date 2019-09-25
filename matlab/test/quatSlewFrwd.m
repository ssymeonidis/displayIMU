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
clear all; %close all;
addpath('..');
addpath('../utils');

% define environment variables
iter  = 100;
alpha = 0.05;
nRand = 10;

% create corner list
u1a   = {[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0]};
f1a   = {[0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 0, 1]};
u2a   = {[0, 0, 1], [1, 0  0], [0, 1, 0], [1, 0, 0]};
f2a   = {[0, 1, 0], [0, 0, 1], [0, 0, 1], [0, 1, 0]};

u1b   = {[0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]};
f1b   = {[1, 0, 0], [1, 0, 0], [0, 0, 1], [0, 0, 1]};
u2b   = {[0, 0, 1], [0, 1  0], [0, 1, 0], [1, 0, 0]};
f2b   = {[1, 0, 0], [0, 0, 1], [1, 0, 0], [0, 0, 1]};

u1c   = {[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]};
f1c   = {[1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 1, 0]};
u2c   = {[0, 1, 0], [0, 0  1], [1, 0, 0], [0, 0, 1]};
f2c   = {[1, 0, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0]};

u1d   = {[0, 0, -1], [0, 0, -1], [0, 0, -1], [0, 0, -1]};
f1d   = {[1, 0,  0], [1, 0,  0], [0, 1,  0], [0, 1,  0]};
u2d   = {[0, 1,  0], [0, 0  -1], [1, 0,  0], [0, 0, -1]};
f2d   = {[1, 0,  0], [0, 1,  0], [0, 1,  0], [1, 0,  0]};

u1e   = {[1,  0, 0], [1,  0, 0], [1, 0,  0], [1, 0,  0]};
f1e   = {[0, -1, 0], [0, -1, 0], [0, 0, -1], [0, 0, -1]};
u2e   = {[0,  0, 1], [1,  0  0], [0, 1,  0], [1, 0,  0]};
f2e   = {[0, -1, 0], [0,  0, 1], [0, 0, -1], [0, 1,  0]};

u1f   = {[ 0, 0, -1], [ 0, 0, -1], [0,  0, -1], [0,  0, -1]};
f1f   = {[-1, 0,  0], [-1, 0,  0], [0, -1,  0], [0, -1,  0]};
u2f   = {[ 0, 1,  0], [ 0, 0  -1], [1,  0,  0], [0,  0, -1]};
f2f   = {[-1, 0,  0], [ 0, 1,  0], [0, -1,  0], [1,  0,  0]};

u1g   = {[ 0,  0, -1], [ 0,  0, -1], [ 0,  0, -1], [ 0,  0, -1]};
f1g   = {[-1,  0,  0], [-1,  0,  0], [ 0, -1,  0], [ 0, -1,  0]};
u2g   = {[ 0, -1,  0], [ 0,  0  -1], [-1,  0,  0], [ 0,  0, -1]};
f2g   = {[-1,  0,  0], [ 0, -1,  0], [ 0, -1,  0], [-1,  0,  0]};

u1h   = {[0, -1, 0], [0, -1, 0], [0, -1, 0], [0, -1, 0]};
f1h   = {[1,  0, 0], [1,  0, 0], [0,  0, 1], [0,  0, 1]};
u2h   = {[0,  0, 1], [0, -1  0], [0, -1, 0], [1,  0, 0]};
f2h   = {[1,  0, 0], [0,  0, 1], [1,  0, 0], [0,  0, 1]};

u1i   = {[ 0, 1, 0], [ 0, 1, 0], [0, 1,  0], [0, 1,  0]};
f1i   = {[-1, 0, 0], [-1, 0, 0], [0, 0, -1], [0, 0, -1]};
u2i   = {[ 0, 0, 1], [ 0, 1  0], [0, 1,  0], [1, 0,  0]};
f2i   = {[-1, 0, 0], [ 0, 0, 1], [1, 0,  0], [0, 0, -1]};

u1j   = {[ 0, -1, 0], [ 0, -1, 0], [ 0, -1,  0], [ 0, -1,  0]};
f1j   = {[-1,  0, 0], [-1,  0, 0], [ 0,  0, -1], [ 0,  0, -1]};
u2j   = {[ 0,  0, 1], [ 0, -1  0], [ 0, -1,  0], [ 1,  0,  0]};
f2j   = {[-1,  0, 0], [ 0,  0, 1], [ 1,  0,  0], [ 0,  0, -1]};

u1k   = {[ 0, -1,  0], [ 0, -1,  0], [ 0, -1,  0], [ 0, -1,  0]};
f1k   = {[-1,  0,  0], [-1,  0,  0], [ 0,  0, -1], [ 0,  0, -1]};
u2k   = {[ 0,  0, -1], [ 0, -1   0], [ 0, -1,  0], [-1,  0,  0]};
f2k   = {[-1,  0,  0], [ 0,  0, -1], [-1,  0,  0], [ 0,  0, -1]};

u1l   = {[0, 0, -1], [0, 0, -1], [0, 0, -1], [0, 0, -1]};
f1l   = {[1, 0,  0], [1, 0,  0], [0, 1,  0], [0, 1,  0]};
u2l   = {[0, 1,  0], [0, 0  -1], [1, 0,  0], [0, 0, -1]};
f2l   = {[1, 0,  0], [0, 1,  0], [0, 1,  0], [1, 0,  0]};

u1m   = {[ 0, 0, 1], [ 0, 0, 1], [0,  0, 1], [0,  0, 1]};
f1m   = {[-1, 0, 0], [-1, 0, 0], [0, -1, 0], [0, -1, 0]};
u2m   = {[ 0, 1, 0], [ 0, 0  1], [1,  0, 0], [0,  0, 1]};
f2m   = {[-1, 0, 0], [ 0, 1, 0], [0, -1, 0], [1,  0, 0]};

u1n   = {[ 0, 0, -1], [ 0, 0, -1], [0,  0, -1], [0,  0, -1]};
f1n   = {[-1, 0,  0], [-1, 0,  0], [0, -1,  0], [0, -1,  0]};
u2n   = {[ 0, 1,  0], [ 0, 0  -1], [1,  0,  0], [0,  0, -1]};
f2n   = {[-1, 0,  0], [ 0, 1,  0], [0, -1,  0], [1,  0,  0]};

u1o   = {[ 0,  0, -1], [ 0,  0, -1], [ 0,  0, -1], [ 0,  0, -1]};
f1o   = {[-1,  0,  0], [-1,  0,  0], [ 0, -1,  0], [ 0, -1,  0]};
u2o   = {[ 0, -1,  0], [ 0,  0  -1], [-1,  0,  0], [ 0,  0, -1]};
f2o   = {[-1,  0,  0], [ 0, -1,  0], [ 0, -1,  0], [-1,  0,  0]};

u1p   = {[ 1, 0, 0], [ 1, 0, 0], [0,  1, 0], [0,  1, 0]};
f1p   = {[ 0, 1, 0], [ 0, 0, 1], [1,  0, 0], [0,  0, 1]};
u2p   = {[-1, 0, 0], [-1, 0  0], [0, -1, 0], [0, -1, 0]};
f2p   = {[ 0, 1, 0], [ 0, 0, 1], [1,  0, 0], [0,  0, 1]};

u1q   = {[0, 0,  1], [0, 0,  1], [ 0,  1, 0], [0,  1,  0]};
f1q   = {[0, 1,  0], [1, 0,  0], [ 1,  0, 0], [0,  0,  1]};
u2q   = {[0, 0, -1], [0, 0, -1], [ 0, -1, 0], [0, -1,  0]};
f2q   = {[0, 1,  0], [1, 0,  0], [-1,  0, 0], [0,  0, -1]};

u1    = [u1a, u1b, u1c, u1d, u1e, u1f, u1g, u1h, u1i, u1j, u1k, u1l, u1m, u1n, u1o, u1p, u1q];
f1    = [f1a, f1b, f1c, f1d, f1e, f1f, f1g, f1h, f1i, f1j, f1k, f1l, f1m, f1n, f1o, f1p, f1q];
u2    = [u2a, u2b, u2c, u2d, u2e, u2f, u2g, u2h, u2i, u2j, u2k, u2l, u2m, u2n, u2o, u2p, u2q];
f2    = [f2a, f2b, f2c, f2d, f2e, f2f, f2g, f2h, f2i, f2j, f2k, f2l, f2m, f2n, f2o, f2p, f2q];
clear    u1a  u1b  u1c  u1d  u1e  u1f  u1g  u1h  u1i  u1j  u1k  u1l  u1m  u1n  u1o  u1p  u1q
clear    f1a  f1b  f1c  f1d  f1e  f1f  f1g  f1h  f1i  f1j  f1k  f1l  f1m  f1n  f1o  f1p  f1q
clear    u2a  u2b  u2c  u2d  u2e  u2f  u2g  u2h  u2i  u2j  u2k  u2l  u2m  u2n  u2o  u2p  u2q
clear    f2a  f2b  f2c  f2d  f2e  f2f  f2g  f2h  f2i  f2j  f2k  f2l  f2m  f2n  f2o  f2p  f2q

% add random vectors
for i=1:nRand
  v1  = 512 * rand(1,3) - 256;
  v2  = 512 * rand(1,3) - 256;
  v3  = 512 * rand(1,3) - 256;
  v4  = 512 * rand(1,3) - 256;
  v1  = v1 / sqrt(sum(v1.^2));
  v2  = v2 / sqrt(sum(v2.^2));
  v3  = v3 / sqrt(sum(v3.^2));
  v4  = v4 / sqrt(sum(v4.^2));
  u1  = [u1, v1];
  f1  = [f1, v2];
  u2  = [u2, v3];
  f2  = [f2, v4];
end
clear v1 v2 v3 v4

% test axis-angle compliment
for i=1:length(u1)
  q1{i}    = quat("upFrwd", u1{i}, f1{i});
  q2{i}    = quat("upFrwd", u2{i}, f2{i});
  q        = q1{i};
  figure(1);
  for j=1:iter
    d      = q / q2{i};
    FOM(j) = d.distDeg;
    d      = alpha * d;
    q      = q * d;
    plotVector(q2{i}.up, q2{i}.frwd, q2{i}.rght, q.up, q.frwd, q.rght);
    drawnow;
  end
  figure(2); plot(FOM); title('FOM (deg)'); drawnow;
end