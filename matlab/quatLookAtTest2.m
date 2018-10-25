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

%%
% clear all; close all;

% simple forward/up test
%euler  = [360*rand(), 180*rand()-90, 360*rand()-180];
q1_in  = eulerToQuat(180*euler/pi);
u1_in  = 255*quatToUp(q1_in);
f1_in  = 255*quatToForward(q1_in);
q1_out = quatFromForwardUp(f1_in, u1_in);
u1_out = 255*quatToUp(q1_out);
f1_out = 255*quatToUp(q1_out);

