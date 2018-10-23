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
clear all; close all;

% simple forward/up test
q = eulerToQuat(180*[125,5,10]/pi);
u = quatToUp(q);
f = quatToForward(q);
q_out1 = quatFromForwardUp(f, u);

% partial update test #1
q_init = [1, 0, 0, 0];
u = quatToUp(q);
f = quatToForward(q_init);
q_temp = quatFromForwardUp(f, u);
f = quatToForward(q);
u = quatToUp(q_temp);
q_out2 = quatFromForwardUp(f, u);

% partial update test #2 
% (this shouldn't work)
q_init = [1, 0, 0, 0];
f = quatToForward(q);
u = quatToUp(q_init);
q_temp = quatFromForwardUp(f, u);
f = quatToForward(q_temp);
u = quatToUp(q);
q_out3 = quatFromForwardUp(f, u);