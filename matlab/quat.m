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

classdef quat
    
properties
  val                     % quaternion value
end

properties (Constant)
  epsilon   = 0.001       % near zero threshold
  toDeg     = 180 / pi    % rad to degree
end

methods


%% quaternion constructor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q   = quat(arg1, arg2, arg3, arg4)
  % default argument
  if (nargin < 1)
    q.val    = [1, 0, 0, 0];

  % numeric argument
  elseif isnumeric(arg1)
    if     ((nargin == 1) && length(arg1) == 4)
      q.val = arg1;
    elseif ((nargin == 1) && length(arg1) == 3)
      q.val = [0, arg1(1), arg1(2), arg1(3)];
    elseif (nargin == 3)
      q.val = [0, arg1, arg2, arg3];
    elseif (nargin == 4)
      q.val = [arg1, arg2, arg3, arg4];
    else
      error("invalid numeric argument(s)");
    end

  % random quaternion
  elseif strcmp(arg1, "rand")
    q       = ~quat(rand(1,4));
  
  % euler angles
  elseif strcmp(arg1, "rad")
    if     (nargin == 2)
      q     = q.fromEuler(arg2);
    elseif (nargin == 4)
      q     = q.fromEuler([arg2, arg3, arg4]);
    else
      error("invalid numeric argument(s)");
    end
  elseif strcmp(arg1, "deg")
    if     (nargin == 2)
      q     = q.fromEuler(arg2 / q.toDeg);
    elseif (nargin == 4)
      q     = q.fromEuler([arg2, arg3, arg4] / q.toDeg);
    else
      error("invalid numeric argument(s)"); 
    end
  end
end


%% quaternion subscript reference
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function val = subsref(q, S)
  if       strcmp(S(1).type, '.')
    if     strcmp(S(1).subs, 'val')
      val      = q.val;
    elseif strcmp(S(1).subs, 'mag')
      val      = q.mag();
    elseif strcmp(S(1).subs, 'up')
      val      = q.toUp();
    elseif strcmp(S(1).subs, 'frwd')
      val      = q.toFrwd();
    elseif strcmp(S(1).subs, 'rght')
      val      = q.toRght();
    elseif strcmp(S(1).subs, 'rad')
      val      = q.toEuler();
    elseif strcmp(S(1).subs, 'deg')
      val      = q.toEuler() * q.toDeg;
    end
  elseif   strcmp(S(1).type, '()')
    if    (length(S(1).subs) < 1)
      val      = q.val;
    elseif strcmp(S(1).subs{1},':')
      val      = q.val;
    else
      val      = zeros(size(S(1).subs{1}));
      for i=1:length(S(1).subs{1})
        val(i) = q.val(S(1).subs{1}(i));
      end
    end
  end
end


%% quaternion magnitude
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function val = mag(q)
  val        = sqrt(sum(q.val.^2));
end


%% quaternion normalize
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q   = not(q)
  mag_sq     = sum(q.val.^2);
  if (mag_sq > q.epsilon) && (q.epsilon)
    mag      = sqrt(mag_sq);
    q.val    = q.val ./ mag;
  end
end


%% quaternion conjugate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = ctranspose(q)
  q.val    = [q.val(1), -q.val(2), -q.val(3), -q.val(4)];
end


%% quaternion multiply
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = mtimes(q, arg)
  q1       = q.val;
  q2       = arg.val;
  q.val(1) = q2(1)*q1(1) - q2(2)*q1(2) - q2(3)*q1(3) - q2(4)*q1(4);
  q.val(2) = q2(1)*q1(2) + q2(2)*q1(1) - q2(3)*q1(4) + q2(4)*q1(3);
  q.val(3) = q2(1)*q1(3) + q2(2)*q1(4) + q2(3)*q1(1) - q2(4)*q1(2);
  q.val(4) = q2(1)*q1(4) - q2(2)*q1(3) + q2(3)*q1(2) + q2(4)*q1(1);
end


%% quaternion rotate vector (forward)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = mrdivide(q, v)
  v        = q * quat(v) * q';
  v        = v.val(2:4);
end


%% quaternion rotate vector (reverse)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = mldivide(q, v)
  v        = q' * quat(v) * q;
  v        = v.val(2:4);
end


%% quaternion up component
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = toUp(q)
  v        = q / [0, 0, -1];
end


%% quaternion forward component
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = toFrwd(q)
  v        = q / [1, 0, 0];
end


%% quaternion forward component
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function v = toRght(q)
  v        = q / [0, 1, 0];
end


%% quaternion from Euler angles (radians)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function q = fromEuler(q, ang)
  cy = cos(ang(1) * 0.5);
  sy = sin(ang(1) * 0.5);
  cr = cos(ang(3) * 0.5);
  sr = sin(ang(3) * 0.5);
  cp = cos(ang(2) * 0.5);
  sp = sin(ang(2) * 0.5);

  % conversion to quaternion
  q.val(1) = cy*cr*cp + sy*sr*sp;
  q.val(2) = cy*sr*cp - sy*cr*sp;
  q.val(3) = cy*cr*sp + sy*sr*cp;
  q.val(4) = sy*cr*cp - cy*sr*sp;
end


%% quaternion from Euler angles (radians)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ang = toEuler(q)
  % extract core vector
  q         = q.val;
    
  % roll (x-axis rotation)
  sinr_cosp = 2.0 * (q(1)*q(2) + q(3)*q(4));
  cosr_cosp = 1.0 - 2.0 * (q(2)*q(2) + q(3)*q(3));
  ang(3)    = atan2(sinr_cosp, cosr_cosp);

  % pitch (y-axis rotation)
  sinp      = 2.0 * (q(1)*q(3) - q(4)*q(2));
  if (abs(sinp) >= 1)
    ang(2)  = sign(sinp) * pi / 2;
  else
    ang(2)  = asin(sinp);
  end

  % yaw (z-axis rotation)
  siny_cosp = 2.0 * (q(1)*q(4) + q(2)*q(3));
  cosy_cosp = 1.0 - 2.0 * (q(3)*q(3) + q(4)*q(4));
  ang(1)    = atan2(siny_cosp, cosy_cosp);
end


end % methods

end % classdef