% orBodySetTransform(bodyid, translation, quaternion)
% orBodySetTransform(bodyid, [quaternion translation])
% orBodySetTransform(bodyid, transform matrix) (12x1, 1x12, or 3x4)
%
% Set the affine transformation of the body. The transformation actually describes the first
% link of the body. The rest of the links are derived by the joint angles.
% a quaternion is related to axis and angle via: [cos(theta/2);sin(theta/2)*axis]
function [] = orBodySetTransform(varargin)

command_str = ['body_settransform ' num2str(varargin{1})];
if(nargin >= 3)
    command_str = [command_str ' ' sprintf('%f ', varargin{3}) ' ' sprintf('%f ', varargin{2})];
elseif(nargin == 2)
    command_str = [command_str ' ' sprintf('%f ', varargin{2})];    
else
    error('orBodySetTransform not enough arguments');
end

out = orCommunicator(command_str);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting transform');
%end
