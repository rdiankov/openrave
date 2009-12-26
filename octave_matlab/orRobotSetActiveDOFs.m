% orRobotSetActiveDOfs(robotid, indices, affinedofs, rotationaxis)
%
% robotid - unique id of the robot
% indices - zero based indices of the robot joints to activate
% affinedofs [optional] - is a mask of affine transformations to enable
% 1 - X, adds 1 DOF
% 2 - Y, adds 1 DOF
% 4 - Z, adds 1 DOF
% 8 - RotationAxis, adds 1 DOF, rotationaxis has to be valid
% 16 - full 3D rotation, adds 3 DOF. Because any orientation is a rotation around some axis,
%                        the format of 3D rotation is axis*angle which rotates angle radians around axis
% 32 - quaternion rotation, adds 4 DOF. The quaternion [cos(angle/2) sin(angle/2)*axis]
% rotationaxis [optional] - the 3D rotation axis (if the RotationAxis bit is set in affinedofs)

function [] = orRobotSetActiveDOFs(robotid, indices, affinedofs, rotationaxis)

command_str = ['robot_setactivedofs ' num2str(robotid) ' ' num2str(max(size(indices))) ...
                                      ' ' sprintf('%d ', indices)];
if( exist('affinedofs', 'var') )
    command_str = [command_str ' ' sprintf('%d ', affinedofs)];
end

if( exist('rotationaxis', 'var') )
    command_str = [command_str ' '  sprintf('%f ',rotationaxis)];
else
    command_str = [command_str ' 0'];
end

out = orCommunicator(command_str);
%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting active DOFs');
%end
