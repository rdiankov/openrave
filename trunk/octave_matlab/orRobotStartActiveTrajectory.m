% orRobotStartActiveTrajectory(robotid, jointvalues, timestamps, transformations)
%
% Starts/Queues a robot trajectory of the robot where the size of
% each trajectory point is the current active degrees of freedom
% of the robot (others are held constants)
% D is the number of active degrees of freedom.
% N is the number of points of the trajectory
% robotid - unique id of the robot
% jointvalues - DxN matrix of the joint values of each point in the trajrectory.
% timestamps [optional] - the time stamps in seconds of each trajectory point.
% transformations [optional] - 12xN or 7xN matrix. The base link transformations of
%                              each trajectory point.
%                              If the column size is 12, then it is a 3x4 matrix
%                              in column first order
%                              If the column size is 7, then it is a quaterion and a translation.
%                              If active degrees of freedom contains a affine transformation component
%                              it is overwritten with the transformations matrices
function [] = orRobotStartActiveTrajectory(robotid, jointvalues, timestamps, transformations)

numpts = size(jointvalues,2);
usetrans = 0;
usetimestamps = 0;

if( exist('timestamps','var') )
    usetimestamps = ~isempty(timestamps);
    if( usetimestamps & length(timestamps) ~= numpts )
        error('timestamps has wrong dimensions');
    end
else
    timestamps = [];
end

if( exist('transformations','var') )
    
    
    if( ~isempty(transformations) )
        if( size(transformations,2) ~= numpts )
            error('transformations has wrong dimensions %d', size(transformations,2));
        end

        if( size(transformations,1) == 12 )
            usetrans = 1;
        elseif( size(transformations,1) == 7 )
            usetrans = 2;
        else
            error('cannot determine transformation type %d', size(transformations,1));
        end
    end

else
    transformations = [];
end

command_str = [sprintf('robot_traj %d %d %d %d ', robotid, numpts, usetimestamps, usetrans) ...
    sprintf('%f ', jointvalues, timestamps, transformations)];

out = orCommunicator(command_str);
%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting active DOFs');
%end
