% orBodyDestroy(bodyid)
%
% Destroys a body of id bodyid. bodyid can also be a robot.

function [] = orBodyDestroy(bodyid)

out = orCommunicator(['body_destroy ' num2str(bodyid)]);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error destroying body');
%end