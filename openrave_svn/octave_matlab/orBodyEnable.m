% success = orBodyEnable(bodyid, enable)
%
% Enables or disables the body. If a body is disabled, collision detection
% and physics will will be turned off for it.

function [] = orBodyEnable(bodyid,enable)

out = orCommunicator(['body_enable ' num2str(bodyid) ' ' num2str(enable)]);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error enabling body');
%end