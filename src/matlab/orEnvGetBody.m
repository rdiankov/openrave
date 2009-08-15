% id = orEnvGetBody(bodyname)
%
% returns the id of the body that corresponds to bodyname


function id = orEnvGetBody(bodyname)

out = orCommunicator(['env_getbody ' num2str(bodyname)], 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error getting body');
%end

id = str2double(out);