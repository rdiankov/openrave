% dof = orBodyGetDOF(robotid)
%
% returns the number of active joints of the body

function dof = orBodyGetDOF(bodyid)

command_str = ['body_getdof ' num2str(bodyid)];

out = orCommunicator(command_str, 1);
if(strcmp('error',sscanf(out,'%s',1)))
    error('Error orBodyGetDOF');
end

dof = str2double(out);
