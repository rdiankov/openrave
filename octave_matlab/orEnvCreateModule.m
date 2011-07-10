% moduleid = orEnvCreateModule(modulename, args, destroyduplicates)
% 
% Creates an instance of a module and returns its id for future communicate with it
% modulename - the module name
% args - a string of arguments to send to the module's main function
% destroyduplicates [optional] - if 1, will destroy any previous modules with the same module name.
%                                If 0, will not destroy anything.
%                                The default value is 1. 
function moduleid = orEnvCreateModule(modulename, args, destroyduplicates)

if( ~exist('args', 'var') )
    args = [];
end
if( ~exist('destroyduplicates', 'var') )
    destroyduplicates = 1;
end

out = orCommunicator(['createmodule ' num2str(destroyduplicates) ' ' modulename ' ' args], 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error creating module');
%end

moduleid = str2double(out);
