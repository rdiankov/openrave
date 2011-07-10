% problemid = orEnvCreateProblem(problemname, args, destroyduplicates)
% 
% Creates an instance of a problem and returns its id for future communicate with it
% problemname - the problem name
% args - a string of arguments to send to the problem's main function
% destroyduplicates [optional] - if 1, will destroy any previous problems with the same problem name.
%                                If 0, will not destroy anything.
%                                The default value is 1. 
function problemid = orEnvCreateProblem(problemname, args, destroyduplicates)

if( ~exist('args', 'var') )
    args = [];
end
if( ~exist('destroyduplicates', 'var') )
    destroyduplicates = 1;
end

out = orCommunicator(['createmodule ' num2str(destroyduplicates) ' ' problemname ' ' args], 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error creating problem');
%end

problemid = str2double(out);
