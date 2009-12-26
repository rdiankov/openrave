% output = orProblemSendCommand(cmd, problemid, envlock,dosync)
%
% Sends a command to the problem. The function doesn't return until
% ProblemInstance::SendCommand returns.
% cmd - the string command to send the problem
% problemid - returned id of the problem, if not specified, then
%                        command is sent to all problems
% envlock - if non zero, locks the environment before calling SendCommand (default is 0)
% dosync [optional] - If 1, the SendCommand is called in the main thread, in sync
%                        with the rest of the primitives. If 0, called in a different thread.
% output - the concatenated output of all the problems that the command is sent to
function output = orProblemSendCommand(cmd, problemid, envlock, dosync)

if( ~exist('problemid','var') )
    %display('orProblemSendCommand: please specify problemid');
    problemid = 0;
elseif( isempty(problemid) )
    problemid = 0;
end

if( ~exist('dosync','var') )
    dosync = 0;
end
if( ~exist('envlock','var') )
    envlock = 0;
end

output = orCommunicator(['problem_sendcmd ' num2str(problemid) ' ' num2str(dosync) ' ' num2str(envlock) ' ' cmd], 1);
