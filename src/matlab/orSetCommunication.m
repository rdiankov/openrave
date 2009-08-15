% orSetCommunication(ip,port)
%
% Sets the type of communication to use when sending/receiving OpenRAVE commands.
% Arguments:
%   ip - ip address of computer. If empty, starts internal/in-memory OpenRAVE instance
%   port - If ip address is vaild, the port to communicate. If ip address was empty, the
%          OpenRAVE id to communicate with. If id does not exist, creates an OpenRAVE instance.
function orSetCommunication(ip,port)
global orConnectionParams

if( ~isempty(ip) )
    orConnectionParams.port = port;
    orConnectionParams.ip = ip;
else
    error('not implemented yet');
end
