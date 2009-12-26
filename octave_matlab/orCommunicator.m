% communicates with openrave
% orCommunicator uses the current setting in orConnectionParams
% to read the correct ip and port headers
% readline [optional] - if set, will read the output of the command.
%                       Some commands do not send an output, so the
%                       function can freeze indefinitely.
function [out, sockets] = orCommunicator(command_str, readline)
global orConnectionParams orOpenedSockets

out = [];
if( ~exist('readline','var') )
    readline = 0;
end

sockid = 0;
if( ~isempty(orConnectionParams) )
    index = 1;
    while(index <= length(orOpenedSockets))
        if( strcmp(orConnectionParams.ip, orOpenedSockets{index}.ip) & ...
            orConnectionParams.port == orOpenedSockets{index}.port )
            sockid = orOpenedSockets{index}.sockid;
            break;
        end
        index = index + 1;
    end
    
    if( sockid == 0 )
        % create a new socket
        [ip, port, sockid] = orcreate(orConnectionParams.ip, orConnectionParams.port);
    else
        [ip, port, sockid] = orcreate(sockid, orConnectionParams.ip, orConnectionParams.port);
        if( index > 0 ) 
            % believe that
            ip = orOpenedSockets{index}.ip;
            port = orOpenedSockets{index}.port;
        end
    end
    
    if( sockid == 0 )
        % delete
        if( index <= length(orOpenedSockets) )
            orOpenedSockets(index) = [];
        end
    else
        % succeeded
        sockparams.ip = ip;
        sockparams.port = port;
        sockparams.sockid = sockid;
        sockparams.default = 0;
        orOpenedSockets{index} = sockparams;
    end
else
    % create a default socket

    % first search
    index = 1;
    while(index <= length(orOpenedSockets) )
        if( orOpenedSockets{index}.default)
            sockid = orOpenedSockets{index}.sockid;
            break;
        end
    end

    if( sockid > 0 )
        [ip, port, sockid] = orcreate(sockid);
        orOpenedSockets{index}.sockid = sockid;
        if( sockid == 0 )
            orOpenedSockets(index) = [];
        end
    else
        [ip, port, sockid] = orcreate();

        if( sockid > 0 )
            sockparams.ip = ip;
            sockparams.port = port;
            sockparams.sockid = sockid;
            sockparams.default = 1;

            index = 1;
            found = 0;
            while(index <= length(orOpenedSockets))
                if( strcmp(ip, orOpenedSockets{index}.ip) & ...
                    port == orOpenedSockets{index}.port )
                    orOpenedSockets{index}.sockid = sockid;
                    found = 1;
                end
                index = index + 1;
            end

            if( found == 0 )
                % add
                orOpenedSockets{end+1} = sockparams;
            end
        end
    end
end

if( sockid == 0 )
    error('failed to init connection');
end

orwrite(command_str, sockid);

if( readline )
    out = orread(sockid);
end

sockets = orOpenedSockets;
