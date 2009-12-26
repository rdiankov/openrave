% orEnvLoadScene(filename, [ClearScene])
%
% Loads a new environment.
% filename - The filename of the scene to load. If a relative file
%            is specified, note that it is relative to the current direction
%            of the OpenRAVE executable.
% ClearScene - If 1, then clears the scene before loading. Else leaves the
%              scene alone and loads in addition to it.

function [] = orEnvLoadScene(filename, ClearScene)

if( ~exist('ClearScene', 'var') )
    ClearScene = 0;
end

orCommunicator(['loadscene ' filename ' ' num2str(ClearScene)],1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error loading scene');
%end

