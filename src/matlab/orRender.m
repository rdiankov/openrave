% orRender(cmd)
%
% Controls rendering properties. Cmd can be
% start - starts the GUI to update the internal openrave state
% stop - stops the GUI from updating the internal openrave state (can be used to speed up loading)
function [] = orRender(cmd)

out = orCommunicator(['render ' cmd]);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error rendering');
%end
