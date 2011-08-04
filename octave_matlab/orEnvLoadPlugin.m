% success = orEnvLoadPlugin(filename)
%
% Loads a plugin.
% filename - the relative path of the plugin to load. (*.so for linux, *.dll for windows)

function success = orEnvLoadPlugin(filename, ClearScene)

out = orCommunicator(['env_loadplugin ' filename], 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error destroying body');
%end

success = str2num(out);
