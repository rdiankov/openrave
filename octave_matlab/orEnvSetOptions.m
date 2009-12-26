% orEnvSetOptions(options)
%
% A string of various environment options. Example usage:
% orEnvSetOptions('publishanytime 1');
%
% Current options:
% - simulation [start/stop] [time_step] - toggles the internal simulation loop, ie all the calls to SimulationStep. If time_step is specified, will set the simulation time step for all objects. Note that this is not tied to real time at all, how fast the simulation goes in reality depends on complexity of the scene and the physics engine being used.
% - physics engine_name - switches the physics engine to another one with id 'engine_name'
% - collision checker_name - switches the collision checker to a new one with id 'checker_name'
% - gravity [x y z] - changes to gravity vector
% - selfcollision [on/off] - sets the self collision settings for the current physics engine
% - publishanytime [1/0] - switch between publishing the body transformations
%           to the GUI anytime or only between stepsimulation and server  messsages.
%           When publishing anytime, the GUI will reflect the body movements after every
%           move. This is useful when visualizing internal C++ states. When off, the GUI
%           will only reflect the state of robots after all calls to stepsimulation and
%           server send messages have been done. The default is off.
% - debug [debug level] - toggles debugging messages by RAVELOG.
%                         0  - only RAVEPRINT statements show
%                         1+ - RAVELOG statements with various debug levels show
% - quit - closes the openrave instance
function [] = orEnvSetOptions(options)

out = orCommunicator(['setoptions ' options]);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error setting options');
%end

