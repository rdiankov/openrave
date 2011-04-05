%% success = orEnvStepSimulation(timestep,sync)
%%
%% advances the physics simulation by timestep seconds
%% timestep - seconds to run the simulation for (internal physics engine call)
%% sync - if 0 function will return immediately, if 1 function will wait until step simulation ends
function orEnvStepSimulation(timestep,sync)
if( ~exist('sync', 'var') )
    sync = 1;
end
out = orCommunicator(['env_stepsimulation ' num2str(timestep) ' ' num2str(sync)]);
