% orEnvDestroyProblem(problemid)
%
% Destroys problem instance whose id is problemid.

function orEnvDestroyProblem(problemid)

out = orCommunicator(['env_dstrprob ' num2str(problemid)]);
