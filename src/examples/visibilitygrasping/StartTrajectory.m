%% success = StartTrajectory(robotid, trajdata)
%%
%% Starts a trajectory on the real robot and waits for it, in the end sets the new robot values
%% in the cloned (current) world.
function success = StartTrajectory(robotid,trajdata,timelimit)
global probs

if( ~exist('timelimit','var') )
    timelimit = 0;
end

out = orProblemSendCommand(['traj stream ' trajdata],probs.manip);

disp('waiting for robot');
success = 1;
dowait = 1;
pause(0.1); % pause a little to give a chance for controller to start

basetime = toc;
while(dowait == 1 & (orEnvWait(robotid, 0.05) == 0) )
    if( timelimit > 0 && toc-basetime > timelimit )
        success = 0;
        break;
    end
end
