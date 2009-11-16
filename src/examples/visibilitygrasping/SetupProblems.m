function SetupProblems(robotname)
global probs
probs.task = orEnvCreateProblem('TaskManipulation',robotname);
if( isempty(probs.task) )
    error('failed to create TaskManipulation problem');
end

probs.manip = orEnvCreateProblem('BaseManipulation',robotname);
if( isempty(probs.manip) )
    error('failed to create BaseManipulation problem');
end

probs.visual = orEnvCreateProblem('VisualFeedback',robotname);
if( isempty(probs.visual) )
    error('failed to create BaseManipulation problem');
end
