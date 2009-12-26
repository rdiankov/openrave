% orEnvClose(figureids)
%
% closes the figures and plots
% figureids - array of ids returned from orEnvPlot or other plotting functions
function orEnvClose(figureids)

if( exist('figureids','var') && ~isempty(figureids) )
    orCommunicator(['close ' sprintf('%d ', figureids(:))]);
else
    orCommunicator('close'); 
end
