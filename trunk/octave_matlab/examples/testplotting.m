%% scripts tests plotting functions
more off; % turn off output paging
addopenravepaths();

orEnvClose(); % close all plots
orEnvPlot([-1.5 -1 0],'color',[1 0 0],'size',5);
orEnvPlot([-1.5 -0.5 0; -1.5 0.5 0],'color',[0 1 0; 0 0 0],'size',15);
orEnvPlot([-1.25 -0.5 0; -1.25 0.5 0; -1.5 1 0],'color',[0 1 0; 0 0 1; 1 0 0],'size',3,'line');
orEnvPlot([-0.5 -0.5 0; -0.5 0.5 0],'color',[0 1 0; 1 1 0],'size',0.05,'sphere');

orEnvPlot([0 0 0; 0.5 0 0; 0 0.5 0],'color',[0 1 0; 0 0 1; 1 0 0],'trilist');
orEnvPlot([0 0 0.5; 0.5 0 0.5; 0 0.5 0.5; 0.5 0 0.5; 0.5 0.5 0.5; 0 0.5 0.5],...
          'color',[1 0 0],'transparency',0.5,'trilist');
