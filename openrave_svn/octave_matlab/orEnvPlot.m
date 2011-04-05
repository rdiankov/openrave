% figureid = orEnvPlot(points,...)
%
% plots points or lines in the openrave viewer
% points - Nx3 vector of xyz positions
% optional arguments include 'size', 'color', and 'line'
%   color - Nx3 vector of RGB values between 0 and 1
%   size - Nx1 vector of the sizes in pixels of each point/line
%   line (or linestrip) - if specified, then openrave renders a line strip
%   linelist - if specified, openrave renders a line for every two points
%   trilist - if specified, openrave renders a triangle for every three
%             vertices should be specified in counter-clockwise order
%   sphere - if specified, openrave renders each point as a sphere
%   transparency - [0,1], set transparency of plotted objects (0 is opaque)
function figureid = orEnvPlot(varargin)

points = varargin{1}';
numpoints = size(points,2);
command_str = ['plot ' num2str(numpoints) ' ' sprintf('%.3f ', points(:)')];

pointsize = 0.5;
colors = [];
drawstyle = 0;
transparency = 0;

i = 2;
while(i <= nargin)
    if( strcmp(varargin{i},'size') )
        i = i + 1;
        pointsize = varargin{i};
    elseif( strcmp(varargin{i},'color') )
        i = i + 1;
        colors = varargin{i};
    elseif( strcmp(varargin{i},'line') | strcmp(varargin{i},'linestrip') )
        drawstyle = 1;
    elseif( strcmp(varargin{i},'linelist') )
        drawstyle = 2;
    elseif( strcmp(varargin{i},'sphere') )
        drawstyle = 3;
    elseif( strcmp(varargin{i},'trilist') )
        drawstyle = 4;
    elseif( strcmp(varargin{i},'transparency') )
        i = i + 1;
        transparency = varargin{i};
    end
    
    i = i + 1;
end

out = orCommunicator([command_str ' ' num2str(size(colors,1)) ' ' sprintf('%.3f ', colors') ...
                    ' ' sprintf('%.3f ', pointsize) ' ' num2str(drawstyle) ' ' num2str(transparency)],1);
if(strcmp('error',sscanf(out,'%s',1)))
    error(['Unknown robot id: ' num2str(robotid)]);
end

figureid = str2double(out);
