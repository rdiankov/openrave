%%  GetLargestFreeConvexPolygon(Imask)
%
% find the biggest area and all the edges around it
% randomly sample the edges and record all convex polygons that pass

% Copyright (C) 2008-2010 Rosen Diankov
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
function [bestconvexhull,bestconvexpoints,bestvolume] = GetLargestFreeConvexPolygon(Imask,savefilename,show)
more off
if( ~exist('show','var') )
    show = 1;
end

Ieroded = imerode(double(Imask==0),ones(3));
L = bwlabel(Ieroded,8);
% set the border to 0
L(:,[1 end]) = 0;
L([1 end],:) = 0;
props = ImageRegionProps(L);
% look for the largest one
[areamax, imax] = max([props.Area]);
% get the boundary points
alledges = bwperim(Ieroded,8);
[rall,call] = ind2sub(size(alledges),find(alledges==1));
edges = bwperim(L==imax,8);
[r,c] = ind2sub(size(edges),find(edges==1));

% subsample the points
convexpoints = PrunePoints(transpose([c r]),100,10^2,1,0);
allpoints = transpose([call rall ones(length(call),1)]);
%insidepoints = [size(Imask,2)/2 size(Imask,2)/2;size(Imask,1)/2 size(Imask,1)*0.7;1 1];
insidepoints = [size(Imask,2)/2;size(Imask,1)/2;1];

%imshow(ones(size(edges))); hold on;
%plot(points(2,:),points(1,:),'kx');

% start the convex polygon sampling loop
bestvolume=0;
bestconvexhull = [];
bestconvexpoints = [];
for iter = 1:20000
    if( mod(iter,1000)==0)
        disp(iter);
    end
    p = randperm(size(convexpoints,2));
    activepoints = convexpoints(:,p(1:8));
    try
        [K, Volume] = convhulln(transpose(activepoints),'Pp');
    catch
        % compute a rough measure of volume
        meanhull = mean(activepoints(:,K(:)),2);
        Volume = 0;
        for i = 1:size(K,1)
            Volume = Volume + 0.5*abs(det(activepoints(:,K(i,:))-[meanhull meanhull]));
        end
    end

    if( isempty(K) ) % octave needs this
        continue;
    end
    
    % get all the plane equations to the convex hull boundary
    meanhull = mean(activepoints,2);
    planes = zeros(3,size(K,1));
    for i = 1:size(K,1)
        % check if 0 is in the center
        ndir = activepoints(:,K(i,1)) - activepoints(:,K(i,2));
        n = [ndir(2);-ndir(1)]/norm(ndir);

        % find correct direction for n
        dist = n'*activepoints(:,K(i,1));
        meandist = n'*meanhull;

        if( meandist < dist )
            n = -n;
            dist = -dist;
        end

        planes(1:2,i) = n;
        planes(3,i) = -dist;
    end

    % check that no points have postive distances to all planes
    pointsinside=0;
    for i = 1:size(allpoints,2)
        if( all(transpose(allpoints(:,i))*planes>5) )
            pointsinside = 1;
            break;
        end
    end

    if( pointsinside == 0 )
        pointsinside = 1;
        for i = 1:size(insidepoints,2)
            if( all(transpose(insidepoints(:,i))*planes>5) == 0 )
                pointsinside = 0;
                break;
            end
        end

        if( pointsinside == 1 && bestvolume < Volume )
            bestvolume = Volume;
            bestconvexhull = planes;
            
            %% have to order the convex points in a fan for easier future processing
            indices = K(1,:);
            K(1,:) = [];
            while(~isempty(K))
                ind = find(K(:,1)==indices(end));
                if(~isempty(ind) )
                    indices(end+1) = K(ind(1),2);
                else
                    ind = find(K(:,2)==indices(end));
                    if(~isempty(ind2) )
                        indices(end+1) = K(ind(1),1);
                    else
                        error('bad convex polygon')
                    end
                end
                K(ind(1),:) = [];
            end
            if( indices(1) ~= indices(end) )
                error('convex ends do not match');
            end
            indices(end) = [];
            bestconvexpoints = activepoints(:,indices);
            disp(sprintf('new volume = %f',bestvolume));
            if( show )
                clf;
                plot(convexpoints(1,:),convexpoints(2,:),'kx'); hold on;
                plot(bestconvexpoints(1,[1:end 1]),bestconvexpoints(2,[1:end 1]),'b');
                plot(insidepoints(1,:),insidepoints(2,:),'rx');
                axis equal;
                drawnow;
            end
            save('-v7','convexpolygon.mat','bestconvexhull','bestconvexpoints','bestvolume','Imask');
        end
    end
end

if( exist('savefilename','var') )
    pts = transpose(bestconvexpoints);
    save('-ascii',savefilename,'pts');
    if( show )
        clf;
        imshow(1-Imask); hold on;
        plot(bestconvexpoints(1,[1:end 1]),bestconvexpoints(2,[1:end 1]),'b');
        plot(convexpoints(1,:),convexpoints(2,:),'rx','MarkerSize',10);
    end
end
