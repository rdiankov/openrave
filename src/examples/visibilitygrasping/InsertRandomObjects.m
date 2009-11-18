% dests = InsertRandomObjects(randomize,tableid,avoidbodyids,targetid)

% Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
function dests = InsertRandomObjects(randomize,tableid,avoidbodyids,targetid)

for id = avoidbodyids
    if( orEnvCheckCollision (id) )
        error('in collision with id %d',id);
    end
end

Tidentity = eye(4);
Ttable = reshape(orBodyGetTransform(tableid),[3 4]);
orBodySetTransform(tableid,Tidentity(1:3,1:4));
ab = orBodyGetAABB(tableid);
orBodySetTransform(tableid,Ttable);

%% table up is assumed to be +z, sample the +y axis of the table
Nx = floor(2*ab(1,2)/0.1);
Ny = floor(2*ab(2,2)/0.1);
X = [];
Y = [];
for x = 0:(Nx-1)
    X = [X 0.5*rand(1,Ny)/(Nx+1) + (x+1)/(Nx+1)];
    Y = [Y 0.5*rand(1,Ny)/(Ny+1) + ([0:(Ny-1)]+0.5)/(Ny+1)];
end

offset = [ab(1,1)-ab(1,2);ab(2,1)-ab(2,2); ab(3,1)+ab(3,2)];
trans = [offset(1)+2*ab(1,2)*X; offset(2)+2*ab(2,2)*Y; repmat(offset(3),size(X))];

%% for every destination try inserting a box
if( ~isempty(randomize) )
    numcreated = 0;
    for i = 1:size(trans,2)
        if( rand() < randomize.createprob )
            iobs = floor(rand()*length(randomize.obstacles))+1;
            bodyid = orEnvCreateKinBody(sprintf('obstacle%d',numcreated),randomize.obstacles{iobs});
            if( isempty(bodyid) )
                disp(sprintf('invalid body %s',randomize.obstacles{iobs}));
                continue;
            end

            angs = 0:pi/3:2*pi/3;
            angs = angs(randperm(length(angs)));
            success = 0;
            for roll = angs
                T = Ttable*[RotationMatrixFromAxisAngle([0;0;roll]) trans(:,i); 0 0 0 1];
                orBodySetTransform(bodyid,T);
                success = 1;
                for id = avoidbodyids
                    if( orEnvCheckCollision(id) )
                        success = 0;
                        break;
                    end
                end

                if(success)
                    numcreated = numcreated+1;
                    break;
                end
            end

            if( ~success )
                orBodyDestroy(bodyid);
            end
            if( numcreated >= randomize.maxcreate )
                break;
            end
        end
    end
end

%% find all dests not in collision
dests = [];
if( exist('targetid','var') && ~isempty(targetid) )
    Torig = orBodyGetTransform(targetid);
    numcreated = 0;
    dists = [];
    for i = 1:size(trans,2);
        for roll = 0:pi/4:pi
            T = Ttable*[RotationMatrixFromAxisAngle([0;0;roll]) [trans(1:2,i);0]; 0 0 0 1];
            T(3,4) = Torig(12);
            orBodySetTransform(targetid,T);
            if( ~orEnvCheckCollision(targetid) )
                dests = [dests T(:)];
            end
        end
    end
    orBodySetTransform(targetid,Torig);
end
