% Gpruned = PrunePoints(G, nsize, thresh2,neighsize,ispose)
%
% Prunes region of the grasp set that are too close
% nsize - final grasp set size
% thresh2 - pruning threshold squared (higher reduces grasp set)

% Copyright (C) 2008-2009 Rosen Diankov
% All rights reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
% 1. Redistributions of source code must retain the above copyright
%   notice, this list of conditions and the following disclaimer.
% 2. Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in the
%    documentation and/or other materials provided with the distribution.
% 3. The name of the author may not be used to endorse or promote products
%   derived from this software without specific prior written permission.
% THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
% IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
% OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
% IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
% NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
% DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
% THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
% THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
function Gpruned = PrunePoints(G, nsize, thresh2,neighsize,ispose)

if( ~exist('neighsize','var') )
    neighsize = 1;
end

if( ~exist('ispose','var') )
    ispose = 1;
end

if( ispose )
    W = [0.2;0.2;0.2;0.2;1;1;1];
else
    W = ones(size(G,1),1);
end

iter = 1;
while(size(G,2)>nsize)
    ind = ceil(size(G,2).*rand);
    g = G(:,ind);
    d = W'*(G - repmat(g, [1 size(G,2)])).^2;

    if( ispose )
        g2 = [-g(1:4);g(5:7)];
        % check g's neighbors
        d2 = W'*(G - repmat(g2, [1 size(G,2)])).^2;
        neigh = sum(d < thresh2 | d2 < thresh2);
    else
        neigh = sum(d < thresh2);
    end

    if( neigh > neighsize )
        G(:,ind) = [];
        %size(G,2)
    end
    
    iter = iter+1;
    if( iter > 5000 )
        break;
    end
end

length(G)
Gpruned = G;
