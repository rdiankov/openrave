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
function props = ImageRegionProps(L)

ids = unique(L(:));
ids(ids==0) = [];
props = cell(2, length(ids));
for i = 1:length(ids)
    [r,c] = ind2sub(size(L),find(L==ids(i)));
    props{1,i} = length(r);
    ul = [min(c) min(r)]-1;
    props{2,i} = [ul max(c)-ul(1) max(r)-ul(2)];
end

props = cell2struct(props, {'Area','BoundingBox'}, 1);
