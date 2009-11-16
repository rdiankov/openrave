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
