% values = orBodyGetTransform(bodyid)
%
% Returns the transformations of all the body's first link as a 12 x 1 matrix. 
% (use T=reshape(., [3 4]) to recover).
% T * [X;1] = Xnew

function values = orBodyGetTransform(bodyid)

links = orBodyGetLinks(bodyid);
values = links(:,1);