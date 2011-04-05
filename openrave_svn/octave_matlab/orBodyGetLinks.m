% values = orBodyGetLinks(bodyid)
%
% Returns the transformations of all the body's links in a 12 x L matrix. Where L
% is the number of links and each column is a 3x4 transformation
% (use T=reshape(., [3 4]) to recover).
% T * [X;1] = Xnew

function values = orBodyGetLinks(bodyid)

command_str = ['body_getlinks ' num2str(bodyid)];

out = orCommunicator(command_str, 1);
if(strcmp('error',sscanf(out,'%s',1)))
    error('Error orBodyGetLinks');
end

values = str2num(out);
values = reshape(values,12,size(values,2)/12);