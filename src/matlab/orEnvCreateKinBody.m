% bodyid = orEnvCreateKinBody(name, xmlfile)

function bodyid = orEnvCreateKinBody(name, xmlfile)

out = orCommunicator(['createbody ' name ' ' xmlfile], 1);

%if(strcmp('error',sscanf(out,'%s',1)))
%    error('Error creating kinbody');
%end

bodyid = str2double(out);