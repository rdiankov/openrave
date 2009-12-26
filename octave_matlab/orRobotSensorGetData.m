% data = orRobotSensorGetData(robotid, sensorindex)
%
% Gets the sensor data. The format returned is dependent on the type
% of sensor. Look at the different data SensorData implementations in rave.h.
% Although the data returned is not necessarily one of them.
% options [optional] - options that specify what type of data to request (0 is default)
% data.type - contains the id of the data type (see SensorBase::SensorType)
% For laser data
%  data.laserrange - 3xN array where each column is the direction * distance
%  data.laserpos - 3xN array where each column is the corresponding origin of each range measurement
%  data.laserint - 1xN optional laser intensity array
% For image data
%  data.KK - 3x3 intrinsic matrix
%  data.T - 3x4 camera matrix (to project a point multiply by KK*inv(T))
%  data.P - 3x4 camera projection matrix
%  data.I - the rgb image size(I) = [height width 3]
function data = orRobotSensorGetData(robotid, sensorindex, options)

if( ~exist('options','var') )
    options = 0;
end

data = [];
sout = orCommunicator(['robot_sensordata ' sprintf('%d ', robotid, sensorindex, options)], 1);
values = sscanf(sout, '%f');

if( length(values) == 0 )
    return;
end

data.type = values(1);
if( data.type == 1 )
    % laser data
    Nrange = values(2);
    Npos = values(3);
    Nint = values(4);
    off = 4;
    data.laserrange = reshape(values(off + (1:(3*Nrange))),[3 Nrange]);
    off = off + 3*Nrange;
    data.laserpos = reshape(values(off + (1:(3*Npos))),[3 Npos]);
    if( Npos ~= Nrange )
        data.laserpos = repmat(data.laserpos(:,1),[1 Nrange]); % repeat
    end
    off = off + 3*Npos;

    if( Nint > 0 )
         % intensity
         data.laserint = reshape(values(off + (1:Nint)),[1 Nint]);
         off = off + 3*Nint;
    end
elseif( data.type == 2 )
    width = values(2);
    height = values(3);
    data.KK = [values(4) 0 values(6); 0 values(5) values(7); 0 0 1];
    data.T = reshape(values(8:19),[3 4]);
    Tinv = inv([data.T; 0 0 0 1]);
    data.P = data.KK*Tinv(1:3,:);
    
    %data.I = permute(reshape(values(20:(19+width*height*3)), [3 width height]), [3 2 1])/255;
        
    % decode the RLE
    off = 20;
    indssize = values(off);
    imvalues = values(off+(1:indssize));
    diffs = values(off+indssize+1+(1:values(off+indssize+1)));

    inds = cumsum([ 1; diffs ]);
    J = zeros(1, inds(end)-1);
    J(inds(1:end-1)) = 1;
    I = transpose(reshape(imvalues(cumsum(J)), [width height]));
    data.I(:,:,1) = bitand(I,255);
    data.I(:,:,2) = bitand(floor(I/256),255);
    data.I(:,:,3) = bitand(floor(I/256^2),255);
    data.I = data.I/255;
else
    data.I = [];
end
