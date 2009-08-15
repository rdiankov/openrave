function setvalues(robotid, values)
    limits = orRobotGetDOFLimits(robotid);
if( any(values'<limits(:,1)|values'>limits(:,2)) )
    display('bad limits')
    return
end
orBodySetJointValues(robotid,values,0:(length(values)-1))
