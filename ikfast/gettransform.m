function gettransform(robotid,joints)
more off
manips = orRobotGetManipulators(robotid);

orBodySetJointValues(robotid,joints,manips{1}.armjoints);
links = orBodyGetLinks(robotid);
Thand = reshape(links(:,manips{1}.eelink+1),[3 4]);
Thand_frombase = inv([reshape(links(:,manips{1}.baselink+1),[3 4]);0 0 0 1]) * [Thand; 0 0 0 1];

display(['transform: ' sprintf('%f, ',transpose(Thand_frombase))])

display(['transform: ' sprintf('%f ',transpose(Thand_frombase(1:3,1:4))) sprintf('%f ',joints(1))])

