%% Simple demo to test the physics capabilities of OpenRAVE.
%% Create a scene and randomly create objects in it.
more off; % turn off output paging
addopenravepaths()

orEnvLoadScene('data/hanoi.env.xml',1); % load a scene
orEnvSetOptions('physics ode'); % start the physics engine
orEnvSetOptions('gravity 0 0 -9.8'); % set gravity


bodylist = {'data/lego2.kinbody.xml', 'data/lego4.kinbody.xml', 'data/mug1.kinbody.xml'};
index = 1;
while(1)
    % continually create random kinbodies
    irand = ceil(rand(1)*length(bodylist));
    bodyid = orEnvCreateKinBody(sprintf('mybody%d', index), bodylist{irand});
    orBodySetTransform(bodyid, [-0.5;-0.5;2]+0.4*rand(3,1));
    index = index + 1;
    pause(0.2);
end

% use to stop the simulation
%orEnvSetOptions('simulation stop');
