Using PlayerController for OpenRAVE
-----------------------------------

At the moment player controller interacts with Player through the ActArray interface. At the moment, the controller is optimized for Player 2.0.4 usage (you can find the sources for player 2.0.4 in the playerfiles folder). In order to use the player controller, several modifications are needed to in order to account for complex trajectory functionality required to move robots. To solve this, player 2.0.4 needs to be patched up. In the playerfiles directory, there is a python script that will do the necessary patching.

In order to use it, first cd playerfiles, then

python patchplayer.py player_directory


You can find more information about the entire installation process on the OpenRAVE wiki. Specifically go to

http://openrave.programmingvision.com/index.php?title=Misc:PlayerController


for questions contact Rosen Diankov (rdiankov@cs.cmu.edu)
