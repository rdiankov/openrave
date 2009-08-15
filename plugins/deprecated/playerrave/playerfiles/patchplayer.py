#! /usr/bin/python

# Program patches the player directory with symbolic links
import os, sys

names = ["/client_libs/libplayerc/client.c",
         "/client_libs/libplayerc/dev_actarray.c",
         "/client_libs/libplayerc/dev_visionobject.c",
         "/client_libs/libplayerc/dev_visionserver.c",
         "/client_libs/libplayerc/Makefile.am",
         "/client_libs/libplayerc/playerc.h",
         "/client_libs/libplayerc++/actarrayproxy.cc",
         "/client_libs/libplayerc++/Makefile.am",
         "/client_libs/libplayerc++/playerc++.h",
         "/client_libs/libplayerc++/playerclient.cc",
         "/client_libs/libplayerc++/playerclient.h",
         "/client_libs/libplayerc++/visionobjectproxy.cc",
         "/client_libs/libplayerc++/visionserverproxy.cc",
         "/libplayercore/interface_util.cc",
         "/libplayercore/player.h",
         "/libplayerxdr/functiontable.c",
         "/server/drivers/mixed/rmp/canio.h",
         "/server/drivers/mixed/rmp/canio_rmpusb.h",
         "/server/drivers/mixed/rmp/canio_rmpusb.cc",
         "/server/drivers/mixed/rmp/segwayrmp.h",
         "/server/drivers/mixed/rmp/segwayrmp.cc",
         "/server/drivers/mixed/rmp/test_usb.cc",
         "/server/drivers/camera/sphere/setpwc_api.c",
         "/server/drivers/camera/sphere/setpwc_api.h"
         ];

base=os.getcwd();
for fullname in names :
    os.system("rm -f " + sys.argv[1] + fullname);
    os.system("ln -s " + base + fullname + " " + sys.argv[1] + fullname);
