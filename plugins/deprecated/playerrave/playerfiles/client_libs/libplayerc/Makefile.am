SUBDIRS = . test bindings doc

AM_CPPFLAGS = -Wall -I$(top_srcdir) -g

# TODO: figure out how to handle the jpeg wrappers
# -I$(top_srcdir)/client_libs/libplayerpacket

# create the pkg-config entry
pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = playerc.pc

EXTRA_DIST = playerc.pc.in
CLEANFILES = playerc.pc

lib_LTLIBRARIES = libplayerc.la
libplayerc_la_LDFLAGS = $(PLAYER_VERSION_INFO) -rpath $(libdir) $(top_builddir)/libplayerxdr/libplayerxdr.la
libplayerc_la_DEPENDENCIES = $(top_builddir)/libplayerxdr/libplayerxdr.la

libplayerc_la_SOURCES = playerc.h \
                        mclient.c \
                        client.c \
                        device.c \
                        utils.c \
                        error.c \
                        error.h \
                        dev_actarray.c \
                        dev_aio.c \
                        dev_bumper.c \
                        dev_blobfinder.c \
                        dev_camera.c \
                        dev_dio.c \
                        dev_fiducial.c \
                        dev_gps.c \
                        dev_graphics2d.c \
                        dev_graphics3d.c \
                        dev_gripper.c \
                        dev_ir.c \
                        dev_laser.c \
                        dev_limb.c \
                        dev_localize.c \
                        dev_log.c \
                        dev_map.c \
                        dev_opaque.c \
                        dev_planner.c \
                        dev_position1d.c \
                        dev_position2d.c \
                        dev_position3d.c \
                        dev_power.c \
                        dev_ptz.c \
                        dev_rfid.c \
                        dev_simulation.c \
                        dev_sonar.c \
                        dev_speech.c \
                        dev_speech_recognition.c \
						dev_visionserver.c \
                        dev_wifi.c \
			dev_wsn.c

#dev_joystick.c
#dev_truth.c
#dev_motor.c

#

libplayerc_la_LIBADD = $(top_builddir)/replace/*.lo
if INCLUDE_JPEG
libplayerc_la_LIBADD += $(top_builddir)/libplayerjpeg/*.lo
endif

libplayerc_includedir = $(includedir)/player-2.0/libplayerc
libplayerc_include_HEADERS = playerc.h
