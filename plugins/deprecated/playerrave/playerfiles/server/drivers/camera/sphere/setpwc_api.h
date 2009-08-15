/* The GPL applies to this program.
  In addition, as a special exception, the copyright holders give
  permission to link the code of portions of this program with the
  OpenSSL library under certain conditions as described in each
  individual source file, and distribute linked combinations
  including the two.
  You must obey the GNU General Public License in all respects
  for all of the code used other than OpenSSL.  If you modify
  file(s) with this exception, you may extend this exception to your
  version of the file(s), but you are not obligated to do so.  If you
  do not wish to do so, delete this exception statement from your
  version.  If you delete this exception statement from all source
  files in the program, then also delete it here.
*/

#ifndef SETPWC_FUNCTIONS
#define SETPWC_FUNCTIONS

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#define _LINUX_TIME_H 1 /* to get things compile on kernel 2.6.x */
#include <linux/videodev.h>

#include "pwc-ioctl.h"

const int SET_PAN  = 0;
const int SET_TILT = 1;

const int GET_PAN  = 0;
const int GET_TILT = 1;

void error_exit(char *what_ioctl);

void check_video_device(int *fd);

void not_supported(char *what);

void dump_current_settings(int fd);

void query_pan_tilt_status(int fd, int *status);

void reset_pan_tilt(int fd, int what);

void get_pan_or_tilt_limits(int fd, char what, int *min, int *max);

void set_pan_or_tilt(int fd, char what, int value);

void set_pan_and_tilt(int fd, int pan, int tilt);

void set_framerate(int fd, int framerate);

void flash_settings(int fd);

void restore_settings(int fd);

void restore_factory_settings(int fd);

void set_compression_preference(int fd, int pref);

void set_automatic_gain_control(int fd, int pref);

void set_shutter_speed(int fd, int pref);

void set_automatic_white_balance_mode(int fd, char *mode);

void set_automatic_white_balance_mode_red(int fd, int val);

void set_automatic_white_balance_mode_blue(int fd, int val);

void set_automatic_white_balance_speed(int fd, int val);

void set_automatic_white_balance_delay(int fd, int val);

void set_led_on_time(int fd, int val);

void set_led_off_time(int fd, int val);

void set_sharpness(int fd, int val);

void set_backlight_compensation(int fd, int val);

void set_antiflicker_mode(int fd, int val);

void set_noise_reduction(int fd, int val);

#endif
