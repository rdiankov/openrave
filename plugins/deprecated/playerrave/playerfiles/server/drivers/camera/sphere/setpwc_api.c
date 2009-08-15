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

#include "setpwc_api.h"

void error_exit(char *what_ioctl)
{
  fprintf(stderr, "Error while doing ioctl %s: %s\n", what_ioctl, strerror(errno));

  /* commented out: some versions of the driver seem to return
   * unexpected errors */
  /* exit(1); */
}

void check_video_device(int *fd)
{
  char *device = "/dev/video0";

  if (*fd == -1)
  {
    /* open device */
    *fd = open(device, O_RDWR);
    if (*fd == -1)
    {
      fprintf(stderr, "Error while accessing device %s: %s\n", device, strerror(errno));
      exit(1);
    }
  }
}

void not_supported(char *what)
{
  printf("%s is not supported by the combination\n", what);
  printf("of your webcam and the driver.\n");
}

void dump_current_settings(int fd)
{
  struct video_capability vcap;
  struct video_window vwin;
  struct video_picture vpic;
  struct pwc_probe pwcp;
  int dummy;
  struct pwc_whitebalance pwcwb;
  struct pwc_leds pwcl;
  struct pwc_mpt_range pmr;
  struct pwc_mpt_angles pma;

  /* get name */
  if (ioctl(fd, VIDIOCGCAP, &vcap) == -1)
    error_exit("VIDIOCGCAP");
  printf("Current device: %s\n", vcap.name);

  /* verify that it IS a Philips Webcam */
  if (ioctl(fd, VIDIOCPWCPROBE, &pwcp) == -1)
    error_exit("VIDIOCPWCPROBE");
  if (strcmp(vcap.name, pwcp.name) != 0)
    printf("Warning: this might not be a Philips compatible webcam!\n");
  printf("VIDIOCPWCPROBE returns: %s - %d\n", pwcp.name, pwcp.type);

  /* get resolution/framerate */
  if (ioctl(fd, VIDIOCGWIN, &vwin) == -1)
    error_exit("VIDIOCGWIN");
  printf("Resolution (x, y): %d, %d\n", vwin.width, vwin.height);
  printf("Offset: %d, %d\n", vwin.x, vwin.y);
  if (vwin.flags & PWC_FPS_FRMASK)
    printf("Framerate: %d\n", (vwin.flags & PWC_FPS_FRMASK) >> PWC_FPS_SHIFT);

  /* color (etc.) settings */
  if (ioctl(fd, VIDIOCGPICT, &vpic) == -1)
    error_exit("VIDIOCGPICT");
  printf("Brightness: %d\n", vpic.brightness);
  printf("Hue: %d\n", vpic.hue);
  printf("Colour: %d\n", vpic.colour);
  printf("Contrast: %d\n", vpic.contrast);
  printf("Whiteness: %d\n", vpic.whiteness);
  printf("Palette: ");
  switch(vpic.palette) {
  case VIDEO_PALETTE_GREY:
    printf("Linear intensity grey scale (255 is brightest).\n");
    break;
  case VIDEO_PALETTE_HI240:
    printf("The BT848 8bit colour cube.\n");
    break;
  case VIDEO_PALETTE_RGB565:
    printf("RGB565 packed into 16 bit words.\n");
    break;
  case VIDEO_PALETTE_RGB555:
    printf("RGV555 packed into 16 bit words, top bit undefined.\n");
    break;
  case VIDEO_PALETTE_RGB24:
    printf("RGB888 packed into 24bit words.\n");
    break;
  case VIDEO_PALETTE_RGB32:
    printf("RGB888 packed into the low 3 bytes of 32bit words. The top 8bits are undefined.\n");
    break;
  case VIDEO_PALETTE_YUV422:
    printf("Video style YUV422 - 8bits packed 4bits Y 2bits U 2bits V\n");
    break;
  case VIDEO_PALETTE_YUYV:
    printf("Describe me\n");
    break;
  case VIDEO_PALETTE_UYVY:
    printf("Describe me\n");
    break;
  case VIDEO_PALETTE_YUV420:
    printf("YUV420 capture\n");
    break;
  case VIDEO_PALETTE_YUV411:
    printf("YUV411 capture\n");
    break;
  case VIDEO_PALETTE_RAW:
    printf("RAW capture (BT848)\n");
    break;
  case VIDEO_PALETTE_YUV422P:
    printf("YUV 4:2:2 Planar\n");
    break;
  case VIDEO_PALETTE_YUV411P:
    printf("YUV 4:1:1 Planar\n");
    break;
  case VIDEO_PALETTE_YUV420P:
    printf("YUV 4:2:0 Planar\n");
    break;
  case VIDEO_PALETTE_YUV410P:
    printf("YUV 4:1:0 Planar\n");
    break;
  default:
    printf("Unknown! (%d)\n", vpic.palette);
  }

  if (ioctl(fd, VIDIOCPWCGCQUAL, &dummy) == -1)
    error_exit("VIDIOCPWCGCQUAL");
  printf("Compression preference: %d\n", dummy);

  if (ioctl(fd, VIDIOCPWCGAGC, &dummy) == -1)
    error_exit("VIDIOCPWCGAGC");
  printf("Automatic gain control: %d\n", dummy);

  if (ioctl(fd, VIDIOCPWCGAWB, &pwcwb) == -1)
    error_exit("VIDIOCPWCGAWB");
  printf("Whitebalance mode: ");
  if (pwcwb.mode == PWC_WB_AUTO)
    printf("auto\n");
  else if (pwcwb.mode == PWC_WB_MANUAL)
    printf("manual (red: %d, blue: %d)\n", pwcwb.manual_red, pwcwb.manual_blue);
  else if (pwcwb.mode == PWC_WB_INDOOR)
    printf("indoor\n");
  else if (pwcwb.mode == PWC_WB_OUTDOOR)
    printf("outdoor\n");
  else if (pwcwb.mode == PWC_WB_FL)
    printf("artificial lightning ('fl')\n");
  else
    printf("unknown!\n");

  if (ioctl(fd, VIDIOCPWCGLED, &pwcl) != -1)
  {
    printf("Led ON time: %d\n", pwcl.led_on);
    printf("Led OFF time: %d\n", pwcl.led_off);
  }
  else
  {
    not_supported("Blinking of LED");
  }

  if (ioctl(fd, VIDIOCPWCGCONTOUR, &dummy) == -1)
    error_exit("VIDIOCPWCGCONTOUR");
  printf("Sharpness: %d\n", dummy);

  if (ioctl(fd, VIDIOCPWCGBACKLIGHT, &dummy) == -1)
    error_exit("VIDIOCPWCGBACKLIGHT");
  printf("Backlight compensation mode: ");
  if (dummy == 0) printf("off\n"); else printf("on\n");

  if (ioctl(fd, VIDIOCPWCGFLICKER, &dummy) != -1)
  {
    printf("Anti-flicker mode: ");
    if (dummy == 0) printf("off\n"); else printf("on\n");
  }
  else
  {
    not_supported("Anti-flicker mode");
  }

  if (ioctl(fd, VIDIOCPWCGDYNNOISE, &dummy) != -1)
  {
    printf("Noise reduction mode: %d ", dummy);
    if (dummy == 0) printf("(none)");
    else if (dummy == 3) printf("(high)");
    printf("\n");
  }
  else
  {
    not_supported("Noise reduction mode");
  }

  if (ioctl(fd, VIDIOCPWCMPTGRANGE, &pmr) == -1)
  {
    not_supported("Pan/tilt range");
  }
  else
  {
    printf("Pan min. : %d, max.: %d\n", pmr.pan_min, pmr.pan_max);
    printf("Tilt min.: %d, max.: %d\n", pmr.tilt_min, pmr.tilt_max);
  }

  pma.absolute=1;
  if (ioctl(fd, VIDIOCPWCMPTGANGLE, &pma) == -1)
  {
    not_supported("Get pan/tilt position");
  }
  else
  {
    printf("Pan  (degrees * 100): %d\n", pma.pan);
    printf("Tilt (degrees * 100): %d\n", pma.tilt);
  }
}


void query_pan_tilt_status(int fd, int *status)
{
  struct pwc_mpt_status pms;

  if (ioctl(fd, VIDIOCPWCMPTSTATUS, &pms) == -1)
    error_exit("VIDIOCPWCMPTSTATUS");

  *status = pms.status;
}

void reset_pan_tilt(int fd, int what)
{
  if (ioctl(fd, VIDIOCPWCMPTRESET, &what) == -1)
    error_exit("VIDIOCPWCMPTRESET");
}

void set_pan_or_tilt(int fd, char what, int value)
{
  struct pwc_mpt_angles pma;

  pma.absolute=1;
  if (ioctl(fd, VIDIOCPWCMPTGANGLE, &pma) == -1)
    error_exit("VIDIOCPWCMPTGANGLE");

  if (what == SET_PAN)
    pma.pan = value;
  else if (what == SET_TILT)
    pma.tilt = value;

  if (ioctl(fd, VIDIOCPWCMPTSANGLE, &pma) == -1)
    error_exit("VIDIOCPWCMPTSANGLE");
}

void set_pan_and_tilt(int fd, int pan, int tilt)
{
  struct pwc_mpt_angles pma;

  pma.absolute=1;
  if (ioctl(fd, VIDIOCPWCMPTGANGLE, &pma) == -1)
    error_exit("VIDIOCPWCMPTGANGLE");

  pma.pan = pan;
  pma.tilt = tilt;

  if (ioctl(fd, VIDIOCPWCMPTSANGLE, &pma) == -1)
    error_exit("VIDIOCPWCMPTSANGLE");
}

void get_pan_or_tilt_limits(int fd, char what, int *min, int *max)
{
  struct pwc_mpt_range pmr;

  if (ioctl(fd, VIDIOCPWCMPTGRANGE, &pmr) == -1)
  {
    error_exit("VIDIOCPWCMPTGRANGE");
  }
  else
  {
    if (what == GET_PAN)
    {
      *min = pmr.pan_min;
      *max = pmr.pan_max;
    }
    else
    {
      *min = pmr.tilt_min;
      *max = pmr.tilt_max;
    }
  }

}


void set_framerate(int fd, int framerate)
{
  struct video_window vwin;

  /* get resolution/framerate */
  if (ioctl(fd, VIDIOCGWIN, &vwin) == -1)
    error_exit("VIDIOCGWIN");

  if (vwin.flags & PWC_FPS_FRMASK)
  {
    /* set new framerate */
    vwin.flags &= ~PWC_FPS_FRMASK;
    vwin.flags |= (framerate << PWC_FPS_SHIFT);

    if (ioctl(fd, VIDIOCSWIN, &vwin) == -1)
      error_exit("VIDIOCSWIN");
  }
  else
  {
    fprintf(stderr, "This device doesn't support setting the framerate.\n");
    exit(1);
  }
}

void flash_settings(int fd)
{
  if (ioctl(fd, VIDIOCPWCSUSER) == -1)
    error_exit("VIDIOCPWCSUSER");
}

void restore_settings(int fd)
{
  if (ioctl(fd, VIDIOCPWCRUSER) == -1)
    error_exit("VIDIOCPWCRUSER");
}

void restore_factory_settings(int fd)
{
  if (ioctl(fd, VIDIOCPWCFACTORY) == -1)
    error_exit("VIDIOCPWCFACTORY");
}

void set_compression_preference(int fd, int pref)
{
  if (ioctl(fd, VIDIOCPWCSCQUAL, &pref) == -1)
    error_exit("VIDIOCPWCSCQUAL");
}

void set_automatic_gain_control(int fd, int pref)
{
  if (ioctl(fd, VIDIOCPWCSAGC, &pref) == -1)
    error_exit("VIDIOCPWCSAGC");
}

void set_shutter_speed(int fd, int pref)
{
  if (ioctl(fd, VIDIOCPWCSSHUTTER, &pref) == -1)
    error_exit("VIDIOCPWCSSHUTTER");
}

void set_automatic_white_balance_mode(int fd, char *mode)
{
  struct pwc_whitebalance pwcwb;

  if (ioctl(fd, VIDIOCPWCGAWB, &pwcwb) == -1)
    error_exit("VIDIOCPWCGAWB");

  if (strcasecmp(mode, "auto") == 0)
    pwcwb.mode = PWC_WB_AUTO;
  else if (strcasecmp(mode, "manual") == 0)
    pwcwb.mode = PWC_WB_MANUAL;
  else if (strcasecmp(mode, "indoor") == 0)
    pwcwb.mode = PWC_WB_INDOOR;
  else if (strcasecmp(mode, "outdoor") == 0)
    pwcwb.mode = PWC_WB_OUTDOOR;
  else if (strcasecmp(mode, "fl") == 0)
    pwcwb.mode = PWC_WB_FL;
  else
  {
    fprintf(stderr, "'%s' is not a known white balance mode.\n", mode);
    exit(1);
  }

  if (ioctl(fd, VIDIOCPWCSAWB, &pwcwb) == -1)
    error_exit("VIDIOCPWCSAWB");
}

void set_automatic_white_balance_mode_red(int fd, int val)
{
  struct pwc_whitebalance pwcwb;

  if (ioctl(fd, VIDIOCPWCGAWB, &pwcwb) == -1)
    error_exit("VIDIOCPWCGAWB");

  pwcwb.manual_red = val;

  if (ioctl(fd, VIDIOCPWCSAWB, &pwcwb) == -1)
    error_exit("VIDIOCPWCSAWB");
}

void set_automatic_white_balance_mode_blue(int fd, int val)
{
  struct pwc_whitebalance pwcwb;

  if (ioctl(fd, VIDIOCPWCGAWB, &pwcwb) == -1)
    error_exit("VIDIOCPWCGAWB");

  pwcwb.manual_blue = val;

  if (ioctl(fd, VIDIOCPWCSAWB, &pwcwb) == -1)
    error_exit("VIDIOCPWCSAWB");
}

void set_automatic_white_balance_speed(int fd, int val)
{
  struct pwc_wb_speed pwcwbs;

  if (ioctl(fd, VIDIOCPWCGAWBSPEED, &pwcwbs) == -1)
    error_exit("VIDIOCPWCGAWBSPEED");

  pwcwbs.control_speed = val;

  if (ioctl(fd, VIDIOCPWCSAWBSPEED, &pwcwbs) == -1)
    error_exit("VIDIOCPWCSAWBSPEED");
}

void set_automatic_white_balance_delay(int fd, int val)
{
  struct pwc_wb_speed pwcwbs;

  if (ioctl(fd, VIDIOCPWCGAWBSPEED, &pwcwbs) == -1)
    error_exit("VIDIOCPWCGAWBSPEED");

  pwcwbs.control_delay = val;

  if (ioctl(fd, VIDIOCPWCSAWBSPEED, &pwcwbs) == -1)
    error_exit("VIDIOCPWCSAWBSPEED");
}

void set_led_on_time(int fd, int val)
{
  struct pwc_leds pwcl;

  if (ioctl(fd, VIDIOCPWCGLED, &pwcl) == -1)
    error_exit("VIDIOCPWCGLED");

  pwcl.led_on = val;

  if (ioctl(fd, VIDIOCPWCSLED, &pwcl) == -1)
    error_exit("VIDIOCPWCSLED");
}

void set_led_off_time(int fd, int val)
{
  struct pwc_leds pwcl;

  if (ioctl(fd, VIDIOCPWCGLED, &pwcl) == -1)
    error_exit("VIDIOCPWCGLED");

  pwcl.led_off = val;

  if (ioctl(fd, VIDIOCPWCSLED, &pwcl) == -1)
    error_exit("VIDIOCPWCSLED");
}

void set_sharpness(int fd, int val)
{
  if (ioctl(fd, VIDIOCPWCSCONTOUR, &val) == -1)
    error_exit("VIDIOCPWCSCONTOUR");
}

void set_backlight_compensation(int fd, int val)
{
  if (ioctl(fd, VIDIOCPWCSBACKLIGHT, &val) == -1)
    error_exit("VIDIOCPWCSBACKLIGHT");
}

void set_antiflicker_mode(int fd, int val)
{
  if (ioctl(fd, VIDIOCPWCSFLICKER, &val) == -1)
    error_exit("VIDIOCPWCSFLICKER");
}

void set_noise_reduction(int fd, int val)
{
  if (ioctl(fd, VIDIOCPWCSDYNNOISE, &val) == -1)
    error_exit("VIDIOCPWCSDYNNOISE");
}
