/**
 * pgcommon.h -- netkinect common declarations
 * (re: sizes, network config, etc.) for client
 * and server.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <zlib.h>
#include "jpeglib.h"

// Network settings
#define PORTNUM "1337"
#define BACKLOG 5
#define PACKET_SIZE sizeof(SERVERCOMMAND)

#define HOSTNAME_STR_LEN 256
#define DEFAULT_HOSTNAME "pgurniak.dyndns.org"

#define US_PER_MS 1000
#define SLEEP_TIME_MS 5

// Enumeration defining the possible commands
// the server can send to the client.  Values specified
// explicitly to ensure that values are consistent
// on different machines (should be anyway).
typedef enum CMD{
  SHUT_DOWN = 0x00,
  SEND_RGB = 0x01,
  SEND_DEPTH = 0x02,
  
  MOVE_FORWARD = 0x10,
  MOVE_BACKWARD = 0x11,
  TURN_LEFT = 0x12,
  TURN_RIGHT = 0x13,
  ROBOT_STOP = 0x14

} SERVERCOMMAND;

// Kinect parameters (start with K_)
#define K_RGB_HEIGHT 480
#define K_RGB_WIDTH 640
#define K_RGB_BYTES 3

#define K_DEPTH_HEIGHT 480
#define K_DEPTH_WIDTH 640

#define DEVICE_NUM 0

// ROS-related constants
#define TOPIC_RGB "camera/rgb/image_mono"
#define QSIZE_RGB 10 // Yes, I know it's 'queue'
#define TOPIC_DEPTH "camera/depth/image"
#define QSIZE_DEPTH 10
#define TOPIC_POINTS "camera/rgb/points"
#define QSIZE_POINTS 1 // Specified in FAQ
#define TOPIC_CAM "cam"
#define QSIZE_CAM 10

#define SCHEDULER_FREQUENCY 10 // in Hz

// Parameters taken fron Nicolas Burrus's homepage
// Color
#define fx_rgb 5.2921508098293293e+02
#define fy_rgb 5.2556393630057437e+02
#define cx_rgb 3.2894272028759258e+02
#define cy_rgb 2.6748068171871557e+02
#define k1_rgb 2.6451622333009589e-01
#define k2_rgb -8.3990749424620825e-01
#define p1_rgb -1.9922302173693159e-03
#define p2_rgb 1.4371995932897616e-03
#define k3_rgb 9.1192465078713847e-01
// Depth
#define fx_d 5.9421434211923247e+02
#define fy_d 5.9104053696870778e+02
#define cx_d 3.3930780975300314e+02
#define cy_d 2.4273913761751615e+02
#define k1_d -2.6386489753128833e-01
#define k2_d 9.9966832163729757e-01
#define p1_d -7.6275862143610667e-04
#define p2_d 5.0350940090814270e-03
#define k3_d -1.3053628089976321e+00


// Parameters for depth correction
#define DEPTH_M -0.0030711016
#define DEPTH_B 3.3309495161

/*
float rotate[3][3] = { { 9.9984628826577793e-01, 1.2635359098409581e-03,
			 -1.7487233004436643e-02 },
		       { -1.4779096108364480e-03, 9.9992385683542895e-01,
			 -1.2251380107679535e-02 },
		       { 1.7470421412464927e-02, 1.2275341476520762e-02,
			 9.9977202419716948e-01}};
*/

/*
float rotate[9] = {
  9.9984628826577793e-01, 1.2635359098409581e-03,
  -1.7487233004436643e-02, -1.4779096108364480e-03,
  9.9992385683542895e-01, -1.2251380107679535e-02,
  1.7470421412464927e-02, 1.2275341476520762e-02,
  9.9977202419716948e-01 };


// Transformation and rotation
#define TRAN_X 1.9985242312092553e-02
#define TRAN_Y -7.4423738761617583e-04
#define TRAN_Z -1.0916736334336222e-02
*/


// Identity transform
/*
float rotate[9] = {
  1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0};
*/

#define TRAN_X 0.0
#define TRAN_Y 0.0
#define TRAN_Z 0.0




/*
// Parameters from my calibration results
#define fx_rgb 5.1826772812058857e+02
#define fy_rgb 5.1773792736460496e+02
#define cx_rgb 3.1663713017566778e+02
#define cy_rgb 2.6602608537056176e+02
#define k1_rgb 2.4251976847354140e-01
#define k2_rgb -9.3617452770254372e-01
#define p1_rgb -2.3195168700935217e-03
#define p2_rgb 1.7676944241394146e-03
#define k3_rgb 1.1632140628529370e+00

#define fx_d 5.7234862370629105e+02
#define fy_d 5.7292445234873844e+02
#define cx_d 3.1794724407857757e+02
#define cy_d 2.5508675973410547e+02
#define k1_d -1.9427189709922524e-01
#define k2_d 9.3126868418516284e-01
#define p1_d 2.0621806395939336e-03
#define p2_d 2.8374791683159510e-03
#define k3_d -1.6571937324774861e+00

const float rotate[9] = {
  9.9961588896735987e-01, -1.8722464473182411e-02,
  2.0433889694455146e-02, 1.8964712225479304e-02,
  9.9975138359550386e-01, -1.1726495174624966e-02,
  -2.0209260604967558e-02, 1.2109513736156758e-02,
  9.9972243421005325e-01 };

#define TRAN_X 1.3698654546024237e-02
#define TRAN_Y -7.9284776842581031e-03
#define TRAN_Z -3.7745466229899150e-03

*/
