#ifndef INCONCE
#include "pgcommon.h"
#include "pgdecompress.h"
#include "pgcalibrate.h"
#define INCONCE
#endif

// ROS includes, should only be in main
#include "ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ros/conversions.h>

// Benchmark stuff
#define ITER_LENGTH_SECONDS 5
int frameCount = 0;
int byteCount = 0;

// ROS publishers
ros::Publisher rgb_pub;
ros::Publisher depth_pub;
ros::Publisher cloud_pub;

// Compiler options:
// Uncomment the following to visualize the cloud
// #define VISUALIZE
// Uncomment the following to publish depth data
// #define PUBDEPTH

int recvAll(const int sfd, uint8_t * buffer, const int len)
{
  int bytesRead = 0;
  int bytesNeeded = len;
  int n = -1;

  while(bytesRead < len) {
    n = recv(sfd, buffer + bytesRead, bytesNeeded, 0);
    if(n < 0) {
      if(errno == EAGAIN) {
        continue;
      }
      break;
    }
    bytesRead += n;
    bytesNeeded -= n;
  }

  return n == -1 ? -1 : bytesRead;
}

#ifdef VISUALIZE
pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
#endif

int networkLoop()
{
  
  int sockfd, new_fd, rv;
  struct addrinfo hints, *servinfo, *p;
  struct sockaddr_storage their_addr;
  socklen_t sin_size;
  char s[INET6_ADDRSTRLEN];
  int yes = 1;

  bzero((void *)&hints, sizeof(hints));
  hints.ai_family = AF_UNSPEC;  // IPv4 or IPv6
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  // Find out own address info
  if((rv = getaddrinfo(NULL, PORTNUM, &hints, &servinfo)) != 0) {
    perror("getaddrinfo");
    exit(1);
  }
  
  // Iterate through list and bind to first usable result
  for(p = servinfo; p != NULL; p = p->ai_next) {
    if((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) \
      {
	perror("socket (L73)");
	continue;
      }

    if(setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                  sizeof(int)) == -1) {
      perror("setsockopt (L79)");
      exit(1);
    }

    if(bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      perror("bind (L85)");
      continue;
    }

    // Acquired, set, and bound successfully for sockfd
    break;
  }

  // If reached end of list, no result found                                   
  if(p == NULL) {
    fprintf(stderr, "Failed to bind\n");
    exit(1);
  }

  freeaddrinfo(servinfo);

  if(listen(sockfd, BACKLOG) == -1) {
    perror("listen (L102)");
    exit(1);
  }

  printf("Socket opened, waiting for client...\n");
  
  // Block until client connects, wait to start ROS 
  sin_size = sizeof(their_addr);
  new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
  if(new_fd == -1) {
    perror("accept (L112)");
    exit(1);
  }

  // Client connected, get name info
  inet_ntop(their_addr.ss_family,
            get_in_addr((struct sockaddr *)&their_addr),
            s, INET6_ADDRSTRLEN);

  // TODO: figure out how many of these aren't used
  uint8_t * rgbbuf = (uint8_t *)malloc(640*480*3*sizeof(uint8_t));
  uint8_t * rgbbuf_compressed = (uint8_t *)malloc(640*480*3*sizeof(uint8_t));
  uint8_t * depthbuf = (uint8_t *)malloc(640*480*3*sizeof(uint8_t));
  uint8_t * depthbuf_compressed = (uint8_t *)malloc(640*480*3*sizeof(uint8_t));
  uint8_t * rgbbuf_undistort = (uint8_t *)malloc(640*480*3*sizeof(uint8_t));
  uint8_t * depthbuf_undistort = (uint8_t *)malloc(640*480*3*sizeof(uint8_t));
  float * depthfp = (float *)malloc(640*480*sizeof(float));

  uint8_t * rgb_flip = (uint8_t *)malloc(640*480*3*sizeof(uint8_t));
  uint8_t * depth_flip = (uint8_t *)malloc(640*480*sizeof(uint16_t));
  
  uint32_t depthSize;
  uint32_t rgbSize;

  // Set up zlib
  setup_depth();

  // Set up JPEG
  setup_rgb();

  // Set up distortion maps
  compute_rgb_map();
  compute_depth_map();
  
  alarm(ITER_LENGTH_SECONDS);

  // Initialize ROS message
  sensor_msgs::Image rgb_msg;
  rgb_msg.width = 640;
  rgb_msg.height = 480;
  rgb_msg.encoding = "rgb8";
  rgb_msg.is_bigendian = 0;
  rgb_msg.step = 640*3;
  rgb_msg.data.resize(640*480*3);

  sensor_msgs::Image depth_msg;
  depth_msg.height = 480;
  depth_msg.width = 640;
  depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg.is_bigendian = 1;
  depth_msg.step = 640*sizeof(float);
  depth_msg.data.resize(640*480*sizeof(float));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg(new pcl::PointCloud<pcl::PointXYZ>());

  // Main receive loop
  while(1) {

    // Pull rgb size
    int rv = recv(new_fd, (void *)&rgbSize, sizeof(uint32_t), 0);
    // Pull rgb frame
    recvAll(new_fd, rgbbuf_compressed, rgbSize);

    // Pull depth size
    rv = recv(new_fd, (void *)&depthSize, sizeof(uint32_t), 0);
    // Pull depth frame
    recvAll(new_fd, depthbuf_compressed, depthSize*sizeof(uint8_t));

    // Decompress frame
    rv = decompress_rgb(rgbbuf, rgbbuf_compressed);
    decompress_depth(depthbuf, depthbuf_compressed, depthSize, 640*480*3*sizeof(uint8_t));    
    
    // Flip frame
    preFlipRGB(rgbbuf, rgb_flip);
    preFlipDepth((uint16_t *)depthbuf, (uint16_t *)depth_flip);
    
    // Undistort frame
    // undistort_rgb(rgb_flip, rgbbuf_undistort);
    
    //flipRowsRGB(rgbbuf, rgbbuf_undistort);

    byteCount += rgbSize;
    
    // Undistort frame
    // undistort_depth((const uint16_t *)depthbuf, (uint16_t *)depthbuf_undistort);
    // flipRowsDepth(depthbuf, depthbuf_undistort);

    // Convert to float
    convertDepthToFP((uint16_t *)depth_flip, depthfp);

    memcpy((uint8_t *)&depth_msg.data[0], depthfp, 640*480*sizeof(float));

    #ifdef PUBDEPTH
    // Publish depth
    depth_pub.publish(depth_msg);
    #endif

    // Generate cloud
    // invalidateCloud(cloud_msg);
    makePointCloud(rgb_flip, depthfp, cloud_msg);
    
    #ifdef VISUALIZE
    // Visualize cloud
    viewer.showCloud(cloud_msg);
    #endif
    

    // Publish messages together
    
    // Copy data into ROS message
    memcpy((uint8_t *)&rgb_msg.data[0], rgb_flip, 640*480*3);
    rgb_pub.publish(rgb_msg);
    cloud_pub.publish(cloud_msg);
    
    byteCount += depthSize;
    
    frameCount++;

    ros::spinOnce();
    
  }

  

}

void alarmHandler(int signum)
{
  printf("Average FPS over last %d seconds: %f\n", ITER_LENGTH_SECONDS, (float)frameCount/ITER_LENGTH_SECONDS);
  printf("Effective bandwidth: %f B/s\n", byteCount/((float)ITER_LENGTH_SECONDS));
  byteCount = 0;
  frameCount = 0;
  alarm(ITER_LENGTH_SECONDS);
}

int main(int argc, char ** argv)
{

  // Initialize ROS message system
  ros::init(argc, argv, "netbench");
  ros::NodeHandle n;
  rgb_pub = n.advertise<sensor_msgs::Image>("camera/rgb/image_color", 1);
  cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZ> >("camera/depth/points", 5);
  
  #ifdef PUBDEPTH
  depth_pub = n.advertise<sensor_msgs::Image>("camera/depth/image", 1);
  #endif

  signal(SIGALRM, alarmHandler);
  networkLoop();
  return 0;
}
