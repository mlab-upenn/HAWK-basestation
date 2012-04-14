/**
 * Compression code for the Networked kinect module
 */

#ifndef INCONCE
#include "pgcommon.h"
#include "pgdecompress.h"
#define INCONCE
#endif

z_stream strm;

/**                                                                                                       * get_in_addr(): returns either IPv4 or IPv6 address given 
 * sockaddr struct.
 */
void * get_in_addr(struct sockaddr *sa)
{
  if(sa->sa_family == AF_INET) {
    return &(((struct sockaddr_in *)sa)->sin_addr);
  }
  else {
    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }
}


int setup_depth( void )
{
  int ret;
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = 0;
  strm.next_in = Z_NULL;
  ret = inflateInit(&strm);
  if(ret != Z_OK) {
    printf("inflateInit failed!\n");
    return -1;
  }
  return 0;
}

int decompress_depth(uint8_t * dest, const uint8_t * src, const int size,
		const int dest_size)
{
  strm.avail_in = size;
  strm.next_in = (Bytef *)src;
  strm.avail_out = dest_size;
  strm.next_out = dest;
  
  int ret = inflate(&strm, Z_NO_FLUSH);
  
  if(ret != Z_STREAM_END) {
    printf("Expected Z_STREAM_END!\n");
    return -1;
  }

  ret = inflateReset(&strm);
  if(ret != Z_OK) {
    printf("zlib reset failed!\n");
    return -1;
  }
  return 0;
}

struct jpeg_decompress_struct cinfo;
struct jpeg_error_mgr jerr;

int setup_rgb(void)
{
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);
  return 0;
}

int decompress_close(void)
{
  jpeg_destroy_decompress(&cinfo);
  return 0;
}

int decompress_rgb(uint8_t * dest, uint8_t * src)
{
  jpeg_mem_src(&cinfo, src, K_RGB_HEIGHT*K_RGB_WIDTH*K_RGB_BYTES);
  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);
  int row_stride = cinfo.output_width*cinfo.output_components;

  while(cinfo.output_scanline < cinfo.output_height) {
    uint8_t * ptr = dest + row_stride*cinfo.output_scanline;
    jpeg_read_scanlines(&cinfo, &ptr, 1);
  }
  jpeg_finish_decompress(&cinfo);

  return 0;
}

