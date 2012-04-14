
void * get_in_addr(struct sockaddr * sa);

int setup_depth(void);

int decompress_depth(uint8_t * det, const uint8_t * src, const int size, const int dest_size);

int setup_rgb(void);

int decompress_close(void);

int decompress_rgb(uint8_t * dest, uint8_t * src);


