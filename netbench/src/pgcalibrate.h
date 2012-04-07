
void compute_rgb_map(void);

void compute_depth_map(void);

void undistort_rgb(const uint8_t * rgb_buf, uint8_t * dest_buf);
void undistort_depth(const uint16_t * depth_buf, uint16_t * dest_buf);
