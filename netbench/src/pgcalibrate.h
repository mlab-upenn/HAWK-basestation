
void compute_rgb_map(void);

void compute_depth_map(void);

void undistort_rgb(const uint8_t * rgb_buf, uint8_t * dest_buf);
void undistort_depth(const uint16_t * depth_buf, uint16_t * dest_buf);

void convertDepthToFP(uint16_t * depth_buf, float * fp_buf);

void invalidateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg);

void makePointCloud(const uint8_t * rgb_buf, const float * depth_buf,
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg);

void flipRowsRGB(uint8_t * rgb_buf, uint8_t * flip_buf);
void flipRowsDepth(uint8_t * depth_buf, uint8_t * flip_buf);

void preFlipRGB(uint8_t * rgb_buf, uint8_t * flip_buf);
void preFlipDepth(uint16_t * depth_buf, uint16_t * flip_buf);
