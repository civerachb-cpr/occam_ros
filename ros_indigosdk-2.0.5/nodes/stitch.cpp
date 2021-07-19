#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "indigo.h"

/*

Note that for Omni Stereo, the image numbering is different than
the camera info numbering.
The stereo pair images are (0,1) (2,3) (4,5) (6,7) (8,9) but the
camera info pairs are (0,5) (1,6) (2,7) (3,8) (4,9).

  <node pkg="ros_indigosdk" type="stitch" name="stitch" ns="s0" output="screen">
        <param name="radius" type="int" value="5000000" />
        <param name="rotation" type="int" value="0" />
        <param name="scalewidth" type="int" value="1000" />
        <param name="crop" type="int" value="1" />

        <remap from="image0"   to="/f0/image" />
        <remap from="image1"   to="/f2/image" />
        <remap from="image2"   to="/f4/image" />
        <remap from="image3"   to="/f6/image" />
        <remap from="image4"   to="/f8/image" />
	<remap from="camera_info0"   to="/f0/camera_info" />
	<remap from="camera_info1"   to="/f1/camera_info" />
	<remap from="camera_info2"   to="/f2/camera_info" />
	<remap from="camera_info3"   to="/f3/camera_info" />
	<remap from="camera_info4"   to="/f4/camera_info" />
  </node>

 */

using namespace sensor_msgs;
using namespace message_filters;

void* blend_handle = 0;
IOccamBlendFilter* blend_iface = 0;
IOccamParameters* param_iface = 0;
CameraInfo last_info[5];
ros::Publisher image_stitched;

void reportError(int r) {
  char buf[256];
  occamGetErrorString((OccamError)r, buf, sizeof(buf));
  std::cerr<<"Fatal error: "<<buf<<" ("<<r<<")"<<std::endl;
  abort();
}
#define OCCAM_CHECK(call) {int r = (call);if (r != OCCAM_API_SUCCESS) reportError(r);}

void convert(const Image& img0, OccamImage& img1) {
  memset(&img1,0,sizeof(img1));
  img1.refcnt = 1;
  img1.cid = (char*)"";
  img1.backend = OCCAM_CPU;
  img1.width = img0.width;
  img1.height = img0.height;
  img1.step[0] = img0.step;
  img1.data[0] = (uint8_t*)&img0.data[0];
  if (img0.encoding == "mono8")
    img1.format = OCCAM_GRAY8;
  else if (img0.encoding == "rgb8" ||
	   img0.encoding == "bgr8")
    img1.format = OCCAM_RGB24;
  else {
    std::cerr<<"Unsupported encoding: "<<img0.encoding<<std::endl;
    abort();
  }
}
void convert(const OccamImage& img0, Image& img1) {
  unsigned bpp;
  if (img0.format == OCCAM_GRAY8) {
    img1.encoding = "mono8";
    bpp = 1;
  }
  else if (img0.format == OCCAM_RGB24) {
    img1.encoding = "rgb8";
    bpp = 3;
  }
  else {
    std::cerr<<"Unsupported format: "<<img0.format<<std::endl;
    abort();
  }

  img1.width = img0.width;
  img1.height = img0.height;
  img1.step = img0.step[0];
  img1.data.resize(img0.step[0]*img0.height);
  img1.is_bigendian = 0;

  const uint8_t* srcp = img0.data[0];
  int src_step = img0.step[0];
  uint8_t* dstp = &img1.data[0];
  int dst_step = img1.step;
  for (int j=0;j<img1.height;++j,dstp+=dst_step,srcp+=src_step)
    memcpy(dstp,srcp,img1.width*bpp);
}

void setCameraInfo(const CameraInfoConstPtr& info0,
		   const CameraInfoConstPtr& info1,
		   const CameraInfoConstPtr& info2,
		   const CameraInfoConstPtr& info3,
		   const CameraInfoConstPtr& info4) {
  CameraInfoConstPtr info[5] {
    info0,info1,info2,info3,info4
  };

  bool same = true;
  for (unsigned i=0;i<5;++i)
    if (last_info[i].D != info[i]->D ||
	last_info[i].K != info[i]->K ||
	last_info[i].P != info[i]->P) {
      same = false;
      break;
    }
  if (same)
    return;

  double D[5][5];
  double K[5][9];
  double R[5][9];
  double T[5][3];
  int sensor_width[5];
  int sensor_height[5];
  double* Dp[5];
  double* Kp[5];
  double* Rp[5];
  double* Tp[5];
  for (unsigned i=0;i<5;++i) {
    last_info[i] = *info[i];

    Dp[i] = D[i];
    Kp[i] = K[i];
    Rp[i] = R[i];
    Tp[i] = T[i];
    
    sensor_width[i] = info[i]->width;
    sensor_height[i] = info[i]->height;
    
    D[i][0] = info[i]->D[0];
    D[i][1] = info[i]->D[1];
    D[i][2] = info[i]->D[2];
    D[i][3] = info[i]->D[3];
    D[i][4] = info[i]->D[4];

    K[i][0] = info[i]->K[0];
    K[i][1] = info[i]->K[1];
    K[i][2] = info[i]->K[2];
    K[i][3] = info[i]->K[3];
    K[i][4] = info[i]->K[4];
    K[i][5] = info[i]->K[5];
    K[i][6] = info[i]->K[6];
    K[i][7] = info[i]->K[7];
    K[i][8] = info[i]->K[8];

    R[i][0] = info[i]->P[0];
    R[i][1] = info[i]->P[1];
    R[i][2] = info[i]->P[2];
    T[i][0] = info[i]->P[3];
    R[i][3] = info[i]->P[4];
    R[i][4] = info[i]->P[5];
    R[i][5] = info[i]->P[6];
    T[i][1] = info[i]->P[7];
    R[i][6] = info[i]->P[8];
    R[i][7] = info[i]->P[9];
    R[i][8] = info[i]->P[10];
    T[i][2] = info[i]->P[11];    
  }

  OCCAM_CHECK(blend_iface->configure(blend_handle,5,sensor_width,sensor_height,Dp,Kp,Rp,Tp));      
}

void callback(const ImageConstPtr& image0,
	      const ImageConstPtr& image1,
	      const ImageConstPtr& image2,
	      const ImageConstPtr& image3,
	      const ImageConstPtr& image4) {

  OccamImage img0;
  OccamImage img1;
  OccamImage img2;
  OccamImage img3;
  OccamImage img4;
  OccamImage* img_out = 0;
  convert(*image0, img0);
  convert(*image1, img1);
  convert(*image2, img2);
  convert(*image3, img3);
  convert(*image4, img4);

  OccamImage* imgp[] = {&img0,&img1,&img2,&img3,&img4};
  OCCAM_CHECK(blend_iface->compute(blend_handle,imgp,&img_out));

  Image img_out_ros;
  convert(*img_out, img_out_ros);
  img_out_ros.header = image0->header;
  img_out_ros.encoding = image0->encoding;
  
  image_stitched.publish(img_out_ros);
  
  occamFreeImage(img_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "indigosdk_rectify");

  ros::NodeHandle nh;

  OCCAM_CHECK(occamInitialize());
  OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_BLEND_FILTER, "cylb", &blend_handle));
  assert(blend_handle);
  OCCAM_CHECK(occamGetInterface(blend_handle, IOCCAMBLENDFILTER, (void**)&blend_iface));
  OCCAM_CHECK(occamGetInterface(blend_handle, IOCCAMPARAMETERS, (void**)&param_iface));
  assert(blend_iface);
  assert(param_iface);

  int radius = 5000000;
  int rotation = 0;
  int scalewidth = 1000;
  int crop = 1;

  nh.getParam("stitch/radius", radius);
  nh.getParam("stitch/rotation", rotation);
  nh.getParam("stitch/scalewidth", scalewidth);
  nh.getParam("stitch/crop", crop);
  
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_RADIUS,radius));
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_ROTATION,rotation));
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_SCALEWIDTH,scalewidth));
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_CROP,crop));
  
  message_filters::Subscriber<Image> image0_sub(nh, "image0", 1);
  message_filters::Subscriber<Image> image1_sub(nh, "image1", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "image2", 1);
  message_filters::Subscriber<Image> image3_sub(nh, "image3", 1);
  message_filters::Subscriber<Image> image4_sub(nh, "image4", 1);
  TimeSynchronizer<Image, Image, Image, Image, Image> image_sync
    (image0_sub, image1_sub, image2_sub, image3_sub, image4_sub, 10);
  image_sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  message_filters::Subscriber<CameraInfo> info0_sub(nh, "camera_info0", 1);
  message_filters::Subscriber<CameraInfo> info1_sub(nh, "camera_info1", 1);
  message_filters::Subscriber<CameraInfo> info2_sub(nh, "camera_info2", 1);
  message_filters::Subscriber<CameraInfo> info3_sub(nh, "camera_info3", 1);
  message_filters::Subscriber<CameraInfo> info4_sub(nh, "camera_info4", 1);
  TimeSynchronizer<CameraInfo, CameraInfo, CameraInfo, CameraInfo, CameraInfo> info_sync
    (info0_sub, info1_sub, info2_sub, info3_sub, info4_sub, 10);
  info_sync.registerCallback(boost::bind(&setCameraInfo, _1, _2, _3, _4, _5));
  
  image_stitched = nh.advertise<sensor_msgs::Image>(nh.resolveName("image_stitched"), 10);
  
  ros::spin();

  return 0;
}
