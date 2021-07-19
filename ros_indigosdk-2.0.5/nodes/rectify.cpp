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

  <node pkg="ros_indigosdk" type="rectify" name="rectify" ns="r0" output="screen">
        <remap from="image_top"   to="/f0/image" />
        <remap from="camera_info_top"   to="/f0/camera_info" />
        <remap from="image_bottom"   to="/f1/image" />
        <remap from="camera_info_bottom"   to="/f5/camera_info" />
  </node>

 */

using namespace sensor_msgs;
using namespace message_filters;

void* rectify_handle = 0;
IOccamStereoRectify* rectify_iface = 0;
CameraInfo last_info_top;
CameraInfo last_info_bottom;
ros::Publisher image_top_rect;
ros::Publisher image_bottom_rect;

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

void callback(const ImageConstPtr& image_top, const CameraInfoConstPtr& info_top,
	      const ImageConstPtr& image_bottom, const CameraInfoConstPtr& info_bottom) {

  if (last_info_top.D != info_top->D ||
      last_info_top.K != info_top->K ||
      last_info_top.P != info_top->P ||
      last_info_bottom.D != info_bottom->D ||
      last_info_bottom.K != info_bottom->K ||
      last_info_bottom.P != info_bottom->P) {
    assert(info_top->D.size()>=5);
    assert(info_top->K.size()==9);
    assert(info_top->P.size()==12);
    
    double D0[] = {
      info_top->D[0],
      info_top->D[1],
      info_top->D[2],
      info_top->D[3],
      info_top->D[4]};
    double D1[] = {info_bottom->D[0],
		   info_bottom->D[1],
		   info_bottom->D[2],
		   info_bottom->D[3],
		   info_bottom->D[4]};
    double K0[] = {
      info_top->K[0],info_top->K[1],info_top->K[2],
      info_top->K[3],info_top->K[4],info_top->K[5],
      info_top->K[6],info_top->K[7],info_top->K[8],
    };
    double K1[] = {
      info_bottom->K[0],info_bottom->K[1],info_bottom->K[2],
      info_bottom->K[3],info_bottom->K[4],info_bottom->K[5],
      info_bottom->K[6],info_bottom->K[7],info_bottom->K[8],
    };
    double R0[] = {
      info_top->P[0],info_top->P[1],info_top->P[2],
      info_top->P[4],info_top->P[5],info_top->P[6],
      info_top->P[8],info_top->P[9],info_top->P[10],
    };
    double R1[] = {
      info_bottom->P[0],info_bottom->P[1],info_bottom->P[2],
      info_bottom->P[4],info_bottom->P[5],info_bottom->P[6],
      info_bottom->P[8],info_bottom->P[9],info_bottom->P[10],
    };
    double T0[] = {
      info_top->P[3],
      info_top->P[7],
      info_top->P[11],
    };
    double T1[] = {
      info_bottom->P[3],
      info_bottom->P[7],
      info_bottom->P[11],
    };    
    
    double* Dp[] = {D0,D1};
    double* Kp[] = {K0,K1};
    double* Rp[] = {R0,R1};
    double* Tp[] = {T0,T1};

    unsigned sensor_width = info_top->width;
    unsigned sensor_height = info_top->height;
    rectify_iface->configure(rectify_handle,2,sensor_width,sensor_height,Dp,Kp,Rp,Tp,1);

    last_info_top = *info_top;
    last_info_bottom = *info_bottom;
  }

  OccamImage img1_in;
  OccamImage* img1_out = 0;
  OccamImage img2_in;
  OccamImage* img2_out = 0;
  convert(*image_top, img1_in);
  convert(*image_bottom, img2_in);
  rectify_iface->rectify(rectify_handle,0,&img1_in,&img1_out);
  rectify_iface->rectify(rectify_handle,1,&img2_in,&img2_out);

  Image img1_out_ros;
  Image img2_out_ros;
  convert(*img1_out, img1_out_ros);
  convert(*img2_out, img2_out_ros);
  img1_out_ros.header = image_top->header;
  img1_out_ros.encoding = image_top->encoding;
  img2_out_ros.header = image_bottom->header;
  img2_out_ros.encoding = image_bottom->encoding;

  image_top_rect.publish(img1_out_ros);
  image_bottom_rect.publish(img2_out_ros);

  occamFreeImage(img1_out);
  occamFreeImage(img2_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "indigosdk_rectify");

  ros::NodeHandle nh;

  OCCAM_CHECK(occamInitialize());
  OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_STEREO_RECTIFY, "prec", &rectify_handle));
  occamGetInterface(rectify_handle,IOCCAMSTEREORECTIFY,(void**)&rectify_iface);
  
  message_filters::Subscriber<Image> image_top_sub(nh, "image_top", 1);
  message_filters::Subscriber<CameraInfo> info_top_sub(nh, "camera_info_top", 1);
  message_filters::Subscriber<Image> image_bottom_sub(nh, "image_bottom", 1);
  message_filters::Subscriber<CameraInfo> info_bottom_sub(nh, "camera_info_bottom", 1);
  TimeSynchronizer<Image, CameraInfo, Image, CameraInfo> sync
    (image_top_sub, info_top_sub, image_bottom_sub, info_bottom_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  image_top_rect = nh.advertise<sensor_msgs::Image>(nh.resolveName("image_top_rect"), 10);
  image_bottom_rect = nh.advertise<sensor_msgs::Image>(nh.resolveName("image_bottom_rect"), 10);
  
  ros::spin();

  return 0;
}
