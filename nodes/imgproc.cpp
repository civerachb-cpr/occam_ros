#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "indigo.h"

/*

  <node pkg="ros_indigosdk" type="imgproc" name="imgproc" ns="i0" output="screen">
        <remap from="image_in"   to="/f0/image_bayer" />
        <remap from="image_out"   to="/f0/image_color" />
  </node>

  <node pkg="ros_indigosdk" type="imgproc" name="imgproc" ns="i0" output="screen">
        <param name="is_color" type="int" value="1" />
        <param name="image_processing_enabled" type="int" value="1" />
        <param name="brightness" type="int" value="1970" />
        <param name="gamma" type="int" value="1000" />
        <param name="black_level" type="int" value="20" />
        <param name="white_balance_red" type="int" value="1177" />
        <param name="white_balance_green" type="int" value="1268" />
        <param name="white_balance_blue" type="int" value="1645" />

        <remap from="image_in"   to="/f0/image_bayer" />
        <remap from="image_out"   to="/f0/image_color" />
  </node>

*/

using namespace sensor_msgs;
using namespace message_filters;

typedef std::shared_ptr<OccamImage> OccamImagePtr;
void* debayerf_handle;
void* imagef_handle;
int is_color = 1;
CameraInfo last_info_top;
CameraInfo last_info_bottom;
ros::Publisher image_out;

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
  if (img0.encoding == "mono8" ||
      img0.encoding == "bayer_bggr8")
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

OccamImagePtr processImage(OccamImagePtr img0) {
  if (is_color) {
    IOccamImageFilter* imagef_iface;
    occamGetInterface(debayerf_handle,IOCCAMIMAGEFILTER,(void**)&imagef_iface);
    OccamImage* img1 = 0;
    imagef_iface->compute(debayerf_handle,img0.get(),&img1);
    img0 = OccamImagePtr(img1,occamFreeImage);
  }

  IOccamImageFilter* imagef_iface;
  occamGetInterface(imagef_handle,IOCCAMIMAGEFILTER,(void**)&imagef_iface);
  OccamImage* img1 = 0;
  imagef_iface->compute(imagef_handle,img0.get(),&img1);
  return OccamImagePtr(img1,occamFreeImage);
}

void callback(const ImageConstPtr& image) {
  OccamImagePtr img_in = OccamImagePtr((OccamImage*)occamAlloc(sizeof(OccamImage)),occamFree);
  memset(&*img_in, 0, sizeof(OccamImage));
  convert(*image, *img_in);
  OccamImagePtr img_out = processImage(img_in);
  Image img_out_ros;
  convert(*img_out, img_out_ros);
  img_out_ros.header = image->header;
  image_out.publish(img_out_ros);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "indigosdk_rectify");

  ros::NodeHandle nh;

  OCCAM_CHECK(occamInitialize());
  OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_DEBAYER_FILTER, "dbf", &debayerf_handle));
  OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_IMAGE_FILTER, "imf", &imagef_handle));

  int image_processing_enabled = 1;
  int brightness = 1970;
  int gamma = 1000;
  int black_level = 20;
  int white_balance_red = 1177;
  int white_balance_green = 1268;
  int white_balance_blue = 1645;

  nh.getParam("imgproc/is_color", is_color);
  nh.getParam("imgproc/image_processing_enabled", image_processing_enabled);
  nh.getParam("imgproc/brightness", brightness);
  nh.getParam("imgproc/gamma", gamma);
  nh.getParam("imgproc/black_level", black_level);
  nh.getParam("imgproc/white_balance_red", white_balance_red);
  nh.getParam("imgproc/white_balance_green", white_balance_green);
  nh.getParam("imgproc/white_balance_blue", white_balance_blue);

  {
    IOccamParameters* param_iface = 0;
    OCCAM_CHECK(occamGetInterface(imagef_handle, IOCCAMPARAMETERS, (void**)&param_iface));

    OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_COLOR,is_color));
    OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_IMAGE_PROCESSING_ENABLED,image_processing_enabled));
    OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_BRIGHTNESS,brightness));
    OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_GAMMA,gamma));
    OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_BLACK_LEVEL,black_level));
    if (is_color) {
      OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_WHITE_BALANCE_RED,white_balance_red));
      OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_WHITE_BALANCE_GREEN,white_balance_green));
      OCCAM_CHECK(param_iface->setValuei(imagef_handle,OCCAM_WHITE_BALANCE_BLUE,white_balance_blue));
    }
  }

  ros::Subscriber image_in = nh.subscribe("image_in", 10, callback);
  image_out = nh.advertise<sensor_msgs::Image>(nh.resolveName("image_out"), 10);
  
  ros::spin();

  return 0;
}
