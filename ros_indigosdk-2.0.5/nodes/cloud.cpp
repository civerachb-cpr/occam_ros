#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <stereo_msgs/DisparityImage.h>
#include "indigo.h"

/*

Note that for Omni Stereo, the image numbering is different than
the camera info numbering.
The stereo pair images are (0,1) (2,3) (4,5) (6,7) (8,9) but the
camera info pairs are (0,5) (1,6) (2,7) (3,8) (4,9).

  <node pkg="ros_indigosdk" type="cloud" name="cloud" ns="r0" output="screen">
        <remap from="image"   to="/f0/image" />
        <remap from="disp"   to="/f0/disp" />
        <remap from="camera_info"   to="/f0/camera_info" />
        <remap from="camera_info_other"   to="/f5/camera_info" />
  </node>

 */

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters;

void* rectify_handle = 0;
IOccamStereoRectify* rectify_iface = 0;
CameraInfo last_info;
CameraInfo last_info_other;
ros::Publisher cloud;

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
  else if (img0.encoding == "16SC1")
    img1.format = OCCAM_SHORT1;
  else if (img0.encoding == "32FC1")
    img1.format = OCCAM_FLOAT1;
  else {
    std::cerr<<"Unsupported encoding: "<<img0.encoding<<std::endl;
    abort();
  }
}


void callback(const ImageConstPtr& image,
	      const DisparityImageConstPtr& disp,
	      const CameraInfoConstPtr& info,
	      const CameraInfoConstPtr& info_other) {

  if (last_info.D != info->D ||
      last_info.K != info->K ||
      last_info.P != info->P ||
      last_info_other.D != info_other->D ||
      last_info_other.K != info_other->K ||
      last_info_other.P != info_other->P) {
    assert(info->D.size()>=5);
    assert(info->K.size()==9);
    assert(info->P.size()==12);
    
    double D0[] = {
      info->D[0],
      info->D[1],
      info->D[2],
      info->D[3],
      info->D[4]};
    double D1[] = {info_other->D[0],
		   info_other->D[1],
		   info_other->D[2],
		   info_other->D[3],
		   info_other->D[4]};
    double K0[] = {
      info->K[0],info->K[1],info->K[2],
      info->K[3],info->K[4],info->K[5],
      info->K[6],info->K[7],info->K[8],
    };
    double K1[] = {
      info_other->K[0],info_other->K[1],info_other->K[2],
      info_other->K[3],info_other->K[4],info_other->K[5],
      info_other->K[6],info_other->K[7],info_other->K[8],
    };
    double R0[] = {
      info->P[0],info->P[1],info->P[2],
      info->P[4],info->P[5],info->P[6],
      info->P[8],info->P[9],info->P[10],
    };
    double R1[] = {
      info_other->P[0],info_other->P[1],info_other->P[2],
      info_other->P[4],info_other->P[5],info_other->P[6],
      info_other->P[8],info_other->P[9],info_other->P[10],
    };
    double T0[] = {
      info->P[3],
      info->P[7],
      info->P[11],
    };
    double T1[] = {
      info_other->P[3],
      info_other->P[7],
      info_other->P[11],
    };    
    
    double* Dp[] = {D0,D1};
    double* Kp[] = {K0,K1};
    double* Rp[] = {R0,R1};
    double* Tp[] = {T0,T1};

    unsigned sensor_width = info->width;
    unsigned sensor_height = info->height;
    rectify_iface->configure(rectify_handle,2,sensor_width,sensor_height,Dp,Kp,Rp,Tp,1);

    last_info = *info;
    last_info_other = *info_other;
  }

  OccamImage image_in;
  OccamImage disp_in;
  convert(*image, image_in);
  convert(disp->image, disp_in);

  OccamPointCloud* pc0 = nullptr;
  int index = 0;
  OccamImage* image_inp = &image_in;
  OccamImage* disp_inp = &disp_in;
  rectify_iface->generateCloud
    (rectify_handle, 1, &index, 0, &image_inp, &disp_inp, &pc0);

  sensor_msgs::PointCloud2 pc2;
  pc2.header.seq = disp->header.seq;
  pc2.header.frame_id = "occam";
  pc2.header.stamp = disp->header.stamp;

  pc2.height = 1;
  pc2.width = pc0->point_count;

  unsigned point_step = 0;
    
  sensor_msgs::PointField& fx = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
  fx.name = "x";
  fx.offset = point_step;
  point_step += sizeof(float);
  fx.datatype = sensor_msgs::PointField::FLOAT32;
  fx.count = 1;

  sensor_msgs::PointField& fy = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
  fy.name = "y";
  fy.offset = point_step;
  point_step += sizeof(float);
  fy.datatype = sensor_msgs::PointField::FLOAT32;
  fy.count = 1;

  sensor_msgs::PointField& fz = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
  fz.name = "z";
  fz.offset = point_step;
  point_step += sizeof(float);
  fz.datatype = sensor_msgs::PointField::FLOAT32;
  fz.count = 1;

  if (pc0->rgb) {
    sensor_msgs::PointField& frgb = *pc2.fields.insert(pc2.fields.end(), sensor_msgs::PointField());
    frgb.name = "rgb";
    frgb.offset = point_step;
    point_step += sizeof(float);
    frgb.datatype = sensor_msgs::PointField::FLOAT32;
    frgb.count = 1;
  }

  pc2.is_bigendian = false;
  pc2.point_step = point_step;
  pc2.row_step = point_step * pc0->point_count;
  pc2.is_dense = true;

  for (int j=0,k=0;j<pc0->point_count;++j,k+=3) {
    float x = pc0->xyz[k+0] / 1000.f;
    float y = pc0->xyz[k+1] / 1000.f;
    float z = pc0->xyz[k+2] / 1000.f;

    pc2.data.insert(pc2.data.end(), (uint8_t*)&x,(uint8_t*)&x + sizeof(x));
    pc2.data.insert(pc2.data.end(), (uint8_t*)&y,(uint8_t*)&y + sizeof(y));
    pc2.data.insert(pc2.data.end(), (uint8_t*)&z,(uint8_t*)&z + sizeof(z));

    if (pc0->rgb) {
      uint8_t r = pc0->rgb[k+0];
      uint8_t g = pc0->rgb[k+1];
      uint8_t b = pc0->rgb[k+2];
      uint32_t rgb = (uint32_t(r)<<16) | (uint32_t(g)<<8) | uint32_t(b);
      float rgbf = *(float*)&rgb;
      pc2.data.insert(pc2.data.end(), (uint8_t*)&rgbf,(uint8_t*)&rgbf + sizeof(rgbf));
    }
  }

  cloud.publish(pc2);

  occamFreePointCloud(pc0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "indigosdk_rectify");

  ros::NodeHandle nh;

  OCCAM_CHECK(occamInitialize());
  OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_STEREO_RECTIFY, "prec", &rectify_handle));
  occamGetInterface(rectify_handle,IOCCAMSTEREORECTIFY,(void**)&rectify_iface);

  message_filters::Subscriber<Image> image_sub(nh, "image", 1);
  message_filters::Subscriber<DisparityImage> disp_sub(nh, "disp", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "camera_info", 1);
  message_filters::Subscriber<CameraInfo> info_other_sub(nh, "camera_info_other", 1);
  TimeSynchronizer<Image, DisparityImage, CameraInfo, CameraInfo> sync
    (image_sub, disp_sub, info_sub, info_other_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  cloud = nh.advertise<sensor_msgs::PointCloud2>(nh.resolveName("cloud"), 10);
  
  ros::spin();

  return 0;
}
