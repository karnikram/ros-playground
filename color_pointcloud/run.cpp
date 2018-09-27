#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

cv::Mat pt_2d;
cv::Mat pt_3d(4,1,CV_32FC1);

cv::Mat p_matrix;
pcl::PointCloud<pcl::PointXYZ> pc;
pcl::PointXYZRGB rgb_pc_pt;

cv_bridge::CvImageConstPtr image_ptr;
cv::Mat image;

sensor_msgs::PointCloud2 rgb_pc_msg;

ros::Publisher pub;

cv::Point project(const pcl::PointXYZ &pt)
{
  pt_3d.at<float>(0) = pt.x;
  pt_3d.at<float>(1) = pt.y;
  pt_3d.at<float>(2) = pt.z;
  pt_3d.at<float>(3) = 1.0f;

  pt_2d = p_matrix * pt_3d;
  return cv::Point(pt_2d.at<float>(0)/pt_2d.at<float>(2),pt_2d.at<float>(1)/pt_2d.at<float>(2));

}

void callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg, const sensor_msgs::ImageConstPtr& image_msg)
{

  pcl::fromROSMsg(*pc_msg, pc);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

  image_ptr = cv_bridge::toCvShare(image_msg);

  for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++)
  {
    if(pt->x < 0)
    {
      continue;
    }

    cv::Point xy = project(*pt);

    if(xy.inside(cv::Rect(0,0,image_ptr->image.cols,image_ptr->image.rows)))
    {
      uint8_t b = image_ptr->image.at<cv::Vec3b>(xy)[0];
      uint8_t g = image_ptr->image.at<cv::Vec3b>(xy)[1];
      uint8_t r = image_ptr->image.at<cv::Vec3b>(xy)[2];

      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

      rgb_pc_pt.rgb = *reinterpret_cast<float*>(&rgb);
      rgb_pc_pt.x = pt->x;
      rgb_pc_pt.y = pt->y;
      rgb_pc_pt.z = pt->z;

      pc_rgb->push_back(rgb_pc_pt);
    }
  }

  pcl::toROSMsg(*pc_rgb,rgb_pc_msg);
  rgb_pc_msg.header = pc_msg->header;
  pub.publish(rgb_pc_msg);
}

int main(int argc, char **argv)
{
  //3 x 4 projection matrix to project points in the lidar's frame to the camera's image plane
  p_matrix = (cv::Mat_<float>(3,4) << 609.6954, -721.4216, -1.2513, -123.0418, 180.3842,
                                  7.6448, -719.6515, -101.0167, 0.9999, 0.0001, 0.0105,
                                  -0.2694);

  std::cout << p_matrix << std::endl;

  ros::init(argc, argv, "run");
  ros::NodeHandle nh;

  //point cloud and image topics to subscribe to
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/kitti/velo/pointcloud", 100);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/kitti/camera_color_left/image_raw", 100);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), pc_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  pub = nh.advertise<sensor_msgs::PointCloud2>("rgb_pointcloud", 1000);

  ros::spin();

  return 0;
}
