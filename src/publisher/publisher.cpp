#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "../cluster/cluster_extraction.cpp"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

PointCloud::Ptr base (new PointCloud);

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  if (base->points.size() == 0)
  {
    //pcl::copyPointCloud(msg, *base);
    base = msg->makeShared(); 
  }
}


int main(int argc, char** argv)
{
  ros::init (argc, argv, "my_cloud_base");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
    
    
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/cloud_base", 1, callback);
  while (base->points.size() == 0){
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  
  
  ros::Publisher pub = nh.advertise<PointCloud> ("/cloud_light", 1);
  
  
  
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> targets = getCluster(base);
  PointCloud::Ptr msg = targets.at(1);
  std::cout << "get msg" << std::endl;
  

  while (nh.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
    pub.publish (msg);
    std::cout << "msg published" << std::endl;
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  return 0;
}
