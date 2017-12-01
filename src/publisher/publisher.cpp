#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
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
  ros::Publisher pub1 = nh.advertise<geometry_msgs::Point> ("/goal_position", 1);
  
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> targets = getCluster(base);
  PointCloud::Ptr light = targets.at(1);
  PointCloud::Ptr box = targets.at(0);
  pcl::CentroidPoint<pcl::PointXYZRGB> boxCentroid;
  for (std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator_indirection<pcl::PointXYZRGB> >::iterator i = box->points.begin(); i != box->points.end(); i++)
  {
    pcl::PointXYZRGB p = *i;
    boxCentroid.add(p);
  }
  pcl::PointXYZRGB c;
  boxCentroid.get(c);
  geometry_msgs::Point goal;
  goal.x = c.x;
  goal.y = c.y;
  goal.z = c.z;  

  while (nh.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
    pub.publish (light);
    pub1.publish (goal);
    std::cout << "published" << std::endl;
    ros::spinOnce ();
    loop_rate.sleep ();
  }
  return 0;
}
