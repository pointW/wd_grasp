#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*input, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (5000);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  std::vector<pcl::PointXYZRGB> centroids;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> targets;
  
  //int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    
    pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
    for (std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator_indirection<pcl::PointXYZRGB> >::iterator i = cloud_cluster->points.begin(); i != cloud_cluster->points.end(); i++)
    {
      pcl::PointXYZRGB p = *i;
      centroid.add(p);
    }
    pcl::PointXYZRGB c;
    centroid.get(c);
    
    
    clusters.push_back(cloud_cluster);
    centroids.push_back(c);
    
    
    

    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    //std::stringstream ss;
    //ss << "cloud_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    //j++;
  }
  
  std::cout << clusters.size() << " clusters" << std::endl;
  
  for (int i = 0; i < clusters.size(); i ++)
  {
    Eigen::Vector3i rgb = centroids.at(i).getRGBVector3i ();
    std::cout << "r: " << rgb(0) << " g: " << rgb(1) << " b:" << rgb(2) << std::endl;
    if (rgb(0) > 100 && rgb(1) > 100 && rgb(2) > 100)
    {
      targets.push_back(clusters.at(i));
      std::cout << clusters.at(i)->size() << std::endl;
      break;
    }
  }
  
  for (int i = 0; i < clusters.size(); i ++)
  {
    Eigen::Vector3i rgb = centroids.at(i).getRGBVector3i ();
    std::cout << "r: " << rgb(0) << " g: " << rgb(1) << " b:" << rgb(2) << std::endl;
    if (rgb(0) < 100 && rgb(1) < 100 && rgb(2) > 70)
    {
      targets.push_back(clusters.at(i));
      std::cout << clusters.at(i)->size() << std::endl;
      break;
    }
  }
  return targets;
}

