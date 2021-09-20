/*
 * point_cloud_global_to_local_node.cpp
 *
 *  Created on: Sept 20, 2021
 *      Author: Maximilian Stoelzle
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map_pcl_loader_node");
  ros::NodeHandle nh("~");
  gm::setVerbosityLevelToDebugIfFlagSet(nh);

  ros::Publisher localPointCloudPub;
  std::string local_point_cloud_topic;
  nh.param<std::string>("local_point_cloud_topic", local_point_cloud_topic, "local_point_cloud");
  localPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>(local_point_cloud_topic, 1, true);

  ros::Subscriber robotPoseSub;
  std::string robot_pose_topic;
  nh.param<std::string>("robot_pose_topic", robot_pose_topic, "/slam/pose");
  // robotPoseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robot_pose_topic, 1, true);

  std::string point_cloud_path;
  nh.param<std::string>("point_cloud_path", point_cloud_path, "/mnt/c/Users/mstolzle/Documents/ethz/MT/anymal-datasets/hoenggerberg/leica-maps/ETH_HPH_binary.ply");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPLYFile<pcl::PointXYZ> (point_cloud_path, *cloud) == -1)
  {
    PCL_ERROR ("Couldn't read pcd file\n");
    return EXIT_FAILURE;
  }

  /* grid_map::GridMapPclLoader gridMapPclLoader;
  const std::string pathToCloud = gm::getPcdFilePath(nh);
  gridMapPclLoader.loadParameters(gm::getParameterPath());
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  gm::processPointcloud(&gridMapPclLoader, nh);

  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(nh));

  // publish point cloud

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap, msg);
  gridMapPub.publish(msg); */

  // run
  ros::spin();
  return EXIT_SUCCESS;
}