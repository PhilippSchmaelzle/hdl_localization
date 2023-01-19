#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    map_update_sub = nh.subscribe("/map_request/pcd", 10,              &GlobalmapServerNodelet::map_update_callback, this);

    globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalmapServerNodelet::pub_once_cb, this, true, true);
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
    lidar_map_frame_id = private_nh.param<std::string>("lidar_map_frame_id", "map");
    utm_frame_id = private_nh.param<std::string>("utm_frame_id", "utm");

    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = lidar_map_frame_id;

    std::ifstream utm_file(globalmap_pcd + ".utm");

    if (utm_file.is_open()) {
      double utm_easting;
      double utm_northing;
      double altitude;
      double roll;
      double pitch;
      double yaw;
      utm_file >> utm_easting >> utm_northing >> altitude >> yaw >> pitch >> roll;

      tf2::Quaternion map_orientation_q;
      map_orientation_q.setRPY(roll, pitch, yaw);
      map_orientation_q.normalize();

      geometry_msgs::TransformStamped  stamped_transform;
      stamped_transform.header.frame_id = utm_frame_id;
      stamped_transform.header.stamp = ros::Time::now();
      stamped_transform.child_frame_id = lidar_map_frame_id;

      stamped_transform.transform.translation.x = utm_easting;
      stamped_transform.transform.translation.y = utm_northing;
      stamped_transform.transform.translation.z = altitude;

      stamped_transform.transform.rotation.x = map_orientation_q.getX();
      stamped_transform.transform.rotation.y = map_orientation_q.getY();
      stamped_transform.transform.rotation.z = map_orientation_q.getZ();
      stamped_transform.transform.rotation.w = map_orientation_q.getW();

      tf2_static_broad.sendTransform(stamped_transform);

      ROS_INFO("Transformation %s to %s provided!\nx: %f\ny: %f\nz: %f\ny: %f\np: %f\nr: %f", utm_frame_id.c_str(), lidar_map_frame_id.c_str(), utm_easting, utm_northing, altitude, yaw, pitch, roll);
    }
    else{
      ROS_ERROR_STREAM("No transformation " << utm_frame_id << " to " << lidar_map_frame_id << "provided! Please create " << globalmap_pcd << ".utm file with x y z r p y Values!");
    }


    if (utm_file.is_open() && private_nh.param<bool>("convert_utm_to_local", true)) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;

      for(auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      ROS_INFO_STREAM("Global map offset by UTM reference coordinates (x = "
                      << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
    }

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
  }

  void pub_once_cb(const ros::WallTimerEvent& event) {
    globalmap_pub.publish(globalmap);
  }

  void map_update_callback(const std_msgs::String &msg){
    ROS_INFO_STREAM("Received map request, map path : "<< msg.data);
    std::string globalmap_pcd = msg.data;
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = lidar_map_frame_id;

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    std::ifstream utm_file(globalmap_pcd + ".utm");

    if (utm_file.is_open()) {
      double utm_easting;
      double utm_northing;
      double altitude;
      double roll;
      double pitch;
      double yaw;
      utm_file >> utm_easting >> utm_northing >> altitude;

      tf2::Quaternion map_orientation_q;
      map_orientation_q.setRPY(roll, pitch, yaw);
      map_orientation_q.normalize();

      geometry_msgs::TransformStamped  stamped_transform;
      stamped_transform.header.frame_id = utm_frame_id;
      stamped_transform.header.stamp = ros::Time::now();
      stamped_transform.child_frame_id = lidar_map_frame_id;

      stamped_transform.transform.translation.x = utm_easting;
      stamped_transform.transform.translation.y = utm_northing;
      stamped_transform.transform.translation.z = altitude;

      stamped_transform.transform.rotation.x = map_orientation_q.getX();
      stamped_transform.transform.rotation.y = map_orientation_q.getY();
      stamped_transform.transform.rotation.z = map_orientation_q.getZ();
      stamped_transform.transform.rotation.w = map_orientation_q.getW();

      tf2_static_broad.sendTransform(stamped_transform);

      ROS_INFO("Transformation %s to %s provided!\nx: %f\ny: %f\nz: %f\np: %f\nr: %f\ny: %f", utm_frame_id.c_str(), lidar_map_frame_id.c_str(), utm_easting, utm_northing, altitude, roll, pitch, yaw);
    }
    else{
      ROS_ERROR_STREAM("No transformation " << utm_frame_id << " to " << lidar_map_frame_id << "provided! Please create " << globalmap_pcd << ".utm file with x y z r p y Values!");
    }


    globalmap = filtered;
    globalmap_pub.publish(globalmap);
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Subscriber map_update_sub;

  ros::WallTimer globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;

  std::string lidar_map_frame_id;
  std::string utm_frame_id;
  tf2_ros::StaticTransformBroadcaster tf2_static_broad;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
