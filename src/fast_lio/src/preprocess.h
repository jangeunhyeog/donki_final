// preprocess.h — M300 (Pacecat LDS-M300-E) 전용
// Livox SDK, Velodyne, Ouster 관련 코드 제거
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// M300 only — lidar_type 값은 yaml에서 설정되지만 내부적으로 항상 default_handler 사용
enum LID_TYPE { M300 = 1 };

enum TIME_UNIT
{
  SEC = 0,
  MS  = 1,
  US  = 2,
  NS  = 3
};

class Preprocess
{
public:
  Preprocess();
  ~Preprocess();

  void process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(int lid_type, double bld, int pfilt_num);

  PointCloudXYZI pl_full, pl_surf;
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool given_offset_time;

private:
  // M300: standard PointCloud2 (x, y, z, intensity) handler
  void default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg);

  double vx, vy, vz;
};
