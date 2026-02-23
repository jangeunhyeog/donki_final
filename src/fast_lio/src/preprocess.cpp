// preprocess.cpp — M300 (Pacecat LDS-M300-E) 전용
// Velodyne, Ouster, Livox(AVIA/MID360) 핸들러 제거
#include "preprocess.h"

#define RETURN_FALSE (-1)

Preprocess::Preprocess()
  : blind(0.05), lidar_type(M300), point_filter_num(3),
    N_SCANS(6), SCAN_RATE(10), time_unit(US),
    given_offset_time(false), time_unit_scale(1.0),
    vx(0.0), vy(0.0), vz(0.0)
{
}

Preprocess::~Preprocess() {}

void Preprocess::set(int lid_type, double bld, int pfilt_num)
{
  lidar_type     = lid_type;
  blind          = bld;
  point_filter_num = pfilt_num;
}

/**
 * M300 PointCloud2 처리:
 *   - fields: x, y, z, intensity  (Pacecat 드라이버 출력 형식)
 *   - blind 영역(blind*blind) 내 포인트 제거
 *   - point_filter_num 간격으로 다운샘플
 */
void Preprocess::default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_full.clear();

  pcl::PointCloud<pcl::PointXYZI> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num != 0) continue;

    PointType pt;
    pt.x         = pl_orig.points[i].x;
    pt.y         = pl_orig.points[i].y;
    pt.z         = pl_orig.points[i].z;
    pt.intensity = pl_orig.points[i].intensity;
    pt.curvature = 0.0f;
    pt.normal_x  = 0.0f;
    pt.normal_y  = 0.0f;
    pt.normal_z  = 0.0f;

    if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > blind * blind)
    {
      pl_surf.push_back(std::move(pt));
    }
  }
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                         PointCloudXYZI::Ptr &pcl_out)
{
  // time_unit_scale 설정
  switch (time_unit)
  {
    case SEC:  time_unit_scale = 1e3;  break;
    case MS:   time_unit_scale = 1.0;  break;
    case US:   time_unit_scale = 1e-3; break;
    case NS:   time_unit_scale = 1e-6; break;
    default:   time_unit_scale = 1e-3; break;
  }

  // M300은 lidar_type 무관하게 default_handler 사용
  default_handler(msg);
  *pcl_out = pl_surf;
}
