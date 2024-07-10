#ifndef POINTCLOUD_DIVIDER__VOXEL_GRID_FILTER_HPP_
#define POINTCLOUD_DIVIDER__VOXEL_GRID_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>

template <typename PointT>
class VoxelGridFilter
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef typename PclCloudType::Ptr PclCloudPtr;

public:
  VoxelGridFilter() { resolution_ = 0; }

  void setResolution(float res)
  {
    if (res > 0) {
      resolution_ = res;
    }
  }

  void filter(const PclCloudType & input, PclCloudType & output);

private:
  float resolution_;
};

template class VoxelGridFilter<pcl::PointXYZ>;
template class VoxelGridFilter<pcl::PointXYZI>;

#endif  // POINTCLOUD_DIVIDER__VOXEL_GRID_FILTER_HPP_
