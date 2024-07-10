#ifndef VGRID_FILTER_HPP_
#define VGRID_FILTER_HPP_

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template <typename PointT>
class VoxelGridFilter
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef typename PclCloudType::Ptr PclCloudPtr;

public:
  VoxelGridFilter()
  {
    resolution_ = 0;
  }

  void setResolution(float res)
  {
    if (res > 0)
    {
      resolution_ = res;
    }
  }

  void filter(const PclCloudType& input, PclCloudType& output);

private:
  float resolution_;
};

template class VoxelGridFilter<pcl::PointXYZ>;
template class VoxelGridFilter<pcl::PointXYZI>;

#endif