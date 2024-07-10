#ifndef POINTCLOUD_DIVIDER_HPP
#define POINTCLOUD_DIVIDER_HPP

#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <unordered_set>
#include <tuple>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "grid_info.hpp"
#include "pcd_io.hpp"

template <class PointT>
class PointCloudDivider
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef typename PclCloudType::Ptr PclCloudPtr;
  typedef std::unordered_map<GridInfo<2>, std::tuple<PclCloudType, int, size_t>> GridMapType;
  typedef typename GridMapType::iterator GridMapItr;
  typedef std::multimap<size_t, GridMapItr> GridMapSizeType;
  typedef typename GridMapSizeType::iterator GridMapSizeItr;

public:
  PointCloudDivider()
  {
  }
  ~PointCloudDivider()
  {
  }

  std::pair<double, double> getGridSize() const
  {
    return std::pair<double, double>(grid_size_x_, grid_size_y_);
  }

  void run(std::vector<std::string>& pcd_names, const std::string& output_dir, const std::string& file_prefix,
           const std::string& config);

  void run(const PclCloudPtr& cloud, const std::string& output_dir, const std::string& file_prefix,
           const std::string& config);

  std::string makeFileName(const GridInfo<2>& grid) const;

  void setUseLargeGrid(const bool use_large_grid)
  {
    use_large_grid_ = use_large_grid;
  }

private:
  PclCloudPtr merged_ptr_;
  std::string output_dir_, file_prefix_, config_file_;

  std::unordered_set<GridInfo<2>> grid_set_;

  // Params from yaml
  bool use_large_grid_ = false;
  bool merge_pcds_ = false;
  double leaf_size_ = 0.1;
  double grid_size_x_ = 100;
  double grid_size_y_ = 100;
  double g_grid_size_x_ = grid_size_x_ * 10;
  double g_grid_size_y_ = grid_size_y_ * 10;

  // Maximum number of points per PCD block
  const size_t max_block_size_ = 500000;

  // Map of points distributed to grids
  GridMapType grid_to_cloud_;

  // Segments but sorted by size
  GridMapSizeType seg_by_size_;
  // Map segments to size iterator
  std::unordered_map<GridInfo<2>, GridMapSizeItr> seg_to_size_itr_map_;
  // Only 100 million points are allowed to reside in the main memory at max
  const size_t max_resident_point_num_ = 100000000;
  size_t resident_point_num_ = 0;
  std::string tmp_dir_;
  CustomPCDReader<PointT> reader_;

  PclCloudPtr loadPCD(const std::string& pcd_name);
  void savePCD(const std::string& pcd_name, const pcl::PointCloud<PointT>& cloud);
  void saveMergedPCD();
  void dividePointCloud(const PclCloudPtr& cloud_ptr);
  void paramInitialize();
  void saveGridInfoToYAML(const std::string& yaml_file_path);
  void checkOutputDirectoryValidity() const;

  void saveGridPCD(GridMapItr& grid_it);
  void saveTheRest();
  void mergeAndDownsample();
  void mergeAndDownsample(const std::string& dir_path, std::list<std::string>& pcd_list, size_t total_pnum);
};

#endif
