#include <filesystem>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pointcloud_divider/utility.hpp>

#include <pointcloud_divider/pointcloud_divider.hpp>
#include <pointcloud_divider/voxel_grid_filter.hpp>

namespace fs = std::filesystem;

template <class PointT>
void PointCloudDivider<PointT>::run(const PclCloudPtr& cloud_ptr, const std::string& output_dir,
                                    const std::string& file_prefix, const std::string& config)

{
  output_dir_ = output_dir;
  file_prefix_ = file_prefix;
  config_file_ = config;

  grid_set_.clear();

  paramInitialize();
  checkOutputDirectoryValidity();
  dividePointCloud(cloud_ptr);
  mergeAndDownsample();
}

template <class PointT>
void PointCloudDivider<PointT>::run(std::vector<std::string>& pcd_names, const std::string& output_dir,
                                    const std::string& file_prefix, const std::string& config)
{
  output_dir_ = output_dir;
  file_prefix_ = file_prefix;
  config_file_ = config;
  tmp_dir_ = output_dir + "/tmp/";

  grid_set_.clear();
  paramInitialize();

  if (fs::exists(tmp_dir_))
  {
    fs::remove_all(tmp_dir_);
  }

  if (fs::exists(output_dir_))
  {
    fs::remove_all(output_dir_);
  }

  util::make_dir(output_dir_);
  util::make_dir(tmp_dir_);

  for (const std::string& pcd_name : pcd_names)
  {
    std::cout << "Processing file " << pcd_name << "..." << std::endl;

    do
    {
      auto cloud_ptr = loadPCD(pcd_name);

      dividePointCloud(cloud_ptr);
    } while (reader_.good());
  }

  saveTheRest();

  std::cerr << "Merge and downsample " << std::endl;

  // Now merge and downsample
  mergeAndDownsample();

  std::string yaml_file_path = output_dir_ + "/" + file_prefix_ + "_metadata.yaml";
  saveGridInfoToYAML(yaml_file_path);
}

template <class PointT>
void PointCloudDivider<PointT>::checkOutputDirectoryValidity() const
{
  if (!std::filesystem::exists(output_dir_))
  {
    std::cerr << "Error: " << output_dir_
              << " does not exist. Please make the directory before execute this application." << std::endl;
    exit(1);
  }
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudDivider<PointT>::loadPCD(const std::string& pcd_name)
{
  if (pcd_name != reader_.get_path())
  {
    reader_.setInput(pcd_name);
  }

  PclCloudPtr cloud_ptr(new PclCloudType);

  reader_.readABlock(*cloud_ptr);

  return cloud_ptr;
}

template <class PointT>
void PointCloudDivider<PointT>::savePCD(const std::string& path, const pcl::PointCloud<PointT>& cloud)
{
  if (pcl::io::savePCDFileBinary(path, cloud) == -1)
  {
    std::cerr << "Error: Cannot save PCD: " << path << std::endl;
    exit(1);
  }
}

template <class PointT>
void PointCloudDivider<PointT>::saveMergedPCD()
{
  if (merge_pcds_)
  {
    std::string filename = output_dir_ + "/" + file_prefix_ + ".pcd";
    savePCD(filename, *merged_ptr_);
  }
}

template <class PointT>
void PointCloudDivider<PointT>::dividePointCloud(const PclCloudPtr& cloud_ptr)
{
  if (!cloud_ptr || cloud_ptr->size() <= 0)
  {
    return;
  }

  std::cout << "\tProcessing a block of " << cloud_ptr->size() << " points... \r" << std::flush;

  size_t progress = 0, total_pnum = cloud_ptr->size();

  for (const PointT p : *cloud_ptr)
  {
    if (progress % 10000 == 0 || progress == cloud_ptr->size())
    {
      std::cout << "\tProcessing a block of " << cloud_ptr->size() << " points... "
                << static_cast<int>(static_cast<float>(progress) / static_cast<float>(total_pnum) * 100) << "%\r"
                << std::flush;
    }
    ++progress;

    auto tmp = pointToGrid2(p, grid_size_x_, grid_size_y_);
    auto it = grid_to_cloud_.find(tmp);

    // If the grid has not existed yet, create a new one
    if (it == grid_to_cloud_.end())
    {
      auto& new_grid = grid_to_cloud_[tmp];

      std::get<0>(new_grid).reserve(max_block_size_);
      std::get<0>(new_grid).push_back(p);  // Push the first point to the cloud
      std::get<1>(new_grid) = 0;           // Counter set to 0
      std::get<2>(new_grid) = 0;           // Prev size is 0
    }
    else
    {
      auto& cloud = std::get<0>(it->second);
      auto& counter = std::get<1>(it->second);
      auto& prev_size = std::get<2>(it->second);

      cloud.push_back(p);

      ++resident_point_num_;

      // If the number of points in the segment reach maximum, save the segment to file
      if (cloud.size() == max_block_size_)
      {
        saveGridPCD(it);
      }
      else
      {
        // Otherwise, update the seg_by_size_ if the change of size is significant
        if (cloud.size() - prev_size >= 10000)
        {
          prev_size = cloud.size();
          auto seg_to_size_it = seg_to_size_itr_map_.find(tmp);

          if (seg_to_size_it == seg_to_size_itr_map_.end())
          {
            auto size_it = seg_by_size_.insert(std::make_pair(prev_size, it));
            seg_to_size_itr_map_[tmp] = size_it;
          }
          else
          {
            seg_by_size_.erase(seg_to_size_it->second);
            auto new_size_it = seg_by_size_.insert(std::make_pair(prev_size, it));
            seg_to_size_it->second = new_size_it;
          }
        }
      }

      // If the number of resident points reach maximum, save the biggest resident segment to SSD
      if (resident_point_num_ >= max_resident_point_num_)
      {
        auto max_size_seg_it = seg_by_size_.rbegin();

        saveGridPCD(max_size_seg_it->second);
      }
    }
  }

  std::cout << "\tProcessing a block of " << cloud_ptr->size() << " points... 100%" << std::endl;
}

template <class PointT>
void PointCloudDivider<PointT>::saveGridPCD(GridMapItr& grid_it)
{
  auto& cloud = std::get<0>(grid_it->second);
  auto& counter = std::get<1>(grid_it->second);
  auto& prev_size = std::get<2>(grid_it->second);

  // Try to create a new directory to contain the cloud
  std::ostringstream seg_path, file_path;

  seg_path << tmp_dir_ << "/" << grid_it->first << "/";
  file_path << seg_path.str() << counter << "_" << cloud.size() << ".pcd";

  util::make_dir(seg_path.str());

  if (pcl::io::savePCDFileBinary(file_path.str(), cloud))
  {
    std::cerr << "Error: Cannot save a PCD file at " << file_path.str() << std::endl;
    exit(EXIT_FAILURE);
  }

  resident_point_num_ -= cloud.size();

  // Clear the content of the segment cloud and reserve space for further points
  cloud.clear();
  cloud.reserve(max_block_size_);
  ++counter;  // Increase the counter so the next segment save will not overwrite the previously saved one
  prev_size = 0;

  // Update the seg_by_size_ and seg_to_size_itr_map_
  auto it = seg_to_size_itr_map_.find(grid_it->first);

  if (it != seg_to_size_itr_map_.end())
  {
    seg_by_size_.erase(it->second);
    seg_to_size_itr_map_.erase(it);
  }
}

template <class PointT>
void PointCloudDivider<PointT>::saveTheRest()
{
  for (auto it = grid_to_cloud_.begin(); it != grid_to_cloud_.end(); ++it)
  {
    auto& cloud = std::get<0>(it->second);

    if (cloud.size() > 0)
    {
      saveGridPCD(it);
    }
  }
}

template <class PointT>
void PointCloudDivider<PointT>::mergeAndDownsample()
{
  // Scan the tmp directory and find the segment folders
  fs::path tmp_path(tmp_dir_);

  // Count the number of sub folders, to estimate progress
  int seg_num = 0, progress = 0;

  for (auto& entry : fs::directory_iterator(tmp_path))
  {
    if (fs::is_directory(entry.symlink_status()))
    {
      ++seg_num;
    }
  }

  for (auto& tmp_dir_entry : fs::directory_iterator(tmp_path))
  {
    if (fs::is_directory(tmp_dir_entry.symlink_status()))
    {
      std::cout << "\tProcessing... "
                << static_cast<int>(static_cast<float>(progress) / static_cast<float>(seg_num) * 100) << "%\r"
                << std::flush;
      ++progress;

      std::list<std::string> pcd_list;
      size_t total_pnum = 0;

      for (auto& seg_entry : fs::directory_iterator(tmp_dir_entry.path()))
      {
        if (fs::is_regular_file(seg_entry.symlink_status()))
        {
          auto fname = seg_entry.path().string();
          auto ext = fname.substr(fname.size() - 4);

          if (ext == ".pcd")
          {
            pcd_list.push_back(fname);
            total_pnum += util::point_num(fname);
          }
        }
      }

      // Fuse all PCDs and downsample if necessary
      mergeAndDownsample(tmp_dir_entry.path().string(), pcd_list, total_pnum);
    }
  }

  std::cout << "\tProcessing... 100%" << std::endl;

  // Remove tmp dir
  util::remove(tmp_dir_);
}

template <class PointT>
void PointCloudDivider<PointT>::mergeAndDownsample(const std::string& dir_path, std::list<std::string>& pcd_list,
                                                   size_t total_pnum)
{
  PclCloudPtr new_cloud(new PclCloudType);

  new_cloud->reserve(total_pnum);

  // Merge all PCDs to a single point cloud
  for (auto& fname : pcd_list)
  {
    PclCloudType seg_cloud;

    if (pcl::io::loadPCDFile(fname, seg_cloud))
    {
      std::cerr << "Error: Failed to open a PCD file at " << fname << std::endl;
      exit(EXIT_FAILURE);
    }

    for (auto& p : seg_cloud)
    {
      new_cloud->push_back(p);
    }
  }

  // Downsample if needed
  if (leaf_size_ > 0)
  {
    VoxelGridFilter<PointT> vgf;
    PclCloudPtr filtered_cloud(new PclCloudType);

    vgf.setResolution(leaf_size_);
    vgf.filter(*new_cloud, *filtered_cloud);

    new_cloud = filtered_cloud;
  }

  // Save the new_cloud
  // Extract segment name only (format gx_gy)
  size_t end_name, start_name;

  for (end_name = dir_path.size() - 1; end_name >= 0 && dir_path[end_name] == '/'; --end_name)
  {
  }
  for (start_name = end_name; start_name >= 0 && dir_path[start_name] != '/'; --start_name)
  {
  }

  std::string seg_name_only = dir_path.substr(start_name + 1, end_name - start_name);

  // Parse the @seg_path to get the indices of the segment
  auto ubar_pos = seg_name_only.rfind("_");
  int gx = std::stoi(seg_name_only.substr(0, ubar_pos));
  int gy = std::stoi(seg_name_only.substr(ubar_pos + 1));

  grid_set_.insert(GridInfo<2>(gx, gy));

  // Construct the path to save the new_cloud
  std::string save_path;

  // If save large pcd was turned on, create a folder to contain segment pcds
  if (use_large_grid_)
  {
    int large_gx = static_cast<int>(std::floor(static_cast<float>(gx) / g_grid_size_x_));
    int large_gy = static_cast<int>(std::floor(static_cast<float>(gy) / g_grid_size_y_));
    std::string large_folder = output_dir_ + "/" + std::to_string(large_gx) + "_" + std::to_string(large_gy) + "/";

    // Create a new folder for the large grid
    util::make_dir(large_folder);

    save_path = large_folder + file_prefix_ + "_" + seg_name_only + ".pcd";
  }
  else
  {
    save_path = output_dir_ + "/" + file_prefix_ + "_" + seg_name_only + ".pcd";
  }

  // Save the merged (filtered) cloud
  if (pcl::io::savePCDFileBinary(save_path, *new_cloud))
  {
    std::cerr << "Error: Failed to save a point cloud at " << save_path << std::endl;
    exit(EXIT_FAILURE);
  }

  // Delete the folder containing the segments
  util::remove(dir_path);
}

template <class PointT>
std::string PointCloudDivider<PointT>::makeFileName(const GridInfo<2>& grid) const
{
  std::string file_name = output_dir_;

  file_name = file_name + file_prefix_ + "_";
  file_name = file_name + std::to_string(grid.ix) + "_" + std::to_string(grid.iy) + ".pcd";

  return file_name;
}

template <class PointT>
void PointCloudDivider<PointT>::paramInitialize()
{
  try
  {
    YAML::Node conf = YAML::LoadFile(config_file_)["pointcloud_divider"];
    use_large_grid_ = conf["use_large_grid"].as<bool>();
    merge_pcds_ = conf["merge_pcds"].as<bool>();
    leaf_size_ = conf["leaf_size"].as<double>();
    grid_size_x_ = conf["grid_size_x"].as<double>();
    grid_size_y_ = conf["grid_size_y"].as<double>();
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "YAML Error: " << e.what() << std::endl;
    exit(1);
  }

  g_grid_size_x_ = grid_size_x_ * 10;
  g_grid_size_y_ = grid_size_y_ * 10;

  if (merge_pcds_)
    merged_ptr_.reset(new pcl::PointCloud<PointT>);
}

template <class PointT>
void PointCloudDivider<PointT>::saveGridInfoToYAML(const std::string& yaml_file_path)
{
  std::ofstream yaml_file(yaml_file_path);

  if (!yaml_file.is_open())
  {
    std::cerr << "Error: Cannot open YAML file: " << yaml_file_path << std::endl;
    exit(1);
  }

  yaml_file << "x_resolution: " << grid_size_x_ << std::endl;
  yaml_file << "y_resolution: " << grid_size_y_ << std::endl;

  for (const auto& grid : grid_set_)
  {
    std::string file_name = makeFileName(grid);
    fs::path p(file_name);
    yaml_file << p.filename().string() << ": [" << grid.ix << ", " << grid.iy << "]" << std::endl;
  }

  yaml_file.close();
}

// template class PointCloudDivider<pcl::PointXYZ>;
template class PointCloudDivider<pcl::PointXYZI>;
// template class PointCloudDivider<pcl::PointXYZINormal>;
// template class PointCloudDivider<pcl::PointXYZRGB>;
// template class PointCloudDivider<pcl::PointNormal>;
