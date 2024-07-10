#ifndef CUSTOM_PCD_LOADER_HPP_
#define CUSTOM_PCD_LOADER_HPP_

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include "utility.hpp"

template <typename PointT>
class CustomPCDReader
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef std::tuple<size_t, std::string, int> FieldMetaType;

public:
  CustomPCDReader()
  {
    buffer_ = nullptr;
    clear();
  }

  // Set a file to reading
  void setInput(const std::string& pcd_path);
  // Read a block of points from the input stream
  size_t readABlock(PclCloudType& output);

  // Get path to the current opening PCD
  const std::string& get_path() const
  {
    return pcd_path_;
  }

  void setBlockSize(size_t block_size)
  {
    block_size_ = block_size;
  }

  bool good()
  {
    return file_.good();
  }

  ~CustomPCDReader()
  {
    if (buffer_)
    {
      delete[] buffer_;
    }
  }

private:
  void readHeader(std::ifstream& input);
  size_t readABlock(std::ifstream& input, PclCloudType& output);

  size_t readABlockBinary(std::ifstream& input, PclCloudType& output);
  size_t readABlockASCII(std::ifstream& input, PclCloudType& output);

  void clear()
  {
    version_.clear();
    width_ = height_ = 0;
    field_names_.clear();
    field_sizes_.clear();
    field_types_.clear();
    field_counts_.clear();
    origin_x_ = origin_y_ = origin_z_ = 0.0;
    orientation_w_ = orientation_x_ = orientation_y_ = orientation_z_ = 0.0;
    point_num_ = 0;
    binary_ = true;

    if (file_.is_open())
    {
      file_.close();
    }

    file_.clear();

    point_size_ = read_size_ = 0;
    if (buffer_)
    {
      delete[] buffer_;
    }

    buffer_ = nullptr;
    block_size_ = 30000000;
  }

  // Metadata
  std::string version_;
  size_t width_, height_;
  std::vector<std::string> field_names_;
  std::vector<size_t> field_sizes_;
  std::vector<std::string> field_types_;
  std::vector<int> field_counts_;
  // Viewpoint
  float origin_x_, origin_y_, origin_z_;
  float orientation_w_, orientation_x_, orientation_y_, orientation_z_;
  // Number of points in the PCD file
  size_t point_num_;
  bool binary_;                   // Data: true: binary, false: ascii
  std::ifstream file_;            // Input stream of the PCD file
  size_t block_size_ = 30000000;  // Number of points to read in each readABlock
  size_t point_size_, read_size_;
  char* buffer_;
  std::string pcd_path_;  // Path to the current opening PCD
  std::vector<size_t> read_loc_;
  std::vector<size_t> read_sizes_;
};

template <typename PointT>
void CustomPCDReader<PointT>::setInput(const std::string& pcd_path)
{
  clear();

  file_.open(pcd_path);

  if (!file_.is_open())
  {
    fprintf(stderr, "[%s, %d] %s::Error: Failed to open a file at %s\n", __FILE__, __LINE__, __func__,
            pcd_path.c_str());
    exit(EXIT_FAILURE);
  }

  pcd_path_ = pcd_path;
  readHeader(file_);
}

template <typename PointT>
size_t CustomPCDReader<PointT>::readABlock(PclCloudType& output)
{
  return readABlock(file_, output);
}

template <typename PointT>
inline void buildReadMetadata(std::vector<std::string>& field_names, std::vector<size_t>& field_sizes,
                              std::vector<size_t>& read_loc, std::vector<size_t>& read_sizes);

template <typename PointT>
void CustomPCDReader<PointT>::readHeader(std::ifstream& input)
{
  std::string line;
  std::vector<std::string> vals;  // Each line is splitted into multiple values by a delimiter

  try
  {
    while (input)
    {
      std::getline(input, line);

      if (line.empty())
      {
        continue;
      }

      util::split(line, " ", vals);

      // Read comments
      if (vals[0] == "#")
      {
        continue;
      }

      if (vals[0] == "VERSION")
      {
        version_ = vals[1];

        continue;
      }

      if (vals[0] == "FIELDS")
      {
        for (size_t i = 1; i < vals.size(); ++i)
        {
          field_names_.push_back(vals[i]);
        }

        continue;
      }

      if (vals[0] == "SIZE")
      {
        if (vals.size() - 1 != field_names_.size())
        {
          fprintf(stderr,
                  "[%s, %d] %s::Error: Invalid PCD file format: Number of sizes is different from number of fields. "
                  "File %s\n",
                  __FILE__, __LINE__, __func__, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        for (size_t i = 1; i < vals.size(); ++i)
        {
          field_sizes_.push_back(std::stoi(vals[i]));
        }

        continue;
      }

      if (vals[0] == "TYPE")
      {
        if (vals.size() - 1 != field_names_.size())
        {
          fprintf(stderr,
                  "[%s, %d] %s::Error: Invalid PCD file format: Number of types is different from number of fields. "
                  "File %s\n",
                  __FILE__, __LINE__, __func__, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        for (size_t i = 1; i < vals.size(); ++i)
        {
          field_types_.push_back(vals[i]);
        }

        continue;
      }

      if (vals[0] == "COUNT")
      {
        if (vals.size() - 1 != field_names_.size())
        {
          fprintf(stderr,
                  "[%s, %d] %s::Error: Invalid PCD file format: Number of counts is different from number of fields. "
                  "File %s\n",
                  __FILE__, __LINE__, __func__, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        for (size_t i = 1; i < vals.size(); ++i)
        {
          field_types_.push_back(vals[i]);
        }

        continue;
      }

      if (vals[0] == "WIDTH")
      {
        width_ = std::stoi(vals[1]);

        continue;
      }

      if (vals[0] == "HEIGHT")
      {
        height_ = std::stoi(vals[1]);

        continue;
      }

      if (vals[0] == "VIEWPOINT")
      {
        if (vals.size() != 8)
        {
          fprintf(stderr,
                  "[%s, %d] %s::Error: Invalid PCD file format: Viewpoint must contain 7 fields. Number of actual "
                  "fields: %lu. File %s\n",
                  __FILE__, __LINE__, __func__, vals.size() - 1, pcd_path_.c_str());
          exit(EXIT_FAILURE);
        }

        origin_x_ = std::stof(vals[1]);
        origin_y_ = std::stof(vals[2]);
        origin_z_ = std::stof(vals[3]);

        orientation_w_ = std::stof(vals[4]);
        orientation_x_ = std::stof(vals[5]);
        orientation_y_ = std::stof(vals[6]);
        orientation_z_ = std::stof(vals[7]);

        continue;
      }

      if (vals[0] == "POINTS")
      {
        point_num_ = std::stoi(vals[1]);

        continue;
      }

      if (vals[0] == "DATA")
      {
        binary_ = vals[1].find("binary") != std::string::npos;

        break;
      }

      // After finish parsing fields, break
      // TODO: Roll back to the previous position
      break;
    }
  }
  catch (...)
  {
    fprintf(stderr, "[%s, %d] %s::Error: Failed to read the PCD header from file %s\n", __FILE__, __LINE__, __func__,
            pcd_path_.c_str());
    exit(EXIT_FAILURE);
  }

  // Compute the number of bytes per reading
  point_size_ = read_size_ = 0;

  for (size_t i = 0; i < field_sizes_.size(); ++i)
  {
    point_size_ += field_sizes_[i];
  }

  read_size_ = point_size_ * block_size_;

  if (buffer_)
  {
    delete[] buffer_;
  }

  buffer_ = new char[read_size_];

  if (field_sizes_.size() > 0)
  {
    // Construct read loc and read size, used to read data from files to points
    buildReadMetadata<PointT>(field_names_, field_sizes_, read_loc_, read_sizes_);
  }
}

inline void setFieldReadMetadata(const std::string& field_tag, std::vector<std::string>& field_names,
                                 std::vector<size_t>& field_sizes, std::vector<size_t>& tmp_read_loc, size_t& read_loc,
                                 size_t& read_size)
{
  for (size_t i = 0; i < field_names.size(); ++i)
  {
    if (field_names[i].find(field_tag) != std::string::npos)
    {
      read_loc = tmp_read_loc[i];
      read_size = field_sizes[i];

      return;
    }
  }

  // If the field tag was not found, set both read_loc and read_size to 0
  std::cerr << "Field " << field_tag << " is not available, and will be set to 0." << std::endl;
  read_loc = 0;
  read_size = 0;
}

template <>
inline void buildReadMetadata<pcl::PointXYZ>(std::vector<std::string>& field_names, std::vector<size_t>& field_sizes,
                                             std::vector<size_t>& read_loc, std::vector<size_t>& read_sizes)
{
  size_t field_num = field_names.size();

  read_loc.resize(3);
  read_sizes.resize(3);

  std::vector<size_t> tmp_read_loc(field_num);

  tmp_read_loc[0] = 0;

  for (size_t i = 0; i < field_num - 1; ++i)
  {
    tmp_read_loc[i + 1] = field_sizes[i] + tmp_read_loc[i];
  }

  // Find field x, y, z
  setFieldReadMetadata("x", field_names, field_sizes, tmp_read_loc, read_loc[0], read_sizes[0]);
  setFieldReadMetadata("y", field_names, field_sizes, tmp_read_loc, read_loc[1], read_sizes[1]);
  setFieldReadMetadata("z", field_names, field_sizes, tmp_read_loc, read_loc[2], read_sizes[2]);
}

template <>
inline void buildReadMetadata<pcl::PointXYZI>(std::vector<std::string>& field_names, std::vector<size_t>& field_sizes,
                                              std::vector<size_t>& read_loc, std::vector<size_t>& read_sizes)
{
  size_t field_num = field_names.size();

  read_loc.resize(4);
  read_sizes.resize(4);

  std::vector<size_t> tmp_read_loc(field_num);

  tmp_read_loc[0] = 0;

  for (size_t i = 0; i < field_num - 1; ++i)
  {
    tmp_read_loc[i + 1] = field_sizes[i] + tmp_read_loc[i];
  }

  // Find field x, y, z, intensity
  setFieldReadMetadata("x", field_names, field_sizes, tmp_read_loc, read_loc[0], read_sizes[0]);
  setFieldReadMetadata("y", field_names, field_sizes, tmp_read_loc, read_loc[1], read_sizes[1]);
  setFieldReadMetadata("z", field_names, field_sizes, tmp_read_loc, read_loc[2], read_sizes[2]);
  setFieldReadMetadata("intensity", field_names, field_sizes, tmp_read_loc, read_loc[3], read_sizes[3]);
}

template <typename PointT>
inline void parsePoint(const char* input, std::vector<size_t>& rsize, std::vector<size_t>& loc, PointT& output);

template <>
inline void parsePoint(const char* input, std::vector<size_t>& rsize, std::vector<size_t>& loc, pcl::PointXYZ& output)
{
  memcpy(&(output.x), input + loc[0], rsize[0]);
  memcpy(&(output.y), input + loc[1], rsize[1]);
  memcpy(&(output.z), input + loc[2], rsize[2]);
}

template <>
inline void parsePoint(const char* input, std::vector<size_t>& rsize, std::vector<size_t>& loc, pcl::PointXYZI& output)
{
  memcpy(&(output.x), input + loc[0], rsize[0]);
  memcpy(&(output.y), input + loc[1], rsize[1]);
  memcpy(&(output.z), input + loc[2], rsize[2]);
  memcpy(&(output.intensity), input + loc[3], rsize[3]);
}

template <typename PointT>
size_t CustomPCDReader<PointT>::readABlockBinary(std::ifstream& input, PclCloudType& output)
{
  output.clear();
  output.reserve(block_size_);

  PointT p;

  memset(&p, 0x00, sizeof(p));

  if (input)
  {
    input.read(buffer_, read_size_);

    // Parse the buffer and convert to point
    for (size_t i = 0; i < input.gcount(); i += point_size_)
    {
      parsePoint(buffer_ + i, read_sizes_, read_loc_, p);
      output.push_back(p);
    }

    return input.gcount();
  }

  return 0;
}

template <typename PointT>
void parsePoint(const std::string& point_line, PointT& output);

template <>
inline void parsePoint(const std::string& point_line, pcl::PointXYZ& output)
{
  std::vector<std::string> vals;

  util::split(point_line, " ", vals);
  output.x = std::stof(vals[0]);
  output.y = std::stof(vals[1]);
  output.z = std::stof(vals[2]);
}

template <>
inline void parsePoint(const std::string& point_line, pcl::PointXYZI& output)
{
  std::vector<std::string> vals;

  util::split(point_line, " ", vals);
  output.x = std::stof(vals[0]);
  output.y = std::stof(vals[1]);
  output.z = std::stof(vals[2]);
  output.intensity = (vals.size() < 4) ? 0 : std::stof(vals[3]);
}

template <typename PointT>
size_t CustomPCDReader<PointT>::readABlockASCII(std::ifstream& input, PclCloudType& output)
{
  output.clear();
  output.reserve(block_size_);

  std::string point_line;

  PointT p;

  memset(&p, 0x00, sizeof(p));

  while (input)
  {
    std::getline(input, point_line);

    if (!input.fail())
    {
      // Parse the buffer and convert to point
      parsePoint(point_line, p);
      output.push_back(p);
    }
    else
    {
      fprintf(stderr, "[%s, %d] %s::Error: Failed to read a block of points from file. File %s\n", __FILE__, __LINE__,
              __func__, pcd_path_.c_str());
      exit(EXIT_FAILURE);
    }

    return input.gcount();
  }

  return 0;
}

template <typename PointT>
size_t CustomPCDReader<PointT>::readABlock(std::ifstream& input, PclCloudType& output)
{
  if (binary_)
  {
    return readABlockBinary(input, output);
  }

  return readABlockASCII(input, output);
}

#endif