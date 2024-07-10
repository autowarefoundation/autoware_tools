#ifndef GRID_INFO_HPP_
#define GRID_INFO_HPP_

#include <iostream>
#include <string>
#include <tuple>

template <int dim>
struct GridInfo
{
  int ix, iy, iz;

  GridInfo() : ix(0), iy(0), iz(0)
  {
  }

  GridInfo(int x, int y, int z = 0) : ix(x), iy(y), iz(z)
  {
  }

  friend bool operator==(const GridInfo& one, const GridInfo& other)
  {
    return one.ix == other.ix && one.iy == other.iy && one.iz == other.iz;
  }

  friend bool operator!=(const GridInfo& one, const GridInfo& other)
  {
    return !(one == other);
  }

  friend std::ostream& operator<<(std::ostream& os, const GridInfo& g)
  {
    if (dim < 3)
    {
      os << g.ix << "_" << g.iy;
    }
    else
    {
      os << g.ix << "_" << g.iy << "_" << g.iz;
    }

    return os;
  }
};

// This is for unordered_map and unodered_set
namespace std
{
template <>
struct hash<GridInfo<2>>
{
public:
  size_t operator()(const GridInfo<2>& grid) const
  {
    std::size_t seed = 0;
    seed ^= std::hash<int>{}(grid.ix) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(grid.iy) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

template <>
struct hash<GridInfo<3>>
{
public:
  size_t operator()(const GridInfo<3>& grid) const
  {
    std::size_t seed = 0;

    seed ^= std::hash<int>{}(grid.ix) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(grid.iy) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(grid.iz) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

    return seed;
  }
};

}  // namespace std

template <typename PointT>
GridInfo<2> pointToGrid2(const PointT& p, double res_x, double res_y)
{
  int x_id = static_cast<int>(std::floor(p.x / res_x) * res_x);
  int y_id = static_cast<int>(std::floor(p.y / res_y) * res_y);

  return GridInfo<2>(x_id, y_id);
}

template <typename PointT>
GridInfo<3> pointToGrid3(const PointT& p, double res_x, double res_y, double res_z)
{
  int x_id = static_cast<int>(std::floor(p.x / res_x));
  int y_id = static_cast<int>(std::floor(p.y / res_y));
  int z_id = static_cast<int>(std::floor(p.z / res_z));

  return GridInfo<3>(x_id, y_id, z_id);
}

#endif