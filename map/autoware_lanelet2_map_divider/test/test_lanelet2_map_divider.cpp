// Copyright 2026 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../src/lanelet2_map_divider.hpp"
#include "../src/local_projector.hpp"

#include <rclcpp/logger.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

// Build a small LaneletMap with four lanelets arranged on a 2x2 grid of 10m cells.
// Each point carries `local_x`/`local_y` attributes so that the LOCAL projector in the
// divider can recover its original x/y values on load.
lanelet::LaneletMapPtr make_sample_map()
{
  auto make_point = [](lanelet::Id id, double x, double y) {
    lanelet::Point3d p(id, x, y, 0.0);
    p.attributes()["local_x"] = std::to_string(x);
    p.attributes()["local_y"] = std::to_string(y);
    return p;
  };

  // Four 10m x 10m lanelets tiled at (0..20) x (0..20) so a 10m grid divider produces
  // four cells: (0,0), (10,0), (0,10), (10,10).
  struct Cell
  {
    lanelet::Id ll_id, left_id, right_id;
    double x0, y0;  // lower-left corner
  };
  const std::vector<Cell> cells = {
    {100, 10, 11, 0.0, 0.0},
    {101, 12, 13, 10.0, 0.0},
    {102, 14, 15, 0.0, 10.0},
    {103, 16, 17, 10.0, 10.0},
  };

  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id next_pt_id = 1;
  for (const auto & c : cells) {
    // Left border (bottom-left → top-left of cell)
    auto pl1 = make_point(next_pt_id++, c.x0 + 1.0, c.y0 + 1.0);
    auto pl2 = make_point(next_pt_id++, c.x0 + 1.0, c.y0 + 9.0);
    lanelet::LineString3d left(c.left_id, {pl1, pl2});
    left.attributes()["type"] = "line_thin";
    left.attributes()["subtype"] = "solid";

    // Right border (bottom-right → top-right)
    auto pr1 = make_point(next_pt_id++, c.x0 + 9.0, c.y0 + 1.0);
    auto pr2 = make_point(next_pt_id++, c.x0 + 9.0, c.y0 + 9.0);
    lanelet::LineString3d right(c.right_id, {pr1, pr2});
    right.attributes()["type"] = "line_thin";
    right.attributes()["subtype"] = "solid";

    lanelet::Lanelet llt(c.ll_id, left, right);
    llt.attributes()["type"] = "lanelet";
    llt.attributes()["subtype"] = "road";
    llt.attributes()["location"] = "urban";
    llt.attributes()["one_way"] = "yes";
    map->add(llt);
  }
  return map;
}

std::string write_projector_info_local(const fs::path & dir)
{
  const auto path = dir / "map_projector_info.yaml";
  std::ofstream(path) << "projector_type: Local\n";
  return path.string();
}

std::string write_sample_osm(const fs::path & dir, const std::string & filename = "input.osm")
{
  const auto osm_path = (dir / filename).string();
  auto map = make_sample_map();
  autoware::lanelet2_map_divider::LocalProjector projector;
  lanelet::write(osm_path, *map, projector);
  return osm_path;
}

// Parse the output metadata YAML into a map: filename -> {min_x, min_y}.
struct CellMeta
{
  double x;
  double y;
};
std::map<std::string, CellMeta> parse_metadata(const std::string & yaml_path)
{
  std::map<std::string, CellMeta> out;
  const YAML::Node root = YAML::LoadFile(yaml_path);
  for (const auto & kv : root) {
    const std::string key = kv.first.as<std::string>();
    if (key == "x_resolution" || key == "y_resolution") continue;
    out[key] = {kv.second[0].as<double>(), kv.second[1].as<double>()};
  }
  return out;
}

}  // namespace

class Lanelet2MapDividerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = fs::temp_directory_path() / "test_lanelet2_map_divider";
    fs::remove_all(tmp_dir_);
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override { fs::remove_all(tmp_dir_); }

  fs::path tmp_dir_;
};

TEST_F(Lanelet2MapDividerTest, SingleFileInputProducesFourCells)
{
  const auto input_osm = write_sample_osm(tmp_dir_);
  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_dir = (tmp_dir_ / "out").string();

  autoware::lanelet2_map_divider::Lanelet2MapDivider divider(rclcpp::get_logger("test"));
  divider.set_input(input_osm);
  divider.set_output_dir(output_dir);
  divider.set_map_projector_info_path(projector_info);
  divider.set_grid_size(10.0, 10.0);
  divider.set_prefix("");
  divider.run();

  // Cell directory and four .osm files must exist.
  const auto cell_dir = fs::path(output_dir) / "lanelet2_map.osm";
  ASSERT_TRUE(fs::exists(cell_dir));

  std::set<std::string> cell_files;
  for (const auto & e : fs::directory_iterator(cell_dir)) {
    cell_files.insert(e.path().filename().string());
  }
  EXPECT_EQ(cell_files.size(), 4u);
  EXPECT_TRUE(cell_files.count("0_0.osm"));
  EXPECT_TRUE(cell_files.count("10_0.osm"));
  EXPECT_TRUE(cell_files.count("0_10.osm"));
  EXPECT_TRUE(cell_files.count("10_10.osm"));
}

TEST_F(Lanelet2MapDividerTest, MetadataYamlMatchesOutputFiles)
{
  const auto input_osm = write_sample_osm(tmp_dir_);
  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_dir = (tmp_dir_ / "out").string();

  autoware::lanelet2_map_divider::Lanelet2MapDivider divider(rclcpp::get_logger("test"));
  divider.set_input(input_osm);
  divider.set_output_dir(output_dir);
  divider.set_map_projector_info_path(projector_info);
  divider.set_grid_size(10.0, 10.0);
  divider.set_prefix("");
  divider.run();

  const auto metadata_path = (fs::path(output_dir) / "lanelet2_map_metadata.yaml").string();
  ASSERT_TRUE(fs::exists(metadata_path));

  const YAML::Node root = YAML::LoadFile(metadata_path);
  EXPECT_DOUBLE_EQ(root["x_resolution"].as<double>(), 10.0);
  EXPECT_DOUBLE_EQ(root["y_resolution"].as<double>(), 10.0);

  const auto cells = parse_metadata(metadata_path);
  ASSERT_EQ(cells.size(), 4u);
  EXPECT_DOUBLE_EQ(cells.at("0_0.osm").x, 0.0);
  EXPECT_DOUBLE_EQ(cells.at("0_0.osm").y, 0.0);
  EXPECT_DOUBLE_EQ(cells.at("10_0.osm").x, 10.0);
  EXPECT_DOUBLE_EQ(cells.at("10_0.osm").y, 0.0);
  EXPECT_DOUBLE_EQ(cells.at("0_10.osm").x, 0.0);
  EXPECT_DOUBLE_EQ(cells.at("0_10.osm").y, 10.0);
  EXPECT_DOUBLE_EQ(cells.at("10_10.osm").x, 10.0);
  EXPECT_DOUBLE_EQ(cells.at("10_10.osm").y, 10.0);
}

TEST_F(Lanelet2MapDividerTest, PrefixIsAppliedToOutputFilenames)
{
  const auto input_osm = write_sample_osm(tmp_dir_);
  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_dir = (tmp_dir_ / "out").string();

  autoware::lanelet2_map_divider::Lanelet2MapDivider divider(rclcpp::get_logger("test"));
  divider.set_input(input_osm);
  divider.set_output_dir(output_dir);
  divider.set_map_projector_info_path(projector_info);
  divider.set_grid_size(10.0, 10.0);
  divider.set_prefix("tile");
  divider.run();

  const auto cell_dir = fs::path(output_dir) / "lanelet2_map.osm";
  std::set<std::string> cell_files;
  for (const auto & e : fs::directory_iterator(cell_dir)) {
    cell_files.insert(e.path().filename().string());
  }
  EXPECT_EQ(cell_files.size(), 4u);
  EXPECT_TRUE(cell_files.count("tile_0_0.osm"));
  EXPECT_TRUE(cell_files.count("tile_10_0.osm"));
  EXPECT_TRUE(cell_files.count("tile_0_10.osm"));
  EXPECT_TRUE(cell_files.count("tile_10_10.osm"));
}

TEST_F(Lanelet2MapDividerTest, GridIsCoordinateAligned)
{
  // A 10m grid applied to a map with points starting at x=1,y=1 should still emit
  // the (0,0) cell, not a (1,1) cell. This is the defining property of coord-aligned
  // indices.
  const auto input_osm = write_sample_osm(tmp_dir_);
  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_dir = (tmp_dir_ / "out").string();

  autoware::lanelet2_map_divider::Lanelet2MapDivider divider(rclcpp::get_logger("test"));
  divider.set_input(input_osm);
  divider.set_output_dir(output_dir);
  divider.set_map_projector_info_path(projector_info);
  divider.set_grid_size(10.0, 10.0);
  divider.run();

  const auto metadata_path = (fs::path(output_dir) / "lanelet2_map_metadata.yaml").string();
  const auto cells = parse_metadata(metadata_path);

  // Every cell's min_x and min_y must be an exact multiple of the grid size.
  for (const auto & [name, c] : cells) {
    EXPECT_DOUBLE_EQ(std::fmod(c.x, 10.0), 0.0) << name;
    EXPECT_DOUBLE_EQ(std::fmod(c.y, 10.0), 0.0) << name;
  }
}

TEST_F(Lanelet2MapDividerTest, LargeGridSizeProducesSingleCell)
{
  const auto input_osm = write_sample_osm(tmp_dir_);
  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_dir = (tmp_dir_ / "out").string();

  // With a 100m grid, all points (in 0..20) fall into the single (0,0) cell.
  autoware::lanelet2_map_divider::Lanelet2MapDivider divider(rclcpp::get_logger("test"));
  divider.set_input(input_osm);
  divider.set_output_dir(output_dir);
  divider.set_map_projector_info_path(projector_info);
  divider.set_grid_size(100.0, 100.0);
  divider.run();

  const auto cell_dir = fs::path(output_dir) / "lanelet2_map.osm";
  std::vector<std::string> files;
  for (const auto & e : fs::directory_iterator(cell_dir)) {
    files.push_back(e.path().filename().string());
  }
  ASSERT_EQ(files.size(), 1u);
  EXPECT_EQ(files.front(), "0_0.osm");
}

TEST_F(Lanelet2MapDividerTest, DirectoryInputIsRejected)
{
  // Users with a split map are expected to merge it first; the divider only
  // accepts a single .osm file.
  const fs::path input_dir = tmp_dir_ / "input_dir";
  fs::create_directories(input_dir);
  auto map = make_sample_map();
  autoware::lanelet2_map_divider::LocalProjector projector;
  lanelet::write((input_dir / "a.osm").string(), *map, projector);

  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_dir = (tmp_dir_ / "out").string();

  autoware::lanelet2_map_divider::Lanelet2MapDivider divider(rclcpp::get_logger("test"));
  divider.set_input(input_dir.string());
  divider.set_output_dir(output_dir);
  divider.set_map_projector_info_path(projector_info);
  divider.set_grid_size(10.0, 10.0);

  EXPECT_NO_THROW(divider.run());
  EXPECT_FALSE(fs::exists(output_dir));
}

TEST_F(Lanelet2MapDividerTest, NonExistentInputDoesNotCrash)
{
  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_dir = (tmp_dir_ / "out").string();

  autoware::lanelet2_map_divider::Lanelet2MapDivider divider(rclcpp::get_logger("test"));
  divider.set_input((tmp_dir_ / "does_not_exist.osm").string());
  divider.set_output_dir(output_dir);
  divider.set_map_projector_info_path(projector_info);
  divider.set_grid_size(10.0, 10.0);

  // Should log errors and return without throwing.
  EXPECT_NO_THROW(divider.run());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
