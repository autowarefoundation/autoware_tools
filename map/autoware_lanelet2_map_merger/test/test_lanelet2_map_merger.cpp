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

#include "../src/lanelet2_map_merger.hpp"
#include "../src/local_projector.hpp"

#include <rclcpp/logger.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{

// Build a tiny two-lanelet map that reuses two border points so that writing it
// into separate cells exercises the merger's point-deduplication path.
lanelet::LaneletMapPtr make_sample_map()
{
  auto make_point = [](lanelet::Id id, double x, double y) {
    lanelet::Point3d p(id, x, y, 0.0);
    p.attributes()["local_x"] = std::to_string(x);
    p.attributes()["local_y"] = std::to_string(y);
    return p;
  };

  // Lanelet 100 in cell (0,0); lanelet 101 in cell (10,0). They share left border
  // points (points 3, 4) because right border of 100 == left border of 101.
  lanelet::Point3d p1 = make_point(1, 1.0, 1.0);
  lanelet::Point3d p2 = make_point(2, 1.0, 9.0);
  lanelet::Point3d p3 = make_point(3, 9.0, 1.0);
  lanelet::Point3d p4 = make_point(4, 9.0, 9.0);
  lanelet::Point3d p5 = make_point(5, 19.0, 1.0);
  lanelet::Point3d p6 = make_point(6, 19.0, 9.0);

  lanelet::LineString3d ls_left_a(10, {p1, p2});
  lanelet::LineString3d ls_right_a(11, {p3, p4});  // shared with lanelet 101
  lanelet::LineString3d ls_right_b(12, {p5, p6});
  for (auto ls : {ls_left_a, ls_right_a, ls_right_b}) {
    ls.attributes()["type"] = "line_thin";
    ls.attributes()["subtype"] = "solid";
  }

  lanelet::Lanelet ll_a(100, ls_left_a, ls_right_a);
  lanelet::Lanelet ll_b(101, ls_right_a, ls_right_b);
  for (auto ll : {ll_a, ll_b}) {
    ll.attributes()["type"] = "lanelet";
    ll.attributes()["subtype"] = "road";
    ll.attributes()["location"] = "urban";
    ll.attributes()["one_way"] = "yes";
  }

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(ll_a);
  map->add(ll_b);
  return map;
}

std::string write_projector_info_local(const fs::path & dir)
{
  const auto path = dir / "map_projector_info.yaml";
  std::ofstream(path) << "projector_type: Local\n";
  return path.string();
}

// Write the given map into N .osm files in @dir by splitting lanelets one-per-file,
// simulating the output of a divider.
void write_split_maps(lanelet::LaneletMap & map, const fs::path & dir)
{
  fs::create_directories(dir);
  autoware::lanelet2_map_merger::LocalProjector projector;

  size_t idx = 0;
  for (lanelet::Lanelet & llt : map.laneletLayer) {
    auto sub = std::make_shared<lanelet::LaneletMap>();
    sub->add(llt);
    const auto path = dir / (std::to_string(idx++) + ".osm");
    lanelet::write(path.string(), *sub, projector);
  }
}

// Count how many <node>, <way>, <relation> primitives appear in an .osm file.
struct OsmCounts
{
  size_t nodes = 0;
  size_t ways = 0;
  size_t relations = 0;
};
OsmCounts count_osm_primitives(const std::string & path)
{
  OsmCounts c;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.find("<node ") != std::string::npos) ++c.nodes;
    if (line.find("<way ") != std::string::npos) ++c.ways;
    if (line.find("<relation ") != std::string::npos) ++c.relations;
  }
  return c;
}

}  // namespace

class Lanelet2MapMergerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = fs::temp_directory_path() / "test_lanelet2_map_merger";
    fs::remove_all(tmp_dir_);
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override { fs::remove_all(tmp_dir_); }

  fs::path tmp_dir_;
};

TEST_F(Lanelet2MapMergerTest, MergesSplitMapsIntoSingleFile)
{
  const auto map = make_sample_map();
  const fs::path input_dir = tmp_dir_ / "in";
  write_split_maps(*map, input_dir);

  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_path = (tmp_dir_ / "merged.osm").string();

  autoware::lanelet2_map_merger::Lanelet2MapMerger merger(rclcpp::get_logger("test"));
  merger.set_input_dir(input_dir.string());
  merger.set_output(output_path);
  merger.set_map_projector_info_path(projector_info);
  merger.run();

  ASSERT_TRUE(fs::exists(output_path));
}

TEST_F(Lanelet2MapMergerTest, MergedOutputDeduplicatesSharedPoints)
{
  const auto map = make_sample_map();
  const fs::path input_dir = tmp_dir_ / "in";
  write_split_maps(*map, input_dir);

  // Sanity-check: across the split files, a shared linestring's points appear twice.
  size_t total_nodes_in_inputs = 0;
  for (const auto & entry : fs::directory_iterator(input_dir)) {
    total_nodes_in_inputs += count_osm_primitives(entry.path().string()).nodes;
  }
  // Points 3 and 4 are shared → 6 unique nodes * 2 files where they appear in one = 8.
  EXPECT_EQ(total_nodes_in_inputs, 8u);

  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_path = (tmp_dir_ / "merged.osm").string();

  autoware::lanelet2_map_merger::Lanelet2MapMerger merger(rclcpp::get_logger("test"));
  merger.set_input_dir(input_dir.string());
  merger.set_output(output_path);
  merger.set_map_projector_info_path(projector_info);
  merger.run();

  const auto counts = count_osm_primitives(output_path);
  // After deduplication, exactly 6 unique points and 3 unique line strings remain;
  // 2 lanelets produce 2 relations.
  EXPECT_EQ(counts.nodes, 6u);
  EXPECT_EQ(counts.ways, 3u);
  EXPECT_EQ(counts.relations, 2u);
}

TEST_F(Lanelet2MapMergerTest, MergedMapCanBeReloaded)
{
  const auto map = make_sample_map();
  const fs::path input_dir = tmp_dir_ / "in";
  write_split_maps(*map, input_dir);

  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_path = (tmp_dir_ / "merged.osm").string();

  autoware::lanelet2_map_merger::Lanelet2MapMerger merger(rclcpp::get_logger("test"));
  merger.set_input_dir(input_dir.string());
  merger.set_output(output_path);
  merger.set_map_projector_info_path(projector_info);
  merger.run();

  // Reload the merged file via lanelet2's IO and inspect layer sizes.
  autoware::lanelet2_map_merger::LocalProjector projector;
  lanelet::ErrorMessages errors;
  auto reloaded = lanelet::load(output_path, projector, &errors);
  ASSERT_TRUE(errors.empty()) << "Lanelet2 reported load errors on merged output";
  ASSERT_TRUE(reloaded);
  EXPECT_EQ(reloaded->laneletLayer.size(), 2u);
  EXPECT_EQ(reloaded->lineStringLayer.size(), 3u);
  EXPECT_EQ(reloaded->pointLayer.size(), 6u);
}

TEST_F(Lanelet2MapMergerTest, NonExistentInputDirDoesNotCrash)
{
  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_path = (tmp_dir_ / "merged.osm").string();

  autoware::lanelet2_map_merger::Lanelet2MapMerger merger(rclcpp::get_logger("test"));
  merger.set_input_dir((tmp_dir_ / "nope").string());
  merger.set_output(output_path);
  merger.set_map_projector_info_path(projector_info);

  EXPECT_NO_THROW(merger.run());
  EXPECT_FALSE(fs::exists(output_path));
}

TEST_F(Lanelet2MapMergerTest, EmptyInputDirDoesNotCrash)
{
  const fs::path input_dir = tmp_dir_ / "empty";
  fs::create_directories(input_dir);

  const auto projector_info = write_projector_info_local(tmp_dir_);
  const auto output_path = (tmp_dir_ / "merged.osm").string();

  autoware::lanelet2_map_merger::Lanelet2MapMerger merger(rclcpp::get_logger("test"));
  merger.set_input_dir(input_dir.string());
  merger.set_output(output_path);
  merger.set_map_projector_info_path(projector_info);

  EXPECT_NO_THROW(merger.run());
  EXPECT_FALSE(fs::exists(output_path));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
