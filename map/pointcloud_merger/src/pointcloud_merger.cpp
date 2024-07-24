// BSD 3-Clause License
//
// Copyright (c) 2023, MAP IV
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <pointcloud_merger/pointcloud_merger.hpp>

#include <pcl/console/print.h>

void printErrorAndExit(const std::string & message)
{
  std::cerr << "Error: " << message << std::endl;
  exit(1);
}

int main(int argc, char * argv[])
{
  // Change the default PCL's log level to suppress the following message:
  // `Failed to find match for field 'intensity'.`
  pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ERROR);

  if (argc <= 1) {
    printErrorAndExit("There should be at least 6 runtime arguments.");
  }

  const int n_pcd = std::stoi(argv[1]);

  if (argc != 4 + n_pcd) {
    printErrorAndExit(
      "There should be " + std::to_string(4 + n_pcd) +
      " runtime arguments. input: " + std::to_string(argc));
  }

  std::vector<std::string> pcd_name;
  for (int pcd_id = 0; pcd_id < n_pcd; pcd_id++) {
    pcd_name.push_back(argv[2 + pcd_id]);
  }
  const std::string output_pcd = argv[n_pcd + 2];
  const std::string config = argv[n_pcd + 3];

  // Currently, any PCD will be loaded as pcl::PointXYZI.
  PointCloudMerger<pcl::PointXYZI> divider;
  divider.run(pcd_name, output_pcd, config);

  std::cout << "pointcloud_merger has finished successfully" << std::endl;
  return 0;
}
