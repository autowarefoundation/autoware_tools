#include <pointcloud_divider/pointcloud_divider.hpp>

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

  if (argc != 5 + n_pcd) {
    printErrorAndExit(
      "There should be " + std::to_string(5 + n_pcd) +
      " runtime arguments. input: " + std::to_string(argc));
  }

  std::vector<std::string> pcd_name;
  for (int pcd_id = 0; pcd_id < n_pcd; pcd_id++) {
    pcd_name.push_back(argv[2 + pcd_id]);
  }
  const std::string output_dir = argv[n_pcd + 2];
  const std::string prefix = argv[n_pcd + 3];
  const std::string config = argv[n_pcd + 4];

  // Currently, any PCD will be loaded as pcl::PointXYZI.
  PointCloudDivider<pcl::PointXYZI> divider;
  divider.run(pcd_name, output_dir, prefix, config);

  std::cout << "pointcloud_divider has finished successfully" << std::endl;
  return 0;
}
