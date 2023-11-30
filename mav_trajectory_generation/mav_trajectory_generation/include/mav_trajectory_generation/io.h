#ifndef MAV_TRAJECTORY_GENERATION_YAML_IO_H_
#define MAV_TRAJECTORY_GENERATION_YAML_IO_H_

#include <yaml-cpp/yaml.h>

#include "mav_trajectory_generation/segment_mgv.h"
#include "mav_trajectory_generation/trajectory_mgv.h"
#include "segment_mgv.h"

namespace mgv_trajectory_generation {

YAML::Node coefficientsToYaml(const Eigen::VectorXd& coefficients);
YAML::Node segmentToYaml(const Segment& segment);
YAML::Node segmentsToYaml(const Segment::Vector& segments);
YAML::Node trajectoryToYaml(const Trajectory& trajectory);

bool coefficientsFromYaml(const YAML::Node& node,
                          Eigen::VectorXd* coefficients);
bool segmentFromYaml(const YAML::Node& node, Segment* segment);
bool segmentsFromYaml(const YAML::Node& node, Segment::Vector* segments);
bool trajectoryFromYaml(const YAML::Node& node, Trajectory* trajectory);

bool segmentsToFile(const std::string& filename,
                    const mgv_trajectory_generation::Segment::Vector& segments);

inline bool trajectoryToFile(
    const std::string& filename,
    const mgv_trajectory_generation::Trajectory& trajectory) {
  mgv_trajectory_generation::Segment::Vector segments;
  trajectory.getSegments(&segments);
  return segmentsToFile(filename, segments);
}

bool segmentsFromFile(const std::string& filename,
                      mgv_trajectory_generation::Segment::Vector* segments);

inline bool trajectoryFromFile(
    const std::string& filename,
    mgv_trajectory_generation::Trajectory* trajectory) {
  mgv_trajectory_generation::Segment::Vector segments;
  bool success = segmentsFromFile(filename, &segments);
  trajectory->setSegments(segments);
  return success;
}

bool sampledTrajectoryStatesToFile(const std::string& filename,
                                   const Trajectory& trajectory);

}  // namespace mgv_trajectory_generation

#endif  // MAV_TRAJECTORY_GENERATION_YAML_IO_H_
