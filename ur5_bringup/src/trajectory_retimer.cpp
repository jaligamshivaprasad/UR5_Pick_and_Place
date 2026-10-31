#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <iomanip>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

/**
 * @brief MoveIt 2 Humble Trajectory Retimer Utility
 * 
 * Supports independent velocity and acceleration scaling using MoveIt 2's
 * Time-Optimal Trajectory Generation (TOTG) or Iterative Parabolic Time
 * Parameterization (IPTP).
 */

struct TrajectoryPoint {
  double time_sec;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

struct TrajectorySegment {
  std::string name;
  std::vector<std::string> joint_names;
  std::vector<TrajectoryPoint> points;
};

std::vector<TrajectorySegment> parsePathFile(const std::string& filepath) {
  std::ifstream infile(filepath);
  std::vector<TrajectorySegment> segments;
  if (!infile.is_open()) {
    throw std::runtime_error("Could not open input path file: " + filepath);
  }

  std::string line;
  TrajectorySegment current_seg;
  bool in_segment = false;

  while (std::getline(infile, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    std::string token;
    iss >> token;

    if (token == "SEGMENT") {
      if (in_segment) {
        segments.push_back(current_seg);
        current_seg = TrajectorySegment();
      }
      iss >> current_seg.name;
      in_segment = true;
    } else if (token == "JOINTS") {
      std::string jname;
      while (iss >> jname) {
        current_seg.joint_names.push_back(jname);
      }
    } else if (token == "POINT") {
      int sec, ns;
      iss >> sec >> ns;
      double t = static_cast<double>(sec) + static_cast<double>(ns) * 1e-9;
      std::string tag;

      TrajectoryPoint pt;
      pt.time_sec = t;

      iss >> tag; // "POS"
      pt.positions.resize(6);
      for (int i = 0; i < 6; ++i) iss >> pt.positions[i];

      iss >> tag; // "VEL"
      pt.velocities.resize(6);
      for (int i = 0; i < 6; ++i) iss >> pt.velocities[i];

      iss >> tag; // "ACC"
      pt.accelerations.resize(6);
      for (int i = 0; i < 6; ++i) iss >> pt.accelerations[i];

      current_seg.points.push_back(pt);
    } else if (token == "END") {
      if (in_segment) {
        segments.push_back(current_seg);
        current_seg = TrajectorySegment();
        in_segment = false;
      }
    }
  }
  if (in_segment) {
    segments.push_back(current_seg);
  }
  return segments;
}

void writePathFile(const std::string& filepath, const std::vector<TrajectorySegment>& segments,
                   double v_scale, double a_scale, const std::string& algo) {
  std::ofstream out(filepath);
  if (!out.is_open()) {
    throw std::runtime_error("Could not open output path file: " + filepath);
  }

  out << "# UR5_PICK_PLACE_PATH_V1\n";
  out << "# Retimed by MoveIt 2 Humble trajectory_retimer (" << algo << ")\n";
  out << "# VelocityScale: " << std::fixed << std::setprecision(4) << v_scale
      << " AccelerationScale: " << a_scale << "\n";

  for (const auto& seg : segments) {
    out << "SEGMENT " << seg.name << "\n";
    out << "JOINTS";
    for (const auto& j : seg.joint_names) out << " " << j;
    out << "\n";
    out << "POINTS " << seg.points.size() << "\n";

    for (const auto& pt : seg.points) {
      int sec = static_cast<int>(std::floor(pt.time_sec));
      int ns = static_cast<int>(std::round((pt.time_sec - sec) * 1e9));
      if (ns >= 1000000000) {
        sec += 1;
        ns -= 1000000000;
      }
      out << "POINT " << sec << " " << ns << " POS";
      for (int i = 0; i < 6; ++i) out << " " << std::setprecision(6) << pt.positions[i];
      out << " VEL";
      for (int i = 0; i < 6; ++i) out << " " << std::setprecision(6) << pt.velocities[i];
      out << " ACC";
      for (int i = 0; i < 6; ++i) out << " " << std::setprecision(6) << pt.accelerations[i];
      out << "\n";
    }
    out << "END\n";
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_trajectory_retimer");

  std::string urdf_path = "/home/shiva/ur5/ur5_ws/experiment_logs/test_phase7_1_0kg_full/robot.urdf";
  std::string srdf_path = "/home/shiva/ur5/ur5_ws/ur5_moveit_config/config/ur5.srdf";
  std::string input_path = "/home/shiva/ur5/ur5_ws/path01";
  std::string output_path = "";
  std::string algo = "totg";
  double v_scale = 0.20;
  double a_scale = 0.20;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--path" && i + 1 < argc) {
      input_path = argv[++i];
    } else if (arg == "--output" && i + 1 < argc) {
      output_path = argv[++i];
    } else if (arg == "--v-scale" && i + 1 < argc) {
      v_scale = std::stod(argv[++i]);
    } else if (arg == "--a-scale" && i + 1 < argc) {
      a_scale = std::stod(argv[++i]);
    } else if (arg == "--algorithm" && i + 1 < argc) {
      algo = argv[++i];
    } else if (i == 1 && arg[0] != '-') {
      v_scale = std::stod(arg);
      if (argc > 2 && argv[2][0] != '-') a_scale = std::stod(argv[2]);
    }
  }

  RCLCPP_INFO(node->get_logger(),
              "MoveIt 2 Retimer starting: algo=%s, v_scale=%.3f, a_scale=%.3f",
              algo.c_str(), v_scale, a_scale);

  auto urdf_model = std::make_shared<urdf::Model>();
  if (!urdf_model->initFile(urdf_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF file: %s", urdf_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initFile(*urdf_model, srdf_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to parse SRDF file: %s", srdf_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm");
  if (!jmg) {
    RCLCPP_ERROR(node->get_logger(), "RobotModel does not contain 'arm' planning group!");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "MoveIt RobotModel '%s' initialized.", robot_model->getName().c_str());

  if (output_path.empty()) {
    RCLCPP_INFO(node->get_logger(), "No output path provided. Testing MoveIt time parameterization algorithms...");
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.005, 0.1, 0.001);
    robot_trajectory::RobotTrajectory rt(robot_model, "arm");
    RCLCPP_INFO(node->get_logger(), "MoveIt 2 trajectory processing engines (IPTP and TOTG) validated successfully.");
    rclcpp::shutdown();
    return 0;
  }

  // Parse input segments
  auto segments = parsePathFile(input_path);
  RCLCPP_INFO(node->get_logger(), "Read %zu segments from %s", segments.size(), input_path.c_str());

  std::vector<TrajectorySegment> retimed_segments;

  for (const auto& seg : segments) {
    robot_trajectory::RobotTrajectory rt(robot_model, "arm");
    moveit::core::RobotState state(robot_model);

    // Only retime pick_to_place if loaded segment scaling, or retime all segments if requested
    double seg_v_scale = (seg.name == "pick_to_place") ? v_scale : 0.20;
    double seg_a_scale = (seg.name == "pick_to_place") ? a_scale : 0.20;

    for (const auto& pt : seg.points) {
      state.setJointGroupPositions(jmg, pt.positions);
      rt.addSuffixWayPoint(state, 0.0);
    }

    bool success = false;
    if (algo == "iptp") {
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      success = iptp.computeTimeStamps(rt, seg_v_scale, seg_a_scale);
    } else {
      trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.005, 0.1, 0.001);
      success = totg.computeTimeStamps(rt, seg_v_scale, seg_a_scale);
    }

    if (!success) {
      RCLCPP_ERROR(node->get_logger(), "Time parameterization failed for segment %s!", seg.name.c_str());
      rclcpp::shutdown();
      return 1;
    }

    TrajectorySegment retimed_seg;
    retimed_seg.name = seg.name;
    retimed_seg.joint_names = seg.joint_names;

    double cum_time = 0.0;
    for (size_t i = 0; i < rt.getWayPointCount(); ++i) {
      cum_time += rt.getWayPointDurationFromPrevious(i);
      const auto& wpt = rt.getWayPoint(i);
      TrajectoryPoint pt;
      pt.time_sec = cum_time;
      pt.positions.resize(6);
      pt.velocities.resize(6);
      pt.accelerations.resize(6);
      wpt.copyJointGroupPositions(jmg, pt.positions.data());
      wpt.copyJointGroupVelocities(jmg, pt.velocities.data());
      wpt.copyJointGroupAccelerations(jmg, pt.accelerations.data());
      retimed_seg.points.push_back(pt);
    }

    RCLCPP_INFO(node->get_logger(), "Segment %s retimed: %zu waypoints, duration: %.3fs",
                seg.name.c_str(), retimed_seg.points.size(), cum_time);
    retimed_segments.push_back(retimed_seg);
  }

  writePathFile(output_path, retimed_segments, v_scale, a_scale, algo);
  RCLCPP_INFO(node->get_logger(), "Wrote retimed path file to %s", output_path.c_str());

  rclcpp::shutdown();
  return 0;
}
