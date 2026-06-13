#include "return_home_file_api_internal.h"

#include <ros/ros.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {

struct Options {
  std::string directory;
  std::string csv_path;
  int watch_iterations = 1;
  int poll_ms = 200;
  int expected_processed = 0;
  bool reset = false;
};

void printUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " <snapshot_dir> [options]\n"
      << "Options:\n"
      << "  --csv PATH                 Write CSV metrics.\n"
      << "  --watch N                  Call the directory API N times. Default: 1.\n"
      << "  --poll-ms N                Sleep between watch calls. Default: 200.\n"
      << "  --expected-processed N     Stop early after N processed snapshots.\n"
      << "  --reset                    Reset in-process detector before running.\n";
}

bool parseArgs(int argc, char** argv, Options* options) {
  if (!options) {
    return false;
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return false;
    }
    if (arg == "--csv" && i + 1 < argc) {
      options->csv_path = argv[++i];
      continue;
    }
    if (arg == "--watch" && i + 1 < argc) {
      options->watch_iterations = std::max(1, std::stoi(argv[++i]));
      continue;
    }
    if (arg == "--poll-ms" && i + 1 < argc) {
      options->poll_ms = std::max(0, std::stoi(argv[++i]));
      continue;
    }
    if (arg == "--expected-processed" && i + 1 < argc) {
      options->expected_processed = std::max(0, std::stoi(argv[++i]));
      continue;
    }
    if (arg == "--reset") {
      options->reset = true;
      continue;
    }
    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      return false;
    }
    if (!options->directory.empty()) {
      std::cerr << "Only one snapshot directory is supported.\n";
      return false;
    }
    options->directory = arg;
  }

  if (options->directory.empty()) {
    printUsage(argv[0]);
    return false;
  }
  return true;
}

std::string csvQuote(const std::string& value) {
  std::string quoted = "\"";
  for (const char c : value) {
    if (c == '"') {
      quoted += "\"\"";
    } else {
      quoted += c;
    }
  }
  quoted += "\"";
  return quoted;
}

void writeHeader(std::ostream& out) {
  out << "call_index,state,processed_new_snapshot,home_initialized,returned,"
      << "scan_id,position_number,point_count,init_ms,scan_dir_ms,metadata_ms,"
      << "pcd_load_ms,detect_ms,total_ms,max_rss_kb,metadata_file,"
      << "pointcloud_file,reason\n";
}

void writeRow(std::ostream& out,
              int call_index,
              const sonar_return_home_file_detector::ReturnHomeProbeStats& stats) {
  out << call_index << ','
      << csvQuote(stats.state) << ','
      << (stats.processed_new_snapshot ? 1 : 0) << ','
      << (stats.home_initialized ? 1 : 0) << ','
      << (stats.returned ? 1 : 0) << ','
      << stats.scan_id << ','
      << stats.position_number << ','
      << stats.point_count << ','
      << stats.init_ms << ','
      << stats.scan_dir_ms << ','
      << stats.metadata_ms << ','
      << stats.pcd_load_ms << ','
      << stats.detect_ms << ','
      << stats.total_ms << ','
      << stats.max_rss_kb << ','
      << csvQuote(stats.metadata_file) << ','
      << csvQuote(stats.pointcloud_file) << ','
      << csvQuote(stats.reason) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "return_home_directory_probe",
            ros::init_options::AnonymousName);

  Options options;
  if (!parseArgs(argc, argv, &options)) {
    return 2;
  }

  if (options.reset) {
    sonar_return_home_file_detector::resetReturnHomeFileDetectorForProbe();
  }

  std::ofstream csv_file;
  if (!options.csv_path.empty()) {
    csv_file.open(options.csv_path);
    if (!csv_file) {
      std::cerr << "Failed to open CSV: " << options.csv_path << "\n";
      return 1;
    }
    writeHeader(csv_file);
  }

  writeHeader(std::cout);

  int processed = 0;
  for (int i = 0; i < options.watch_iterations && ros::ok(); ++i) {
    const auto stats =
        sonar_return_home_file_detector::detectReturnHomeFromDirectoryForProbe(
            options.directory);
    writeRow(std::cout, i, stats);
    if (csv_file) {
      writeRow(csv_file, i, stats);
      csv_file.flush();
    }

    if (stats.processed_new_snapshot || stats.home_initialized) {
      ++processed;
    }
    if (options.expected_processed > 0 && processed >= options.expected_processed) {
      break;
    }
    if (i + 1 < options.watch_iterations && options.poll_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(options.poll_ms));
    }
  }

  return 0;
}
