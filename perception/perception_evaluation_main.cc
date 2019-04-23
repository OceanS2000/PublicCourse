// Copyright @2018 Pony AI Inc. All rights reserved.

#include <regex>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/proto/perception_evaluation.pb.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "common/utils/strings/format.h"
#include "perception/perception_evaluator.h"

DECLARE_string(perception_evaluation_label_file_suffix);
DECLARE_string(perception_evaluation_data_file_suffix);

DEFINE_string(evaluation_config_dir, "", "Directory contains evaluation config files.");
DEFINE_string(result_folder, "", "Result folder.");

namespace {

using interface::perception::ObjectType;

constexpr double kEvaluationRange = 30.0;

std::unordered_map<std::string, std::string> ObtainDataToLabelMapping(
    const std::string& data_dir, const std::string& label_dir) {
  std::unordered_map<std::string, std::string> data_to_label_map;
  // Obtain all label files.
  std::vector<std::string> label_files =
      file::path::FindFilesWithPrefixSuffix(
          label_dir, "", FLAGS_perception_evaluation_label_file_suffix);
  LOG(INFO) << "Total label files found: " << label_files.size();

  // Construct data to label file map.
  for (const auto& label_file : label_files) {
    const int file_index = std::stoi(file::path::FilenameStem(label_file));
    const std::string data_file = file::path::Join(data_dir, strings::Format("{}.txt", file_index));
    if (file::path::Exists(data_file)) {
      data_to_label_map.emplace(data_file, label_file);
    } else {
      LOG(WARNING) << "Fail to find label for " << data_file;
    }
  }
  return data_to_label_map;
}

void OutputResult(const std::map<ObjectType, int>& true_positive,
                  const std::map<ObjectType, int>& label_count,
                  const std::map<ObjectType, int>& detect_count) {
  const std::map<ObjectType, std::string> name_map {
      {ObjectType::CAR, "CAR"},
      {ObjectType::PEDESTRIAN, "PEDESTRIAN"},
      {ObjectType::CYCLIST, "CYCLIST"},
  };

  constexpr char kMessageBase[] = R"(
Type: {}
Precision: {}
Recall: {}
F1 score: {}
)";
  std::vector<double> f1_scores;
  for (auto iter : name_map) {
    ObjectType type = iter.first;
    double precision = 0.0;
    double recall = 0.0;
    if (true_positive.find(type) != true_positive.end()) {
      precision =
          (true_positive.at(type)) / (detect_count.at(type) + math::kEpsilon);
      recall =
          (true_positive.at(type)) / (label_count.at(type) + math::kEpsilon);
    }
    const double f1 = 1.0 / (1.0 / precision + 1.0 / recall);
    f1_scores.push_back(f1);
    const std::string content = strings::Format(kMessageBase, iter.second, precision, recall, f1);
    std::cout << content;
  }
  std::cout << "Average F1 score: "
            << (f1_scores[0] + f1_scores[1] + f1_scores[2]) / 3.0 << std::endl;
}

void RunPerceptionEvaluation() {
  if (FLAGS_result_folder.empty()) {
    FLAGS_result_folder = "/tmp/" + std::to_string(time(NULL)) + "/";
  }
  file::CreateFolder(FLAGS_result_folder);
  const auto config_files =
      file::path::FindFilesWithPrefixSuffix(FLAGS_evaluation_config_dir, "", ".config");

  double precision_sum = 0.0;
  double recall_sum = 0.0;
  std::map<interface::perception::ObjectType, int> tps;
  std::map<interface::perception::ObjectType, int> label_count;
  std::map<interface::perception::ObjectType, int> detect_count;
  double total_velocity_diff = 0.0;
  int total_label_count = 0;
  int num_frames = 0;
  for (const auto& config_file : config_files) {
    interface::perception::PerceptionEvaluationConfig config;
    CHECK(file::ReadTextFileToProto(config_file, &config))
        << "Failed to load config file: " << config_file;
    const std::string result_file =
        file::path::Join(FLAGS_result_folder, strings::Format("{}.result", config.scenario_name()));
    const std::string lidar_dir = config.local_data().lidar_dir();
    const std::string label_dir = config.local_data().label_dir();
    const std::string camera_dir = config.local_data().camera_dir();
    CHECK(file::path::Exists(lidar_dir)) << lidar_dir << " doesn't exist.";
    CHECK(file::path::Exists(label_dir)) << label_dir << " doesn't exist.";;
    // Run perception evaluation.
    PerceptionEvaluator::Options options;
    options.scenario_name = config.scenario_name();
    options.lidar_folder = lidar_dir;
    options.camera_folder = camera_dir;
    options.data_to_label_map = ObtainDataToLabelMapping(lidar_dir, label_dir);
    options.evaluation_range = kEvaluationRange;
    auto evaluator = std::make_unique<PerceptionEvaluator>(options);
    const auto eval_result = evaluator->RunEvaluation();
    CHECK(file::WriteProtoToTextFile(eval_result, result_file));
    std::cout << "*********************************************************" << std::endl;
    std::cout << "Evaluation results for " << config_file << std::endl;
    for (const auto& frame_result : eval_result.frame_result()) {
      std::map<interface::perception::ObjectType, int> frame_tps;
      std::map<interface::perception::ObjectType, int> frame_label_count;
      std::map<interface::perception::ObjectType, int> frame_detect_count;
      total_velocity_diff += frame_result.total_velocity_diff();
      for (const auto& tp : frame_result.true_positive()) {
        frame_tps[tp.type()] = tp.count();
        tps[tp.type()] += tp.count();
      }
      for (const auto& tp : frame_result.label_count()) {
        frame_label_count[tp.type()] = tp.count();
        label_count[tp.type()] += tp.count();
        total_label_count += tp.count();
      }
      for (const auto& tp : frame_result.detect_count()) {
        frame_detect_count[tp.type()] = tp.count();
        detect_count[tp.type()] += tp.count();
      }
      OutputResult(frame_tps, frame_label_count, frame_detect_count);
      ++num_frames;
    }
  }
  // Calculate the average precision and recall for all frames.
  double average_precision = 0.0;
  double average_recall = 0.0;
  if (num_frames != 0) {
    average_precision = precision_sum / num_frames;
    average_recall = recall_sum / num_frames;
  }
  std::cout << std::endl << std::endl;
  std::cout << "*********************************************************" << std::endl;
  OutputResult(tps, label_count, detect_count);
  std::cout << "Average velocity difference is " << total_velocity_diff / total_label_count
            << std::endl;
  std::cout << "*********************************************************" << std::endl;
  std::cout << "Evaluation result folder: " << FLAGS_result_folder << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  RunPerceptionEvaluation();
  return 0;
}
