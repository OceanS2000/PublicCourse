// Copyright @2018 Pony AI Inc. All rights reserved.

#include <iostream>
#include <fstream>
#include <algorithm>

#include "homework2/pointcloud.h"

int main() {
  // ATTENTION!!! : please use absolute path for reading the data file.
  const PointCloud pointcloud = ReadPointCloudFromTextFile(
      "/home/ocean/PublicCourse/homework2/data/src.txt");
  std::cout << "Total points read: " << pointcloud.points.size() << std::endl;

  std::ofstream height_histogram, range_histogram;
  height_histogram.open("/home/ocean/PublicCourse/homework2/height_histogram.svg");
  range_histogram.open("/home/ocean/PublicCourse/homework2/range_histogram.svg");
  if (!height_histogram.is_open() || !range_histogram.is_open())
    std::cerr << "Histogram file Failed to open!" << std::endl;

  std::vector<double> heights, ranges;
  for (const auto &point : pointcloud.points) {
    heights.push_back(point.z());
    ranges.push_back(std::sqrt(point.x() * point.x() + point.y() * point.y() + point.z() * point.z()));
  }

  const auto[min_height, max_height] = std::minmax_element(begin(heights), end(heights));
  const auto[min_range, max_range] = std::minmax_element(begin(ranges), end(ranges));
  int height_histogram_number[128], ranges_histogram_number[128], place;
  for (int i = 0; i < 128; i++) {
    height_histogram_number[i] = 0;
    ranges_histogram_number[i] = 0;
  }

  double height_histogram_step = (*max_height - *min_height) / 128;
  for (const double height : heights) {
    place = static_cast<int>((height - *min_height) / height_histogram_step);
    height_histogram_number[place] += 1;
  }
  height_histogram << "<?xml version='1.0' encoding='UTF-8'?>" << std::endl;
  height_histogram << "<svg width='1000' height='500' xmlns='http://www.w3.org/2000/svg'>\n"
                   << "  <defs>\n"
                   << "    <marker id='head' orient='auto' refX='2' refY='2'>\n"
                   << "      <path d='M 2 2 L 1 1 L 5 2 L 1 3 Z' fill='black' />\n"
                   << "    </marker>\n"
                   << "</defs>\n";
  for (int i = 0; i < 128; i++)
    height_histogram << "  <rect x='" << i * 6 + 20 << "' "
                     << "y='20' width='6' "
                     << "height='" << height_histogram_number[i] * 3000.0 / pointcloud.points.size() << "' "
                     << "style='fill:blue;stroke:black;stroke-width:1;fill-opacity:0.6' />"
                     << std::endl;
  height_histogram
      << "  <path d='M 20 470 L 20 20 L 800 20' stroke='black' fill='none' stroke-width='3'  marker-end='url(#head)' />"
      << std::endl << "</svg>" << std::endl;
  height_histogram.close();

  double ranges_histogram_step = (*max_range - *min_range) / 128;
  for (const double range : ranges) {
    place = static_cast<int>((range - *min_range) / ranges_histogram_step);
    ranges_histogram_number[place]++;
  }
  range_histogram << "<?xml version='1.0' encoding='UTF-8'?>" << std::endl;
  range_histogram << "<svg width='1000' height='500' xmlns='http://www.w3.org/2000/svg'>\n"
                  << "  <defs>\n"
                  << "    <marker id='head' orient='auto' refX='2' refY='2'>\n"
                  << "      <path d='M 2 2 L 1 1 L 5 2 L 1 3 Z' fill='black' />\n"
                  << "    </marker>\n"
                  << "  </defs>\n";
  for (int i = 0; i < 128; i++)
    range_histogram << "  <rect x='" << i * 6 + 20 << "' "
                    << "y='20' width='6' "
                    << "height='" << ranges_histogram_number[i] * 5000.0 / pointcloud.points.size() << "' "
                    << "style='fill:blue;stroke:black;stroke-width:1;fill-opacity:0.6' />"
                    << std::endl;
  range_histogram
      << "  <path d='M 20 470 L 20 20 L 800 20' stroke='black' fill='none' stroke-width='3'  marker-end='url(#head)' />"
      << std::endl << "</svg>" << std::endl;
  range_histogram.close();

  std::cout << "Rotation: " << std::endl;
  std::cout << pointcloud.rotation << std::endl;
  std::cout << "Translation: " << std::endl;
  std::cout << pointcloud.translation.transpose() << std::endl;
  return 0;
}
