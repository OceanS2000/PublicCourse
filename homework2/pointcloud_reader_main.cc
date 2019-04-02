// Copyright @2018 Pony AI Inc. All rights reserved.

#include <iostream>

#include "homework2/pointcloud.h"

int main() {
  // ATTENTION!!! : please use absolute path for reading the data file.
  const PointCloud pointcloud = ReadPointCloudFromTextFile(
      "/home/ocean/PublicCourse/homework2/data/src.txt");
  std::cerr << "Total points read: " << pointcloud.points.size() << std::endl;
  double max_height = 0.0, min_height = 0.0;
  for (auto point : pointcloud.points) {
    if (point.z() > max_height)
      max_height = point.z();
    if (point.z() < min_height)
      min_height = point.z();
  }
  int height_histogram_number[63], place;
  for (int i = 0; i < 64; i++)
    height_histogram_number[i] = 0;
  double height_histogram_step = (max_height - min_height) / 64;
  for (auto point : pointcloud.points) {
    place = (int) ((point.z() - min_height) / height_histogram_step);
    height_histogram_number[place]++;
  }
  //std::cout << "<?xml version='1.0' encoding='UTF-8' ?>" << std::endl;
  std::cout << "<svg width='1000' height='500' xmlns='http://www.w3.org/2000/svg'>" << std::endl
            << "  <defs>" << std::endl
            << "    <marker id='head' orient='auto' markerwidth='2' markerheight='4' refX='0.1' refY='2'>" << std::endl
            << "      <path d='M 0 0 L 1 -1 L 0 4 L -1 -1 Z' fill='black' />" << std::endl
            << "    </marker>\n  </defs>" << std::endl;
  for (int i = 0; i < 64; i++)
    std::cout << "  <rect x='" << i * 12 + 20 << "' "
              << "y='20' width='12' "
              << "height='" << height_histogram_number[i] * 1500.0 / pointcloud.points.size() << "' "
              << "style='fill:blue;stroke:black;stroke-width:1;fill-opacity:0.6' />"
              << std::endl;
  std::cout
      << "  <path d='M 20 470 L 20 20 L 800 20' stroke='black' fill='none' stroke-width='3'  marker-end='url(#head)' />"
      << std::endl << "</svg>" << std::endl;

/*  std::cout << "Rotation: " << std::endl;
  std::cout << pointcloud.rotation << std::endl;
  std::cout << "Translation: " << std::endl;
  std::cout << pointcloud.translation.transpose() << std::endl;
*/
  return 0;
}
