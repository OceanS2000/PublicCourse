//
// Created by Ao Shen on 19-4-3.
//

#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace homework2 {

Mat distort(const Mat &source_img, double k1, double k2);

}

int main() {
  Mat original_img, distorted_img;
  original_img = imread("/home/ocean/PublicCourse/homework2/chessboard/chessboard_undistorted.png", IMREAD_COLOR);
  distorted_img = homework2::distort(original_img, 0.1, 0.1);

  imwrite("/home/ocean/PublicCourse/homework2/chessboard/chessboard_distorted.png", IMWRITE_PNG_STRATEGY_RLE);

  namedWindow("Undistorted Image");
  imshow("Undistorted Image", original_img);
  namedWindow("Distorted Image");
  imshow("Distorted Image", distorted_img);
  waitKey(0);
  return 0;
}

Mat homework2::distort(const Mat &source_img, double k1, double k2) {
  auto image = Mat(source_img.rows, source_img.cols, CV_8UC3, Scalar(0, 0, 0));
  /*
   * The following values are not specified in the problem.
   * So they are chosen arbitrarily as:
   * The original image is put on a plane with distance orig_dis to the camera.
   * Matrix of extrinsic parameters are chosen as [I | 0], (no rotation/trans).
   * The camera focus lengths fx, fy = 300.
   * The camera principal point (cx, cy) is at the center of original image.
   */
  double orig_dis = 500;
  double fx = 300, fy = 300;
  int cx = source_img.cols / 2, cy = source_img.rows / 2;

  int newx, newy;
  double iprime, jprime, r;
  for (int i = 0, nRow = source_img.rows; i < nRow; ++i) {
    for (int j = 0, nCol = source_img.cols; j < nCol; ++j) {
      iprime = (i - cy) / orig_dis;
      jprime = (j - cx) / orig_dis;
      r = iprime * iprime + jprime * jprime;
      iprime = iprime * (1 + k1 * r + k2 * r * r) * fy + cy;
      jprime = jprime * (1 + k1 * r + k2 * r * r) * fx + cx;
      newy = static_cast<int>(std::floor(iprime));
      newx = static_cast<int>(std::floor(jprime));
      image.at<Vec3b>(newy, newx) = source_img.at<Vec3b>(i, j);
    }
  }
  return image;
}
