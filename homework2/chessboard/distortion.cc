//
// Created by Ao Shen on 19-4-3.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace homework2 {

void distort(Mat source_img);

}

int main() {
  Mat original_img;
  original_img = imread("/home/ocean/PublicCourse/homework2/chessboard/chessboard_undistorted.png", IMREAD_COLOR);
  namedWindow("Undistorted Image");
  imshow("Undistorted Image", original_img);
  waitKey(0);
  return 0;
}
