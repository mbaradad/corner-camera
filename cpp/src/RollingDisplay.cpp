#include "RollingDisplay.h"

#include <algorithm>
#include <exception>
#include <opencv2/imgproc/imgproc.hpp>

void printMinMax(const cv::Mat& m){
  double minVal;
  double maxVal;
  cv::Point minLoc;
  cv::Point maxLoc;

  std::vector<cv::Mat> channels(3);
// split img:
  split(m, channels);
// get the channels (dont forget they follow BGR order in OpenCV)

  cv::minMaxLoc( channels[0], &minVal, &maxVal, &minLoc, &maxLoc);

  std::cout << "min val : " << minVal << std::endl;
  std::cout << "max val: " << maxVal << std::endl;
}

RollingDisplay::RollingDisplay(std::string name, bool roll_up,
    int ncols, int buf_factor)
{
  if (buf_factor < 1)
    throw std::runtime_error("buffer must be at least same length as display");

  name_ = name;
  roll_up_ = roll_up;
  nrows_ = DEFAULT_ROWS;
  ncols_ = ncols;

  bufsize_ = nrows_ * buf_factor;
  buffer_ = cv::Mat::zeros(bufsize_, ncols_, CV_64FC3);
  cv::namedWindow(name_, CV_WINDOW_NORMAL);

  dstart_ = 0;
  drow_ = 0;
}


RollingDisplay::RollingDisplay(std::string name, bool roll_up,
    int nrows, int ncols, int buf_factor)
{
  if (buf_factor < 1)
    throw std::runtime_error("buffer must be at least same length as display");

  name_ = name;
  roll_up_ = roll_up;
  nrows_ = nrows;
  ncols_ = ncols;

  bufsize_ = nrows_ * buf_factor;
  buffer_ = cv::Mat(bufsize_, ncols_, CV_64FC3);
  buffer_ = buffer_*0;
  cv::namedWindow(name_, CV_WINDOW_NORMAL);

  dstart_ = 0;
  drow_ = 0;
}


cv::Mat RollingDisplay::nextrow()
{
  if (drow_ >= bufsize_) { // need to wrap around to beginning
    wrapAround();
  }
  dstart_ = std::max(0, drow_ - nrows_ + 1);
  return buffer_.row(drow_++);
}


void RollingDisplay::update()
{
  if (roll_up_) {
    cv::flip(buffer_.rowRange(dstart_, dstart_ + nrows_), disp_, 0);
  } else {
    disp_ = buffer_.rowRange(dstart_, dstart_ + nrows_);
  }
  cv::Mat dst_norm;
  cv::normalize(disp_,dst_norm,0.0,1.0,cv::NORM_MINMAX);
  cv::imshow(name_, dst_norm);
}

void RollingDisplay::update(float min_threshold, float max_threshold)
{
  if (roll_up_) {
    cv::flip(buffer_.rowRange(dstart_, dstart_ + nrows_), disp_, 0);
  } else {
    disp_ = buffer_.rowRange(dstart_, dstart_ + nrows_);
  }
  std::cout << " begin rolling" << std::endl;
  cv::Mat dst_norm_1;
  cv::threshold(-1*disp_,dst_norm_1,-1*min_threshold,-1*min_threshold,cv::THRESH_TRUNC);
  cv::Mat dst_norm_2;
  cv::threshold(-1*dst_norm_1,dst_norm_2,max_threshold,max_threshold,cv::THRESH_TRUNC);
  cv::Mat dst_norm;
  dst_norm = (dst_norm_2 - min_threshold)/(max_threshold - min_threshold);
  std::cout << " begin rolling" << std::endl;
  cv::imshow(name_, dst_norm);
}

void RollingDisplay::wrapAround()
{ // copy the last nrows_ - 1 rows to the beginning and update drow
  int end = drow_;
  int start = end - (nrows_ - 1);
  cv::Mat last = buffer_.rowRange(start, end);
  last.copyTo(buffer_.rowRange(0, nrows_ - 1));
  drow_ = nrows_ - 1;
}
