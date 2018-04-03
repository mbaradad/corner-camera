#include "PointGreyInference.h"
#include "test_utils.cpp"
#include "RollingDisplay.h"


int main()
{
  /*  Original params
       int nangles = 100;
       int ncircles = 30;
       float rstep = 20;
       //lambda and beta are gain mat parameters
       double lambda = 7;
       double beta = 100;
       //alpha is running mean parameter:
       //backim_ = alpha_ * backim_ + (1 - alpha_) * dframe_;
       double alpha = 3.0/4;
       double fps = 10;
   */

  int nangles = 100;
  int ncircles = 30;
  float rstep = 20;
  //lambda and beta are gain mat parameters
  double lambda = 7;
  double beta = 100;
  //alpha is running mean parameter: dframe_ is current frame
  //backim_ = alpha_ * backim_ + (1 - alpha_) * dframe_;
  double alpha = 3.0/4;
  double fps = 10;

  bool precompute_background = true;
  int precomputed_background_iterations = 200;

  bool threshold_normalization = true;
  float threshold_min = -0.0001;
  float threshold_max = 0.0001;
  PointGreyInference pginf(nangles, ncircles,
      rstep, lambda, beta, alpha, fps,
                           precompute_background, precomputed_background_iterations,
                           threshold_normalization, threshold_min, threshold_max);

  pginf.processStream(0);
  
  return 0;
}
