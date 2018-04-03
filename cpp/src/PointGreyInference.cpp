#include "PointGreyInference.h"
#include "RollingDisplay.h"

void PointGreyInference::printConfig(FlyCapture2::Camera *cam){
  std::cout << "BRIGHTNESS: " << getProperty(cam,FlyCapture2::BRIGHTNESS) << std::endl;
  std::cout << "AUTO_EXPOSURE: " << getProperty(cam, FlyCapture2::AUTO_EXPOSURE) << std::endl;
  std::cout << "SHARPNESS: " << getProperty(cam, FlyCapture2::SHARPNESS) << std::endl;
  std::cout << "WHITE_BALANCE: " << getProperty(cam, FlyCapture2::WHITE_BALANCE) << std::endl;
  std::cout << "HUE: " << getProperty(cam, FlyCapture2::HUE) << std::endl;
  std::cout << "SATURATION: " << getProperty(cam, FlyCapture2::SATURATION) << std::endl;
  std::cout << "GAMMA: " << getProperty(cam, FlyCapture2::GAMMA) << std::endl;
  std::cout << "IRIS: " << getProperty(cam, FlyCapture2::IRIS) << std::endl;
  std::cout << "FOCUS: " << getProperty(cam, FlyCapture2::FOCUS) << std::endl;
  std::cout << "ZOOM: " << getProperty(cam, FlyCapture2::ZOOM) << std::endl;
  std::cout << "PAN: " << getProperty(cam, FlyCapture2::PAN) << std::endl;
  std::cout << "TILT: " << getProperty(cam, FlyCapture2::TILT) << std::endl;
  std::cout << "SHUTTER: " << getProperty(cam, FlyCapture2::SHUTTER) << std::endl;
  std::cout << "GAIN: " << getProperty(cam, FlyCapture2::GAIN) << std::endl;
  std::cout << "TRIGGER_MODE: " << getProperty(cam, FlyCapture2::TRIGGER_MODE) << std::endl;
  std::cout << "TRIGGER_DELAY: " << getProperty(cam, FlyCapture2::TRIGGER_DELAY) << std::endl;
  std::cout << "FRAME_RATE: " << getProperty(cam, FlyCapture2::FRAME_RATE) << std::endl;
  std::cout << "TEMPERATURE: " << getProperty(cam, FlyCapture2::TEMPERATURE) << std::endl;
}

PointGreyInference::PointGreyInference( int nangles, int ncirlces,
    float rstep, double lambda, double beta, double alpha, double fps,
    bool precomputeBackground, int precomputeBackgroundIterations,
                                        bool threshold_normalization, float threshold_min, float threshold_max)
  : Inference(nangles, ncirlces, rstep, lambda, beta, alpha)
{
  fps_ = fps;
  precomputeBackground_ = precomputeBackground;
  precomputeBackgroundIterations_ = precomputeBackgroundIterations;
  threshold_normalization_ = threshold_normalization;
  threshold_min_ = threshold_min;
  threshold_max_ = threshold_max;
}

PointGreyInference::PointGreyInference( int nangles, int ncirlces,
                                        float rstep, double lambda, double beta, double alpha, double fps)
    : Inference(nangles, ncirlces, rstep, lambda, beta, alpha)
{
  fps_ = fps;
  precomputeBackground_ = false;
  precomputeBackgroundIterations_ = -1;
  threshold_normalization_ = false;

}

PointGreyInference::PointGreyInference(cv::Point corner,
    cv::Point wallpt, cv::Point endpt,
    int nangles, int ncirlces, float rstep,
    double lambda, double beta, double alpha, double fps,
    bool precomputeBackground, int precomputeBackgroundIterations)
  : Inference(corner, wallpt, endpt,
      nangles, ncirlces, rstep, lambda, beta, alpha)
{
  fps_ = fps;
  precomputeBackground_ = precomputeBackground;
  precomputeBackgroundIterations_ = precomputeBackgroundIterations;
  threshold_normalization_ = false;
}


void PointGreyInference::getNextFrame(FlyCapture2::Camera *cam) {
  FlyCapture2::Error error;
  FlyCapture2::Image raw, rgb;
  error = cam->RetrieveBuffer(&raw);
  if (error != FlyCapture2::PGRERROR_OK) {
    throw std::runtime_error("capture error");
  }
  error = raw.Convert(FlyCapture2::PIXEL_FORMAT_BGR16, &rgb);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
    throw std::runtime_error("format problem (16 vs 8 bits)");
  }
  double dataSize = (double) rgb.GetReceivedDataSize();
  double rows = (double) rgb.GetRows();
  unsigned int rowbytes = dataSize / rows;
  cv::Mat frame;
  cv::Mat img;
  frame = cv::Mat(rgb.GetRows(), rgb.GetCols(),
                  CV_16UC3, rgb.GetData(), rowbytes);
  frame.convertTo(img, CV_32FC3);
  img = img / (2 << 16);
  img.copyTo(frame_);
}


void PointGreyInference::processStream(bool disp_rollup)
{ // if disp_rollup > 0, roll up, otherwise roll down
  cv::destroyAllWindows();

  FlyCapture2::Error error;
  FlyCapture2::Camera cam;
  FlyCapture2::CameraInfo camInfo;


  // connect to camera
  error = cam.Connect(0);
  if (error != FlyCapture2::PGRERROR_OK) {
    throw std::runtime_error("Failed to connect to camera");
  }

  error = cam.GetCameraInfo(&camInfo);
  if (error != FlyCapture2::PGRERROR_OK) {
    throw std::runtime_error("Failed to get camera info from camera");
  }
  printf("%s %s %d\n", camInfo.vendorName,
      camInfo.modelName, camInfo.serialNumber);

  FlyCapture2::Format7ImageSettings f7;
  float pPercentage;
  unsigned int packetSize ;
  error = cam.GetFormat7Configuration(&f7, &packetSize, &pPercentage);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }
  f7.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW16;
//f7.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
  error = cam.SetFormat7Configuration(&f7, pPercentage);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }
  error = cam.GetFormat7Configuration(&f7, &packetSize, &pPercentage);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }
  cameraFps_ = fps_;
  setProperty(&cam,FlyCapture2::FRAME_RATE, cameraFps_);
  setProperty(&cam, FlyCapture2::GAMMA, 1);
  autoAdjustProperty(&cam, FlyCapture2::GAIN);
  autoAdjustProperty(&cam, FlyCapture2::SHUTTER);
  //autoAdjustProperty(&cam, FlyCapture2::TEMPERATURE);
  //autoAdjustProperty(&cam, FlyCapture2::AUTO_EXPOSURE);
  //setProperty(&cam, FlyCapture2::AUTO_EXPOSURE, 1);
  setOff(&cam, FlyCapture2::AUTO_EXPOSURE);
  setOff(&cam, FlyCapture2::GAMMA);

  error = cam.StartCapture();
  if (error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
    throw std::runtime_error("Bandwidth exceeded");
  } else if (error != FlyCapture2::PGRERROR_OK) {
    throw std::runtime_error("Failed to start image capture. Mabe run make_grasshopper_work?");
  }
  cameraFps_ = getProperty(&cam,FlyCapture2::FRAME_RATE);

  // get frame info, setup, and initialize running mean
  printConfig(&cam);
  getNextFrame(&cam);
  printConfig(&cam);

  nrows_ = frame_.rows;
  ncols_ = frame_.cols;
  printf("%d x %d framesize\n", nrows_, ncols_);
  printf("%d fps\n", cameraFps_);

  findObsRegion(); // set observation region if haven't already

  setup(); // compute gain, and sample and crop locations

  // initialize the background image
  preprocessFrame();
  updateMeanImage(1);

  plotObsXYLocs("sample locations");

  RollingDisplay display = RollingDisplay("output", disp_rollup, nangles_);
  cv::Mat cur_disp;

  cv::namedWindow("background image", CV_WINDOW_NORMAL);

  /* capture loop */
  printf("Starting inference, press esc to break\n");
  time_t start_time = time(0);

  int total = 0;
  while (1) {
    getNextFrame(&cam);
    //printConfig(&cam);
    preprocessFrame();
    cur_disp = display.nextrow();
    processFrame(cur_disp);

    cv::Mat dst_norm;
    cv::normalize(dframe_,dst_norm,0.0,1.0,cv::NORM_MINMAX);
    cv::imshow("input", dst_norm);
    cv::normalize(backim_,dst_norm,0.0,1.0,cv::NORM_MINMAX);
    cv::imshow("background image", dst_norm);
    cv::normalize((dframe_ - backim_),dst_norm,0.0,1.0,cv::NORM_MINMAX);
    cv::imshow("diff_image", dst_norm);

    if (threshold_normalization_)
      display.update(threshold_min_, threshold_max_);
    else
      display.update();
    if (cv::waitKey(1) == 27) {
      printf("pressed ESC key, exiting inference\n");
      break;
    }
    if ((not precomputeBackground_) or (precomputeBackground_ and total < precomputeBackgroundIterations_)){
      if (precomputeBackground_){
      std::cout << "Updating mean: " << total << " of " << precomputeBackgroundIterations_ << std::endl;
      }
      updateMeanImage(total+1);
    }
    else if (precomputeBackground_){
      std::cout << "Mean wont be updated!" << std::endl;
    }
    total++;
  }

  error = cam.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK) {
    printf("error stopping capture\n");
  }

  cam.Disconnect();
  cv::destroyAllWindows();
}


void PointGreyInference::disableAuto(FlyCapture2::Camera *cam,
    FlyCapture2::PropertyType propType)
{
  FlyCapture2::Property prop;
  prop.type = propType;
  FlyCapture2::Error error = cam->GetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  prop.autoManualMode = false;
  prop.absControl = true;
  error = cam->SetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }
}

float PointGreyInference::getProperty(FlyCapture2::Camera *cam,
                                      FlyCapture2::PropertyType propType)
{
  FlyCapture2::Property prop;
  prop.type = propType;
  FlyCapture2::Error error = cam->GetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  cam->GetProperty(&prop);
  return prop.absValue;
}


void PointGreyInference::setProperty(FlyCapture2::Camera *cam,
    FlyCapture2::PropertyType propType, float ss)
{
  FlyCapture2::Property prop;
  prop.type = propType;
  FlyCapture2::Error error = cam->GetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  prop.absValue = ss;
  prop.autoManualMode = false;
  prop.absControl = true;
  error = cam->SetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  printf("property requested value: %f\n", ss);
  cam->GetProperty(&prop);
  printf("property received value: %f\n", prop.absValue);
}


void PointGreyInference::setExposure(FlyCapture2::Camera *cam, float ss)
{
  printf("setting exposure (shutter)...\n");
  setProperty(cam, FlyCapture2::SHUTTER, ss);
}


void PointGreyInference::setGain(FlyCapture2::Camera *cam, float ss)
{
  printf("setting gain...\n");
  setProperty(cam, FlyCapture2::GAIN, ss);
}


void PointGreyInference::setFPS(FlyCapture2::Camera *cam, float ss)
{
  printf("setting fps...\n");
  setProperty(cam, FlyCapture2::FRAME_RATE, ss);
}


void PointGreyInference::autoAdjustProperty(FlyCapture2::Camera *cam,
    FlyCapture2::PropertyType propType)
{
  FlyCapture2::Property prop;
  prop.type = propType;
  FlyCapture2::Error error = cam->GetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  prop.autoManualMode = false;
  prop.absControl = true;
  prop.onePush = true;
  error = cam->SetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }
}


void PointGreyInference::setOnOff(FlyCapture2::Camera *cam,
                               FlyCapture2::PropertyType propType, bool onOff){
  FlyCapture2::Property prop;
  prop.type = propType;
  FlyCapture2::Error error = cam->GetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

  prop.onOff = onOff;
  error = cam->SetProperty(&prop);
  if (error != FlyCapture2::PGRERROR_OK) {
    error.PrintErrorTrace();
  }

}

void PointGreyInference::setOn(FlyCapture2::Camera *cam,
                                FlyCapture2::PropertyType propType)
{
  setOnOff(cam, propType, true);
}


void PointGreyInference::setOff(FlyCapture2::Camera *cam,
                                            FlyCapture2::PropertyType propType)
{
  setOnOff(cam, propType, false);
}


