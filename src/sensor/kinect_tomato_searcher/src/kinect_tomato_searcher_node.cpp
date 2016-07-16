#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <pthread.h>
#include <geometry_msgs/Point.h>

static const std::string RGB_WINDOW_NAME = "RGB image";
static const std::string DEPTH_WINDOW_NAME = "Depth image";

class SearchTomato {
public:
  SearchTomato()
    : found_flag(false)
    , tick_count(0)
    , last_tick_count(0)
    , not_founding_count(0)
    , kf(6, 4, 0, CV_32F)
    , state(6, 1, CV_32F)
  {
    init_set_kalman(kf);
  }

  ~SearchTomato() {}

  bool searchTomatoPoint(const cv::Mat& capture_rgb, cv::Point& tomato_point) {
    cv::Mat binary_mat = cv::Mat::zeros(capture_rgb.size(), CV_8UC1);
    std::vector<cv::Rect> tomato_boxs;
    // reset transition matrix of kalman filter.
    tick_count = cv::getTickCount();
    double d_time = (tick_count - last_tick_count) / cv::getTickFrequency(); // d_time: infinitesimal difference of time.
    kf.transitionMatrix.at<float>(2) = d_time; // x = x + dt*vx
    kf.transitionMatrix.at<float>(9) = d_time; // y = y + dt*vy
    last_tick_count = tick_count;

    // image processing and seach box for tomato
    imageProcessing(capture_rgb, binary_mat);
    imshow(DEPTH_WINDOW_NAME, binary_mat);
    searchTomatoBox(binary_mat, tomato_boxs);
    printf("search box num is %lu", tomato_boxs.size());

    if (tomato_boxs.size() > 0) {
      not_founding_count = 0;
      if (!found_flag) {
	resetKalman();
	found_flag = true;
      } else
	applyKalman(tomato_boxs);

    } else
      if (++not_founding_count > 10) {
	not_founding_count = 10;
	found_flag = false;
      }

    // set point of tomato
    state = kf.predict();
    tomato_point.x = state.at<float>(0);
    tomato_point.y = state.at<float>(1);

    return found_flag;
  }

private:
  void init_set_kalman(cv::KalmanFilter& kf) {
    // transition state matrix(A). think for [x, y, vx, vy, width, height]. identiry mean state is static.
    cv::setIdentity(kf.transitionMatrix); // NOTE: set state of speed at each step!

    // measurement matrix(H). mean look to only [x, y, width, height].
    kf.measurementMatrix = cv::Mat::zeros(4, 6, CV_32F);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // process noise covariance matrix(Q). all state have noise is 5e-2;
    cv::setIdentity(kf.processNoiseCov, cv::Scalar(5e-2));
  }

  void imageProcessing(const cv::Mat& rgb, cv::Mat& binary_mat) {
    cv::Mat blur, hsv;
    cv::GaussianBlur(rgb, blur, cv::Size(5, 5), 3.0, 3.0);
    cv::cvtColor(blur, hsv, CV_BGR2HSV);
    redFilter(hsv, binary_mat);
    cv::erode(binary_mat, binary_mat, cv::Mat(), cv::Point(-1, -1), 3);
    cv::dilate(binary_mat, binary_mat, cv::Mat(), cv::Point(-1, -1), 5);
    cv::erode(binary_mat, binary_mat, cv::Mat(), cv::Point(-1, -1), 3);
  }

  void redFilter(cv::Mat& hsv, cv::Mat& binary_mat) {
    int a, x, y;
    for(y = 0; y < hsv.rows; y++) {
      for(x = 0; x < hsv.cols; x++) {
	a = hsv.step*y+(x*3);
	if((hsv.data[a] <= 5 || hsv.data[a] >= 175) && hsv.data[a+1] >= 50 && hsv.data[a+2] >= 50 )
	  binary_mat.at<unsigned char>(y,x) = 255;
	else
	  binary_mat.at<unsigned char>(y,x) = 0;
      }
    }
  }

  void searchTomatoBox(cv::Mat& binary_mat, std::vector<cv::Rect>& tomato_boxs) {
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(binary_mat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    findMatchTomato(contours, tomato_boxs);
  }

  void findMatchTomato(std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Rect>& tomato_boxs) {
    cv::Rect bounding_box;
    for (unsigned int i = 0; i < contours.size(); i++) {
      bounding_box = cv::boundingRect(contours[i]);

      float ratio_balance = (float)bounding_box.width / (float)bounding_box.height;
	if (ratio_balance > 1.0f) ratio_balance = 1.0f / ratio_balance;

      // delete mismach. that is smaller or spier
      if (bounding_box.area() >= 500 && bounding_box.area() < 2*cv::contourArea(contours[i]) && ratio_balance > 0.4f)
	tomato_boxs.push_back(bounding_box);
    }
  }

  void applyKalman(std::vector<cv::Rect>& tomato_boxs) {
    cv::Mat meas(4, 1, CV_32F);
    int i = findIndexPlausible(tomato_boxs);

    meas.at<float>(0) = tomato_boxs[i].x + tomato_boxs[i].width/2;
    meas.at<float>(1) = tomato_boxs[i].y + tomato_boxs[i].height/2;
    meas.at<float>(2) = tomato_boxs[i].width;
    meas.at<float>(3) = tomato_boxs[i].height;
    kf.correct(meas);
  }

  int findIndexPlausible(std::vector<cv::Rect>& tomato_boxs) {
    int index = 0;
    unsigned int worst_non_likelihood = 0xffff;
    unsigned int non_likelihood;
    for (int i = 0; i < tomato_boxs.size(); i++) {
      state = kf.predict();
      non_likelihood = 0;
      non_likelihood += abs(tomato_boxs[i].x - state.at<float>(0));
      non_likelihood += abs(tomato_boxs[i].y - state.at<float>(1));
      non_likelihood += abs(tomato_boxs[i].width - state.at<float>(4));
      non_likelihood += abs(tomato_boxs[i].height - state.at<float>(5));
      if (worst_non_likelihood > non_likelihood) {
	worst_non_likelihood = non_likelihood;
	index = i;
      }
    }
    return index;
  }

  void resetKalman() {
//     kf.errorCovPre.at<float>(0) = 1; // px
//     kf.errorCovPre.at<float>(7) = 1; // px
//     kf.errorCovPre.at<float>(14) = 1;
//     kf.errorCovPre.at<float>(21) = 1;
//     kf.errorCovPre.at<float>(28) = 1; // px
//     kf.errorCovPre.at<float>(35) = 1; // px

    for (int i=0; i<6; i++) kf.errorCovPre.at<float>(i*7) = 1;
  }

  bool found_flag;
  long tick_count, last_tick_count; // count while running.
  unsigned int not_founding_count;
  cv::KalmanFilter kf;
  cv::Mat state;
};

class ImageConverter {
private:
  ros::NodeHandle nh;
  ros::Publisher point_pub;
  image_transport::ImageTransport it;
  image_transport::Subscriber rgb_sub;
  image_transport::Subscriber depth_sub;
  cv_bridge::CvImageConstPtr rgb_ptr;
  cv_bridge::CvImageConstPtr depth_ptr;
  SearchTomato searchTomatoObj;
  cv::Point tomato_point;
  bool found_flag;
  geometry_msgs::Point pub_msg;

public:
  ImageConverter()
    : it(nh)
    , searchTomatoObj()
    , tomato_point(-1, -1)
    , found_flag(false)
  {
    // subscrive to input video feed.
    point_pub = nh.advertise<geometry_msgs::Point>("tomato_point", 1);
    rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::rgbCb, this);
    depth_sub = it.subscribe("/camera/depth_registered/image_raw", 1, &ImageConverter::depthCb, this);
  }

  ~ImageConverter() {}

  /**
   * search tomato function.
   * get rgb image, and search tomato.
   * will tomato_point have data.
   *
   * @author Yusuke Doi
   */
  void rgbCb(const sensor_msgs::ImageConstPtr& msg) {
    // get rgb image on kinect
    try {
      rgb_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception by rgb: %s", e.what());
    }

    cv::Mat buf = rgb_ptr->image.clone();
    // search tomato
    if (found_flag = searchTomatoObj.searchTomatoPoint(rgb_ptr->image, tomato_point))
      cv::circle(buf, tomato_point, 2, CV_RGB(0, 255, 0), -1);

    cv::imshow(RGB_WINDOW_NAME, buf);
    cv::waitKey(100);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      depth_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception by depth: %s", e.what());
    }

    //testShowDepth(depth_ptr->image);
    if (found_flag) {
      printf("depth is %u", depth_ptr->image.at<uint16_t>(tomato_point));
//      pub_msg.x = tomato_point.x;
//      pub_msg.y = tomato_point.y;
//      pub_msg.z = depth_ptr->image.at<uint16_t>(tomato_point);
      pub_msg.x = depth_ptr->image.at<uint16_t>(tomato_point);
      pub_msg.y = tomato_point.x;
      pub_msg.z = -tomato_point.y;
      point_pub.publish(pub_msg);
    }
    cv::waitKey(100);
  }

  void testShowDepth(const cv::Mat& depth) {
    static std::vector<uint16_t> m_gamma(2048);
    static bool firstFlag(false);
    if (!firstFlag) {
      for(unsigned int i = 0; i < 2048; i++) {
	float v = i/2048.0;
	v = std::pow(v, 3) * 6;
	m_gamma[i] = v*6*256;
      }
      firstFlag = true;
    }

    int i;
    cv::Mat buf = cv::Mat::zeros(depth.size(), CV_16UC1);
    cv::Mat output = cv::Mat::zeros(depth.size(), CV_8UC3);
    cv::normalize(depth, buf, 0, 2013, cv::NORM_MINMAX);

    for (i = 0; i < depth.rows * depth.cols; i++) {
      int y = (int)(i / depth.cols);
      int x = (int)(i % depth.cols);
      int pval = m_gamma[buf.at<uint16_t>(i)];
      int lb = pval & 0xff;

      switch(pval >> 8) {
      case 0:
	output.at<cv::Vec3b>(y,x)[2] = 255;
	output.at<cv::Vec3b>(y,x)[1] = 255-lb;
	output.at<cv::Vec3b>(y,x)[0] = 255-lb;
	break;
      case 1:
	output.at<cv::Vec3b>(y,x)[2] = 255;
	output.at<cv::Vec3b>(y,x)[1] = lb;
	output.at<cv::Vec3b>(y,x)[0] = 0;
	break;
      case 2:
	output.at<cv::Vec3b>(y,x)[2] = 255-lb;
	output.at<cv::Vec3b>(y,x)[1] = 255;
	output.at<cv::Vec3b>(y,x)[0] = 0;
	break;
      case 3:
	output.at<cv::Vec3b>(y,x)[2] = 0;
	output.at<cv::Vec3b>(y,x)[1] = 255;
	output.at<cv::Vec3b>(y,x)[0] = lb;
	break;
      case 4:
	output.at<cv::Vec3b>(y,x)[2] = 0;
	output.at<cv::Vec3b>(y,x)[1] = 255-lb;
	output.at<cv::Vec3b>(y,x)[0] = 255;
	break;
      case 5:
	output.at<cv::Vec3b>(y,x)[2] = 0;
	output.at<cv::Vec3b>(y,x)[1] = 0;
	output.at<cv::Vec3b>(y,x)[0] = 255-lb;
        break;
      default:
        output.at<cv::Vec3b>(y,x)[2] = 0;
	output.at<cv::Vec3b>(y,x)[1] = 0;
	output.at<cv::Vec3b>(y,x)[0] = 0;
	break;
      }
    }
    imshow(DEPTH_WINDOW_NAME, output);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_tomato_search_node");
  ImageConverter ic;
  cv::namedWindow(RGB_WINDOW_NAME);
  cv::namedWindow(DEPTH_WINDOW_NAME);
  ros::spin();
  cv::destroyWindow(RGB_WINDOW_NAME);
  cv::destroyWindow(DEPTH_WINDOW_NAME);
  return 0;
}
