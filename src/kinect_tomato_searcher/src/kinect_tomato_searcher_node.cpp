#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <kinect_tomato_searcher/SearchConfig.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

class ShowImage {
public:
  ShowImage()
    : m_gamma(2048),
      RGB_WINDOW_NAME {"RGB image"},
      DEPTH_WINDOW_NAME {"Depth image"},
      BINARY_WINDOW_NAME {"Binary image"}
  {
    for(int i = 0; i < 2048; i++) {
      float v = i / 2048.0;
      v = std::pow(v, 3) * 6;
      m_gamma[i] = v * 6 * 256;
    }

    cv::namedWindow(RGB_WINDOW_NAME);
    cv::namedWindow(DEPTH_WINDOW_NAME);
    cv::namedWindow(BINARY_WINDOW_NAME);
  }

  ~ShowImage() {
    cv::destroyWindow(RGB_WINDOW_NAME);
    cv::destroyWindow(DEPTH_WINDOW_NAME);
    cv::destroyWindow(BINARY_WINDOW_NAME);
  }

  void showRGB(const cv::Mat& rgb) {
    imshow(RGB_WINDOW_NAME, rgb);
  }

  void showBinary(const cv::Mat& binary) {
    imshow(BINARY_WINDOW_NAME, binary);
  }

  void showDepth(const cv::Mat& depth) {
    cv::Mat buf = cv::Mat::zeros(depth.size(), CV_16UC1);
    cv::Mat output = cv::Mat::zeros(depth.size(), CV_8UC3);
    cv::normalize(depth, buf, 0, 2013, cv::NORM_MINMAX);

    for (int i = 0; i < depth.rows * depth.cols; i++) {
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

private:
  std::vector<uint16_t> m_gamma;
  const std::string RGB_WINDOW_NAME;
  const std::string DEPTH_WINDOW_NAME;
  const std::string BINARY_WINDOW_NAME;
};


class SearchTomato
{
public:
  SearchTomato()
    : tomato_contours {},
      siObj {},
      server {},
      f(std::bind(&dynamic_reconfigure_callback, std::placeholders::_1, std::placeholders::_2))
  {
    server.setCallback(f);
  }

  void update(const cv::Mat& capture_rgb) {
    siObj.showRGB(capture_rgb);
    cv::Mat binary_mat = cv::Mat::zeros(capture_rgb.size(), CV_8UC1);
    imageProsessing(capture_rgb, binary_mat);
    siObj.showBinary(binary_mat);
    searchTomatoContours(binary_mat, tomato_contours);
  }

  bool searchTomato(const cv::Mat& capture_depth, geometry_msgs::Point& tomato_point) {
    siObj.showDepth(capture_depth);
    cv::Mat maskedDepth = cv::Mat::zeros(capture_depth.size(), CV_16UC1);
    cv::Mat mask = cv::Mat::zeros(capture_depth.size(), CV_8UC1);
    cv::drawContours(mask, tomato_contours, -1, cv::Scalar(255), CV_FILLED, 8);
    siObj.showBinary(mask);

    capture_depth.copyTo(maskedDepth, mask);
    return findClosePoint(maskedDepth, tomato_point);
  }

  bool searchTomato(const cv::Mat& capture_depth, geometry_msgs::PoseArray& tomato_poses) {
    tomato_poses.poses.clear();
    siObj.showDepth(capture_depth);
    cv::Mat maskedDepth = cv::Mat::zeros(capture_depth.size(), CV_16UC1);
    cv::Mat mask = cv::Mat::zeros(capture_depth.size(), CV_8UC1);
    for (std::size_t i {0}; i < tomato_contours.size(); ++i) {
      cv::drawContours(mask, tomato_contours, i, cv::Scalar(255), CV_FILLED, 8);
      siObj.showBinary(mask);

      capture_depth.copyTo(maskedDepth, mask);
      findClosePoint(maskedDepth, tomato_poses);
    }
    return tomato_poses.poses.empty() ? false : true;
  }

private:
  static void dynamic_reconfigure_callback(kinect_tomato_searcher::SearchConfig& config, uint32_t level)
  {
    h_min_ = config.h_min;
    h_max_ = config.h_max;
    s_min_ = config.s_min;
    s_max_ = config.s_max;
    v_min_ = config.v_min;
    v_max_ = config.v_max;
  }

  void imageProsessing(const cv::Mat& rgb, cv::Mat& binary) {
    cv::Mat blur, hsv;
    cv::GaussianBlur(rgb, blur, cv::Size(5, 5), 4.0, 4.0);
    cv::cvtColor(blur, hsv, CV_BGR2HSV);
    redFilter(hsv, binary);
    cv::dilate(binary, binary, cv::Mat(), cv::Point(-1, -1), 2);
    cv::erode(binary, binary, cv::Mat(), cv::Point(-1, -1), 4);
    cv::dilate(binary, binary, cv::Mat(), cv::Point(-1, -1), 1);
  }

  void searchTomatoContours(const cv::Mat& binary, std::vector<std::vector<cv::Point> >& tomato_contours) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> contour;
    cv::Rect bounding_box;
    float ratio_balance;

    cv::findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    tomato_contours.clear();
    while (!contours.empty()) {
      contour = contours.back();
      contours.pop_back();

      bounding_box = cv::boundingRect(contour);
      ratio_balance = (float)bounding_box.width / (float)bounding_box.height;
      if (ratio_balance > 1.0f) ratio_balance = 1.0f / ratio_balance;

      // delete mismach. that is smaller or spier or empty
      if (bounding_box.area() >= 400 && bounding_box.area() < cv::contourArea(contour)*2 && ratio_balance > 0.4f)
        tomato_contours.push_back(contour);
    }
  }

  void redFilter(const cv::Mat& hsv, cv::Mat& binary) {
    int a, x, y;
    for(y = 0; y < hsv.rows; y++) {
      for(x = 0; x < hsv.cols; x++) {
        a = hsv.step*y+(x*3);
        if((hsv.data[a] <= h_min_ || hsv.data[a] >= h_max_) &&
           (hsv.data[a+1] >= s_min_ || hsv.data[a+1] <= s_max_) &&
           (hsv.data[a+2] >= v_min_ || hsv.data[a+2] <= v_max_))
          binary.at<unsigned char>(y,x) = 255;
        else
          binary.at<unsigned char>(y,x) = 0;
      }
    }
  }

  bool findClosePoint(const cv::Mat& maskedDepth, geometry_msgs::Point& tomato_point) {
    uint16_t depth;
    constexpr uint16_t OVER_RANGE = 3000;
    uint16_t close_depth = OVER_RANGE;

    for (int y = 0; y < maskedDepth.rows; y++) {
      const uint16_t* line_point = maskedDepth.ptr<uint16_t>(y);
      for (int x = 0; x < maskedDepth.cols; x++) {
        depth = line_point[x];
        if (512 < depth && depth <  close_depth) {
          tomato_point.x = depth; // for ros coordinate system
          tomato_point.y = x - maskedDepth.cols / 2; // for ros coordinate system
          tomato_point.z = -(y - maskedDepth.rows / 2); // for ros coordinate system
          close_depth = depth;
        }
      }
    }
    return (close_depth != OVER_RANGE ? true : false);
  }

  bool findClosePoint(const cv::Mat& maskedDepth, geometry_msgs::PoseArray& tomato_poses) {
    geometry_msgs::Point tomato_point {};
    if (findClosePoint(maskedDepth, tomato_point)) {
      geometry_msgs::Pose tomato_pose {};
      tomato_pose.position = tomato_point;
      tomato_pose.orientation.w = 1.0;
      tomato_poses.poses.push_back(tomato_pose);
      return true;
    }
    return false;
  }

  static uint16_t h_min_;
  static uint16_t h_max_;
  static uint16_t s_min_;
  static uint16_t s_max_;
  static uint16_t v_min_;
  static uint16_t v_max_;

  std::vector<std::vector<cv::Point> > tomato_contours;
  ShowImage siObj;
  dynamic_reconfigure::Server<kinect_tomato_searcher::SearchConfig> server;
  dynamic_reconfigure::Server<kinect_tomato_searcher::SearchConfig>::CallbackType f;
};
uint16_t SearchTomato::h_min_;
uint16_t SearchTomato::h_max_;
uint16_t SearchTomato::s_min_;
uint16_t SearchTomato::s_max_;
uint16_t SearchTomato::v_min_;
uint16_t SearchTomato::v_max_;


class ImageConverter {
private:
  ros::NodeHandle nh;
  ros::NodeHandle tomapo_nh;
  ros::Publisher poses_pub;
  geometry_msgs::PoseArray pub_msg;
  image_transport::ImageTransport it;
  image_transport::Subscriber rgb_sub;
  image_transport::Subscriber depth_sub;
  cv_bridge::CvImageConstPtr rgb_ptr;
  cv_bridge::CvImageConstPtr depth_ptr;
  SearchTomato stObj;
  const std::size_t height_;
  const double angle_x_per_piccell_;
  const double angle_y_per_piccell_;
  static constexpr double PI {3.141592653589793};

public:
  ImageConverter(const std::size_t width, const std::size_t height, const double angle_of_view_x, const double angle_of_view_y)
    : nh {},
      tomapo_nh {nh, "tomato_point"},
      poses_pub {tomapo_nh.advertise<geometry_msgs::PoseArray>("array", 1)},
      it {nh},
      rgb_sub {it.subscribe("color", 1, &ImageConverter::rgbCb, this)},
      depth_sub {it.subscribe("depth", 1, &ImageConverter::depthCb, this)},
      stObj {},
      height_ {height},
      angle_x_per_piccell_ {(angle_of_view_x * PI / 180.0) / width},
      angle_y_per_piccell_ {(angle_of_view_y * PI / 180.0) / height}
  {
    pub_msg.header.frame_id = "kinect";
  }

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

    stObj.update(rgb_ptr->image);

    cv::waitKey(100);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      depth_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception by depth: %s", e.what());
    }

    if (stObj.searchTomato(depth_ptr->image, pub_msg)) {
      pub_msg.header.stamp = ros::Time::now();
      for (auto& tomato_pose : pub_msg.poses) {
        tomato_pose.position.y = tomato_pose.position.x * tan(angle_x_per_piccell_ * tomato_pose.position.y) / 1000.0; // convert to m
        tomato_pose.position.z = tomato_pose.position.x * tan(angle_y_per_piccell_ * tomato_pose.position.z) / 1000.0; // convert to m
        tomato_pose.position.x = tomato_pose.position.x / 1000.0; // convert to m
      }
      poses_pub.publish(pub_msg);
    }

    cv::waitKey(100);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_tomato_searcher_node");
  ImageConverter ic {512, 424, 70, 60};
  ros::spin();
  return 0;
}
