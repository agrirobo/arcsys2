#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>

class ShowImage {
public:
  ShowImage()
    : m_gamma(2048),
      RGB_WINDOW_NAME("RGB image"),
      DEPTH_WINDOW_NAME("Depth image"),
      BINARY_WINDOW_NAME("Binary image")
  {
    float v;
    for(int i = 0; i < 2048; i++) {
      v = i/2048.0;
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


class SearchTomato {
public:
  SearchTomato()
    : tomato_contours()
    , siObj()
  {}

  ~SearchTomato() {}

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

private:
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
  if((hsv.data[a] <= 5 || hsv.data[a] >= 175) && hsv.data[a+1] >= 50 && hsv.data[a+2] >= 50 )
    binary.at<unsigned char>(y,x) = 255;
  else
    binary.at<unsigned char>(y,x) = 0;
      }
    }
  }

  bool findClosePoint(const cv::Mat& maskedDepth, geometry_msgs::Point& tomato_point) {
    uint16_t depth;
    uint16_t close_depth = 4096;

    for (int y = 0; y < maskedDepth.rows; y++) {
      const uint16_t* line_point = maskedDepth.ptr<uint16_t>(y);
      for (int x = 0; x < maskedDepth.cols; x++) {
  depth = line_point[x];
  if (256 < depth && depth <  close_depth) {
    tomato_point.x = x;
    tomato_point.y = y;
    tomato_point.z = depth;
    close_depth = depth;
  }
      }
    }
    return (close_depth != 4096 ? true : false);
  }

  std::vector<std::vector<cv::Point> > tomato_contours;
  ShowImage siObj;
};


class ImageConverter {
private:
  ros::NodeHandle nh;
  ros::Publisher point_pub;
  geometry_msgs::Point pub_msg;
  image_transport::ImageTransport it;
  image_transport::Subscriber rgb_sub;
  image_transport::Subscriber depth_sub;
  cv_bridge::CvImageConstPtr rgb_ptr;
  cv_bridge::CvImageConstPtr depth_ptr;
  SearchTomato stObj;

public:
  ImageConverter()
    : it(nh)
    , stObj()
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

    stObj.update(rgb_ptr->image);

    cv::waitKey(100);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      depth_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception by depth: %s", e.what());
    }

    if (stObj.searchTomato(depth_ptr->image, pub_msg))
      point_pub.publish(pub_msg);

    cv::waitKey(100);
  }
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_tomato_group_search_node");
  ImageConverter ic;
  ros::spin();
  return 0;
}
