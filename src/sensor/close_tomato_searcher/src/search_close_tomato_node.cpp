#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

class ShowImage {
public:
  ShowImage()
    : m_gamma(2048),
      RGB_WINDOW_NAME("RGB image"),
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
    : si_obj()
  {}

  ~SearchTomato() {}

  bool getPoint(const cv::Mat& capture_rgb, geometry_msgs::Point& tomato_point) {
    si_obj.showRGB(capture_rgb);
    cv::Mat binary = cv::Mat::zeros(capture_rgb.size(), CV_8UC1);
    cv::Rect tomato_box;

    width = capture_rgb.size().width;
    height = capture_rgb.size().height;

    imageProsessing(capture_rgb, binary);
    si_obj.showBinary(binary);
    if (searchTomatoBoundingBox(binary, tomato_box)) {
      tomato_point.x = tomato_box.x + tomato_box.width/2;
      tomato_point.y = tomato_box.y + tomato_box.height/2;
      return true;
    }
    return false;
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

  bool searchTomatoBoundingBox(const cv::Mat& binary, cv::Rect& tomato_box) {
    std::vector<cv::Rect> nominated_boxs; // tomato boxs

    searchCandidateBoxs(binary, nominated_boxs);
    return searchBoxCloseCenter(nominated_boxs, tomato_box);
  }

  void redFilter(const cv::Mat& hsv, cv::Mat& binary) {
    int a, x, y;
    for(y = 0; y < hsv.rows; y++) {
      for(x = 0; x < hsv.cols; x++) {
        a = hsv.step*y+(x*3);
        if((hsv.data[a] <= 5 || hsv.data[a] >= 175)
           && hsv.data[a+1] >= 50
           && hsv.data[a+2] >= 50)
          binary.at<unsigned char>(y,x) = 255;
        else
          binary.at<unsigned char>(y,x) = 0;
      }
    }
  }

  void searchCandidateBoxs(const cv::Mat& binary, std::vector<cv::Rect>& nominated_boxs) {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> contour;
    cv::Rect bounding_box;
    float ratio_balance;

    cv::findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    while (!contours.empty()) {
      contour = contours.back();
      contours.pop_back();

      bounding_box = cv::boundingRect(contour);
      ratio_balance = (float)bounding_box.width / bounding_box.height;
      if (ratio_balance > 1.0f) ratio_balance = 1.0f / ratio_balance;

      // delete mismach. that is smaller or spier or empty
      if (bounding_box.area() >= 400 && bounding_box.area() < cv::contourArea(contour)*2 && ratio_balance > 0.3f)
	nominated_boxs.push_back(bounding_box);
    }
  }

  bool searchBoxCloseCenter(std::vector<cv::Rect>& nominated_boxs, cv::Rect& tomato_box) {
    cv::Rect box;
    int not_likelihood;
    int most_not_likelihood = -1;

    while (!nominated_boxs.empty()) {
      box = nominated_boxs.back();
      nominated_boxs.pop_back();

      not_likelihood = abs(box.x + width/2 - width/2) + abs(box.y + box.height/2 - height/2);
      if (not_likelihood < most_not_likelihood || most_not_likelihood == -1) {
        tomato_box = box;
        most_not_likelihood = not_likelihood;
      }
    }

    return (most_not_likelihood != -1 ? true : false);
  }

  unsigned int width, height;
  ShowImage si_obj;
};

class ImageConverter {
 private:
  ros::NodeHandle nh;
  ros::Publisher pub_obj;
  geometry_msgs::Point pub_msg;
  image_transport::ImageTransport it;
  image_transport::Subscriber camera_sub;
  cv_bridge::CvImageConstPtr camera_ptr;
  SearchTomato st_obj;

 public:
  ImageConverter()
    : it(nh), st_obj()
  {
    pub_obj = nh.advertise<geometry_msgs::Point>("close_tomato_point", 1);
    camera_sub = it.subscribe("/usb_cam/image_raw", 1, &ImageConverter::cameraCb , this);
  }

  ~ImageConverter() {}

  void cameraCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      camera_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception by camera: %s", e.what());
    }

    if (st_obj.getPoint(camera_ptr->image, pub_msg))
      pub_obj.publish(pub_msg);

    cv::waitKey(100);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "search_close_tomato_node");
  ImageConverter ic;
  ros::spin();
  return 0;
}
