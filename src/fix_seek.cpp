//library: cv_bridge, opencv
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber seek_sub_;
  image_transport::Publisher fix_pub_;

  cv_bridge::CvImage *cv_image;
  std_msgs::Header header;

  Mat camImg;
  Mat seekImg;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam_node/image_rect", 1, //MONO8
      &ImageConverter::imageCb, this);
    seek_sub_ = it_.subscribe("/seek_img", 1, //MONO16
      &ImageConverter::seekCb, this);

    fix_pub_ = it_.advertise("/usb_cam_node/image_fix", 1);//RGB8
    
    cv::namedWindow(OPENCV_WINDOW);

    seekImg = Mat::zeros(cv::Size(207,156), CV_8U);
    camImg = Mat::zeros(cv::Size(640,480), CV_8U);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //camImg = Mat::zeros(cv::Size(cv_ptr->image.cols,cv_ptr->image.rows), CV_8U);

    for (int y = 0; y < cv_ptr->image.rows; y++) {
      for (int x = 0; x < cv_ptr->image.cols; x++) {
        camImg.at<uchar>(y, x) = cv_ptr->image.at<uchar>(y, x);
      }
    }

  }


  void seekCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    seekImg =  Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8U);

    int _max = 0x8250;
    int _min = 0x7e50;
    double mean = 0;
    unsigned int count = 0;
    for (int y = 0; y < cv_ptr->image.rows; y++) {
      for (int x = 0; x < cv_ptr->image.cols; x++) {

	//seekImg.at<uchar>(y, x) = cv_ptr->image.at<ushort>(y, x);
        float v = float( cv_ptr->image.at<ushort>(y, x) - _min) / (_max - _min);

        if (v < 0.0) { v = 0; }
        if (v > 1.0) { v = 1; }

        //showImg.at<uchar>(y, x) = (uchar)(v*255.0);
        seekImg.at<uchar>(y, x) = (uchar)(v*255.0);

        //calc sq mean value
        if( y > (int)(cv_ptr->image.rows/2.0) - 30 && y < (int)(cv_ptr->image.rows/2.0) + 30 ){
	  if( x > (int)(cv_ptr->image.cols/2.0) - 30 && x < (int)(cv_ptr->image.cols/2.0) + 30 ){
	    mean += cv_ptr->image.at<ushort>(y, x);
	    count ++;
	  }
	}

      }
    }

    // blur
    medianBlur(seekImg, seekImg, 3);

    mean = mean / (double)count;
    std::cout << "30x30 mean value: " << mean << "\n" << std::endl;

    // Update GUI Window
    //imshow(OPENCV_WINDOW, seekImg);
    //cv::waitKey(1);

    Mat colorImg = Mat::zeros(camImg.rows, camImg.cols, CV_8UC3);
    Mat tmpImg = Mat::zeros(camImg.rows, camImg.cols, CV_8U);

    resize(seekImg, tmpImg, tmpImg.size(), cv::INTER_CUBIC);
    //cv_ptr_r->image = tmp.clone();

    // roll & scale 
    float angle = 0.0, scale = .7;
    int centX = tmpImg.cols*0.5;
    int centY = tmpImg.rows*0.5;
    // center
    cv::Point2f center(centX, centY);
    // affin
    const Mat affine_matrix = cv::getRotationMatrix2D( center, angle, scale );
    cv::warpAffine(tmpImg, tmpImg, affine_matrix, tmpImg.size());

    for (int y = 0; y < camImg.rows; y++) {
      for (int x = 0; x < camImg.cols; x++) {

        colorImg.at<Vec3b>(y, x)[0] = camImg.at<uchar>(y,x);
        colorImg.at<Vec3b>(y, x)[1] = camImg.at<uchar>(y,x)*0.5; 
        colorImg.at<Vec3b>(y, x)[2] = tmpImg.at<uchar>(y,x) * 4;//R

      }
    }

//    std::cout << "cent: " << centX << "," << centY << "\n" << std::endl;
//    std::cout << "rect: " << (int)(camImg.cols*scale) << "," << (int)(camImg.rows*scale) << "\n" << std::endl;

    int sizeX = (int)(camImg.cols*scale);
    int sizeY = (int)(camImg.rows*scale);
    Mat fixImg(colorImg,Rect( centX - (int)(sizeX/2.0), centY - (int)(sizeY/2.0), sizeX, sizeY));

    cv_image = new cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, fixImg);
    fix_pub_.publish(cv_image->toImageMsg());

    imshow(OPENCV_WINDOW, fixImg);
    cv::waitKey(1);
 
  }



};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;

}
