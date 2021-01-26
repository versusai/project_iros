#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <ctime>
#include <std_msgs/Float32.h>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher pub_vx;
  std_msgs::Float32 past_msg;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/rexrov/rexrov/camera/camera_image", 1,
      &ImageConverter::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);
    pub_vx = nh_.advertise<std_msgs::Float32>("vx", 1000);

    past_msg.data = 0.;
    

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    clock_t start, finish;
    start = clock();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());

    Mat img = cv_ptr->image.clone();

    Mat LUT1 = Mat::zeros(Size(1,256), CV_32F);
    // cout << "imgC = \n " << LUT << "\n\n";


    for(int i = 0; i < 256; i++){
        if(i==0){
            LUT1.at<float>(i,0) = -logf(0.1/255);
        }
        else{
            LUT1.at<float>(i,0) = -logf(((float)i)/255.0);
        }
    }

    // Mat LUT(Size(1,5), CV_8U);
    cout << "imgC = \n " << img.size().width << "\n\n";

    Mat FULL = Mat::zeros(Size(img.size().width,img.size().height), CV_8UC3);
    Mat INPUT = Mat::zeros(Size(img.size().width,img.size().height), CV_8UC3);

    Mat DEPTH = Mat::zeros(Size(img.size().width,img.size().height), CV_32F);
    Mat PRINTD = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);
    Mat DEPTHC = Mat::zeros(Size(img.size().width,img.size().height), CV_32FC3);
    Mat PRINTDC = Mat::zeros(Size(img.size().width,img.size().height), CV_8UC3);

    Mat ORIGINAL = Mat::zeros(Size(img.size().width,img.size().height), CV_8UC3);
    Mat MIN = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);
    Mat UDCP = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);

    Mat BINARY = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);

    Mat RED = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);
    Mat GREEN = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);
    Mat BLUE = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);
    Mat RED2 = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);
    Mat GREEN2 = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);
    Mat BLUE2 = Mat::zeros(Size(img.size().width,img.size().height), CV_8U);

    ORIGINAL = img.clone();
    // INPUT = ORIGINAL.clone();

    INPUT = ORIGINAL.clone();

    // cvtColor(fRGB, INPUT, COLOR_BGR2RGB);

    // namedWindow("INPUT");
    // imshow("INPUT", INPUT);
    // waitKey(0);

    // int patch=PATCH;
    int highest=0;
    int min_h=0, min_w=0;
    for(int i=0; i<img.size().height;i++){
        for(int j=0; j<img.size().width; j++){
            MIN.at<uchar>(i,j) = INPUT.at<Vec3b>(i,j)[0]>INPUT.at<Vec3b>(i,j)[1]?INPUT.at<Vec3b>(i,j)[1]:INPUT.at<Vec3b>(i,j)[0];
            if(MIN.at<uchar>(i,j)>highest && i < 100){
                min_h=i;
                min_w=j;
                highest=MIN.at<uchar>(i,j);
            }
            

        }
    }

    // imshow("MIN", MIN);
    // waitKey(0);

    unsigned char A1=INPUT.at<Vec3b>(min_h,min_w)[0];
    unsigned char A2=INPUT.at<Vec3b>(min_h,min_w)[1];
    unsigned char A3=INPUT.at<Vec3b>(min_h,min_w)[2];

    float const1=255.0/A1,const2=255.0/A2,const3=255.0/A3;

    // cout << "min_h: " << min_h << endl;
    // cout << "min_w: " << min_w << endl;
    // cout << "highest: " << highest << endl;
    // cout << "A1: " << float(A1) << endl;
    // cout << "A2: " << float(A2) << endl;
    // cout << "A3: " << float(A3) << endl;

    Mat channel[3];

    split(INPUT, channel);



    // Mat Blue2;
    // ConvertScale(channel[0],BLUE2,const1,0);
    // channel[0].convertTo(BLUE2, CV_8U, const1, 0);

    Mat KERNEL = getStructuringElement(MORPH_RECT, Size(15,15));

    erode(MIN, UDCP, KERNEL, Point(7,7));

    for(int i=0;i<img.size().height;i++){
        for(int j=0;j<img.size().width;j++){
            UDCP.at<uchar>(i,j) = 255 -(unsigned char)(0.95*UDCP.at<uchar>(i,j));
        }
    }

    // imshow("UDCP", UDCP);
    // waitKey(0);
    
    LUT(UDCP, LUT1, DEPTH);

    medianBlur(DEPTH, DEPTH, 5);
    GaussianBlur(DEPTH, DEPTH, Size(11, 11), 0);

    // imshow("DEPTh", DEPTH);
    // waitKey(0);

    cvtColor(DEPTH, DEPTHC, COLOR_GRAY2RGB);

    // cout << "Hello World!\n";
    // cout << log(0.1/255);

    DEPTH.convertTo(BINARY, CV_8U, 255.0/1.1, 0);

    threshold(BINARY, BINARY, 16, 255, THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(100,100));

    erode(BINARY, BINARY, element, Point(45,45));

    imshow("binary", BINARY);
    waitKey(3);

    int sumzero = countNonZero(BINARY);

    // cout << "sumzero " << sumzero << endl;

    float area_rov = M_PI*45.0*45.0;

    float radius_rov = 45*2;

    bool find_area = false;

    double ctmassx = 0.0, ctmassy = 0.0;
    double meandepth = 0.0;
    int pixels = 0;

    if(sumzero > area_rov){
        vector<vector<Point> > cont;
        vector<Vec4i> h;
        findContours(BINARY, cont, RETR_TREE, CHAIN_APPROX_SIMPLE);

        Scalar color( 0, 255, 0 );
        Mat img_drawed;
        img_drawed = INPUT.clone();
        drawContours(img_drawed, cont, -1, color, FILLED);

        // aquiiii -------------

        int max_index_area = 0;
        double area_temp = 0.;
        int i = 0;
        for(int i = 0; i < cont.size(); i++){
            double area = contourArea(cont[i]);
            if(area_temp < area){
                max_index_area = i;
                area_temp = area;
            }
        }

        // cout << "vai " << max_index_area << endl;
        // cout << "vai " << contourArea(cont[max_index_area]) << endl;

        Mat img_center = BINARY.clone();

        for(int i = 0; i < cont.size(); i++){
            if(i != max_index_area){
                drawContours(img_center, cont, i, 0, FILLED);
            }
        }

        // imshow("iamge test2", img_center);
        // waitKey(0);

        if(cont.size() > 0){
            for(int i=0; i<img.size().height;i++){
                for(int j=0; j<img.size().width; j++){
                    // double area = contourArea(cont[c]);
                    if((int)img_center.at<uchar>(i,j) == 255){
                        // if(area > area_rov){
                        pixels++;
                        ctmassx += j;
                        ctmassy += i;
                        meandepth += DEPTH.at<uchar>(i,j);
                        // }
                    }
                }
            }
        }
        ctmassx /= pixels;
        ctmassy /= pixels;
        meandepth = pixels;
        cout << "meand " << meandepth <<  endl;

        cout << "aqui1: " << pixels << " " << ctmassx << " " << ctmassy << endl;

        cvtColor(DEPTH, DEPTHC, COLOR_GRAY2RGB);

        if(cont.size() == 1){
            circle(img_drawed, Point(int(ctmassx), int(ctmassy)), radius_rov, Scalar(255,255,0), 5);
            circle(DEPTHC, Point(int(ctmassx), int(ctmassy)), radius_rov, Scalar(255,255,0), 5);
            find_area = true;
        }else{
            RotatedRect ellipse1 = fitEllipse(cont[max_index_area]);
            // cout << "aqui34 " << ellipse1.size.width << endl;
            if(radius_rov <= ellipse1.size.width && radius_rov <= ellipse1.size.height){
                // ellipse(img_drawed, ellipse1, Scalar(0, 255,255), 3);
                circle(img_drawed, Point(int(ctmassx), int(ctmassy)), radius_rov, Scalar(255,255,0), 5);
                // ellipse(DEPTHC, ellipse1, Scalar(0, 255,255), 3);
                circle(DEPTHC, Point(int(ctmassx), int(ctmassy)), radius_rov, Scalar(255,255,0), 5);
                find_area = true;
            }
        }

        // imshow("iamge test img_drawed", img_drawed);
        // waitKey(0);
        time(&finish);
	    // cout << "Time required = " << (double(clock() - start))/CLOCKS_PER_SEC * 1000 << " seconds" << endl;
        // imshow("iamge test img_depfth", DEPTHC);
        // waitKey(3);

        // drawContours(img_binary, cont, -1, color, FILLED);

        // imshow("image test1", img_drawed);
        // waitKey(0);

        
    }

    if(find_area){
      double maxside=img.size().width/2.0;
      double maxtop=img.size().height/2.0;

      double vx,vy,vi;
      vi=fmin(meandepth/1.1,1.0);
      vx=(ctmassx-maxside)/maxside;
      vy=-(ctmassy-maxtop)/maxtop;
      std_msgs::Float32 msg;

      msg.data = vx;

      pub_vx.publish(msg);

      past_msg.data = vx;

      char str[25];

      sprintf(str,"%.2lf %.2lf %.2lf",vi,vx,vy);

      putText(DEPTHC,str, Point((int(img.size().width/2.0)-125),(img.size().height-50)), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0,255), 2);

      imshow("final", DEPTHC);
      waitKey(1);
    }else{
      double maxside=img.size().width/2.0;
      double maxtop=img.size().height/2.0;

      double vx,vy,vi;
      vi=0.;
      vx=-1.;
      vy=0.;
      // std_msgs::Float32 msg;

      // msg.data = vx;

      pub_vx.publish(past_msg);

      char str[25];

      sprintf(str,"%.2lf %.2lf %.2lf",vi,vx,vy);

      putText(DEPTHC,str, Point((int(img.size().width/2.0)-125),(img.size().height-50)), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0,255), 2);

      imshow("final", DEPTHC);
      waitKey(3);

    }


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}