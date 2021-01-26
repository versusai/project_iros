#include<iostream>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<iostream>
// #include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types_c.h>
#include<cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;

#define FULLTIME true//Show Every Time

#define WIDTH 1280 //Image Size
#define HEIGHT 720 //Image Size
#define FACTOR 0.5//0.5 //Rescale the Frames - Default = 1 --- 1280 x 720 pixels

#define ROI false//Cut the input image
#define ROIX int(0*(WIDTH*FACTOR)/10.0)
#define ROIY int(5*(HEIGHT*FACTOR)/10.0)
#define ROIW int(10*(WIDTH*FACTOR)/10.0)
#define ROIH int(5*(HEIGHT*FACTOR)/10.0)

#define MIN_HEIGHT 100 //Search window of the Global Light

#define REFPATCH 15 //20<p<60//Size of the DCP Patch

#define VCOLOR 1
#define VNORM 2
#define VDCP 4
#define VDEPTH 16
#define VBINARY 32
#define VBINARY2 64
#define VCONTOURC 128
#define VCONTOURD 256
#define VESCAPEC 512
#define VESCAPED 1024
#define VBINARYOBJ 2048

//Both are defined by the type of image
#define VISUALIZATION VCOLOR|VCONTOURC|VBINARY2|VESCAPED
#define IMAGEM VCOLOR|VESCAPED|VCONTOURC|VDEPTH|VDCP|VBINARY2

#define DCP 0 //0 UDCP 1 DCP

#define OUTWATER 0 //0 if UNDERWATER

#define THRESH_DEPTH 50//220 Minimum Threshold for Depth to Avoid Obstacle (Just reference)
#define TYPE_THRE CV_THRESH_BINARY //| CV_THRESH_OTSU //CV_THRESH_BINARY_INV

#define RADIUS_ROV 45*(FACTOR*2) //45Define area for ROV movements
#define MIN_POINT 10*(FACTOR*2) // 10 Number minimum of pixel in a contour
#define AREA_ROV M_PI*RADIUS_ROV*RADIUS_ROV //pixels
#define COR_FAC 1.0 //1.2 //Correction Factor between Radius_ROV and Erode Radius

#define TURN true //Turn when no obstacle found

//#define DARK 0 //Use dark object
#ifdef DARK
    #define THRESH_OBJ 80 // Maximun Threshold for Depth for Critical Obstacles (Just reference)
    #define TYPE_OBJ CV_THRESH_BINARY_INV// | CV_THRESH_OTSU
    #define KT_OBJ 25 //150 - Gain to move top-bottom
    #define KS_OBJ 45//25 //200 - Gain to left-right due near objects
    #define AREA_OBJ AREA_ROV
#endif

#define KT 25//50 //150 - Gain to move top-bottom
#define KS 30//100 //200 - Gain to move to left-right
#define FORWARD false //Go forward
#define FORWARD_PWM 50//20 Constant Speed Forward

#define MIN_MOV 60 //Turning Movement

//Show the commands send to the robot
#ifdef DARK
    #define SEND false
#else
    #define SEND true
#endif

#define DEPTH_MAX 1500+KT //Speed in case of unable to identify escape region

unsigned int PATCH = (((unsigned int)round(REFPATCH*FACTOR*2))%2==0?round(REFPATCH*FACTOR*2)+1:round(REFPATCH*FACTOR*2));
int PATCHSMOOTH = (((int)round((REFPATCH/3.0)*FACTOR*2))%2==0?round((REFPATCH/3.0)*FACTOR*2)+1:round((REFPATCH/3.0)*FACTOR*2));

int main() {

    Mat img = imread("Test3.png");

    // Mat LUT(Size(1,5), CV_8UC1);

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
    imshow("INPUT", INPUT);
    waitKey(0);

    int patch=PATCH;
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

    imshow("MIN", MIN);
    waitKey(0);

    unsigned char A1=INPUT.at<Vec3b>(min_h,min_w)[0];
    unsigned char A2=INPUT.at<Vec3b>(min_h,min_w)[1];
    unsigned char A3=INPUT.at<Vec3b>(min_h,min_w)[2];

    float const1=255.0/A1,const2=255.0/A2,const3=255.0/A3;

    cout << "min_h: " << min_h << endl;
    cout << "min_w: " << min_w << endl;
    cout << "highest: " << highest << endl;
    cout << "A1: " << float(A1) << endl;
    cout << "A2: " << float(A2) << endl;
    cout << "A3: " << float(A3) << endl;

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

    imshow("UDCP", UDCP);
    waitKey(0);
    
    LUT(UDCP, LUT1, DEPTH);

    medianBlur(DEPTH, DEPTH, 5);
    GaussianBlur(DEPTH, DEPTH, Size(11, 11), 0);

    imshow("DEPTh", DEPTH);
    waitKey(0);

    // cout << "Hello World!\n";
    // cout << log(0.1/255);

    DEPTH.convertTo(BINARY, CV_8U, 255.0/1.1, 0);

    threshold(BINARY, BINARY, 10, 255, THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(100,100));

    erode(BINARY, BINARY, element, Point(45,45));

    imshow("binary", BINARY);
    waitKey(0);

    int sumzero = countNonZero(BINARY);

    cout << "sumzero " << sumzero << endl;

    float area_rov = M_PI*45.0*45.0;

    float radius_rov = 45*2;

    bool find_area = false;

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

        cout << "vai " << max_index_area << endl;
        cout << "vai " << contourArea(cont[max_index_area]) << endl;

        Mat img_center = BINARY.clone();

        for(int i = 0; i < cont.size(); i++){
            if(i != max_index_area){
                drawContours(img_center, cont, i, 0, FILLED);
            }
        }

        imshow("iamge test2", img_center);
        waitKey(0);

        double ctmassx = 0.0, ctmassy = 0.0;
        double meandepth = 0.0;
        int pixels = 0;

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
        }else{
            RotatedRect ellipse1 = fitEllipse(cont[max_index_area]);
            cout << "aqui34 " << ellipse1.size.width << endl;
            if(radius_rov <= ellipse1.size.width && radius_rov <= ellipse1.size.height){
                ellipse(img_drawed, ellipse1, Scalar(0, 255,255), 3);
                circle(img_drawed, Point(int(ctmassx), int(ctmassy)), radius_rov, Scalar(255,255,0), 5);
                ellipse(DEPTHC, ellipse1, Scalar(0, 255,255), 3);
                circle(DEPTHC, Point(int(ctmassx), int(ctmassy)), radius_rov, Scalar(255,255,0), 5);
                find_area = true;
            }
        }

        imshow("iamge test img_drawed", img_drawed);
        waitKey(0);
        imshow("iamge test img_depfth", DEPTHC);
        waitKey(0);

        // drawContours(img_binary, cont, -1, color, FILLED);

        // imshow("image test1", img_drawed);
        // waitKey(0);



    }


}