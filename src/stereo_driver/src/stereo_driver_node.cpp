#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/core/core_c.h>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
using namespace cv;
using namespace std;
Mat q;
int deviceID = 2;
int apiID = CAP_ANY;
const double camera_factor = 1000;
const double camera_cx = 256;
const double camera_cy = 212;
const double camera_fx = 1000;
const double camera_fy = 1000;


//обработка события нажатия мыши (вычисление расстояния)
void mouseEvent(int evt, int x, int y, int flags, void* param) {
    Mat* rgb = (Mat*)param;
    double rat = ((int)(*rgb).at<uchar>(y, x)), a = 0.879;
        rat = rat / 255;

    double f = q.at<double>(2, 3);
        double T = -1 / q.at<double>(1, 3);
    double k = (1 - pow(1 - rat, 4));
    
    if (evt == EVENT_LBUTTONDOWN) {
            cout << a * 100 * k * f * T / (int)(*rgb).at<uchar>(y, x) << " m" << endl;
    }
    }

//получение очередного кадра, установленного разрешения
Mat another(){
    Mat foto, gray;
    VideoCapture cap;
    cap.open(deviceID, apiID);
    
    cap.set(CAP_PROP_AUTOFOCUS, 0);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1000);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1000);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        exit(-1);

    }
    cap.read(foto);
    cvtColor(foto, gray, cv::COLOR_BGR2GRAY);
    return gray;
}
 
 
sensor_msgs::PointCloud2 object_msg;

 


int main(int argc, char *argv[])
{
	ros::init (argc, argv, "stereo_driver");
	ros::NodeHandle nh; 
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1); 
	sensor_msgs::PointCloud2 output;
	PointCloud::Ptr cloud(new PointCloud);
    if (argc >= 2) { // проверяем количество аргументов
		deviceID = atoi(argv[1]);
	    }
    //объявление и инициализирование параметров стереопары
    Mat cameraMatrix[2], distCoeffs[2], R1, R2, P1, P2, Q;
    Size img_size;
    FileStorage fsp("parameters.yml", FileStorage::READ);
    fsp["cameraMatrixL"] >> cameraMatrix[0];
    fsp["distCoeffsL"] >> distCoeffs[0];
    fsp["RL"] >> R1;
    fsp["PL"] >> P1;
    fsp["cameraMatrixR"] >> cameraMatrix[1];
    fsp["distCoeffsR"] >> distCoeffs[1];
    fsp["RR"] >> R2;
    fsp["PR"] >> P2;
    fsp["Q"] >> Q;
    fsp["size"] >> img_size;

    q = Q;

    //создание карт преобразований
    Mat map1[2], map2[2];
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, img_size, CV_16SC2, map1[0], map2[0]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, img_size, CV_16SC2, map1[1], map2[1]);


    Mat img, Left, Right, L, R;
    Mat disparity, d;

    //создание объекта класса StereoSGBM с заранее вычисленными параметрами
    Ptr<cv::StereoSGBM> st = cv::StereoSGBM::create(1, 48, 3, 50, 1156, 55, 0, 1, 1, 59, cv::StereoSGBM::MODE_SGBM);

    //необходимо для обработки событий с мыши
    //namedWindow("disparity");
    //setMouseCallback("disparity", mouseEvent, &disparity);

    //получение карты смещений в режиме реального времени с возможностью рассчета расстояния до объекта при нажатии на него мышью
    while (ros::ok())
    {
    
        img = another();
        Mat l(img, cv::Rect(0, 0, img.cols / 2, img.rows));
        Mat r(img, cv::Rect(img.cols / 2, 0, img.cols / 2, img.rows));
        l.copyTo(Left);
        r.copyTo(Right);
        remap(Left, L, map1[0], map2[0], INTER_LINEAR, BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        remap(Right, R, map1[1], map2[1], INTER_LINEAR, BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        medianBlur(L, L, 5);
        medianBlur(R, R, 5);
        st->compute(L, R, d);
        d.convertTo(disparity, CV_8U, 255 / (20 * 20.));
        medianBlur(disparity, disparity, 5);
        imshow("disparity", disparity);
        imshow("live", img);
       
	// преобразование матрицы в облако
	/*for (int m = 0; m < d.rows; m++)
		for (int n = 0; n < d.cols; n++)
		{
			//cout << 
			 // Получаем значение в точке (m, n) на карте глубины
			ushort d_pcl = d.ptr<ushort>(m)[n];
			 // d может не иметь значения, если да, пропустите этот пункт
			if (d_pcl == 0)
				continue;
			 // Если d имеет значение, добавляем точку в облако точек
			
 			
			 // Вычислить пространственные координаты этой точки
			//cout<< double(d_pcl) << endl;
			
		}*/
	for (int m = 0; m < d.rows; m++)
		for (int n = 0; n < d.cols; n++)
		{
			//cout << 
			 // Получаем значение в точке (m, n) на карте глубины
			ushort d_pcl = d.ptr<ushort>(m)[n];
			 // d может не иметь значения, если да, пропустите этот пункт
			if (d_pcl == 0)
				continue;
			 // Если d имеет значение, добавляем точку в облако точек
			PointT p;
 			//if(m!=0 && n !=0 && m!=d.rows && n != d.cols)
				//d_pcl = (d.ptr<ushort>(m-1)[n] + d.ptr<ushort>(m+1)[n] + d.ptr<ushort>(m)[n-1] + d.ptr<ushort>(m)[n+1])/4;
			 // Вычислить пространственные координаты этой точки
			//cout<< double(d_pcl) << endl;
			if(d_pcl > 80 and d_pcl < 350){
			p.z = (10-10*double(d_pcl) / (camera_factor));
			p.x = -(n - camera_cx) * p.z / camera_fx;
			p.y = -(m - camera_cy) * p.z / camera_fy;
			 // Получаем его цвет из изображения rgb
			 // rgb - это трехканальное изображение в формате BGR, поэтому получайте цвета в следующем порядке
			//p.b = 999999;
			//p.g = 999999;
			//p.r = 999999;
 
			 // добавляем p в облако точек
			cloud->points.push_back(p);}
		}/**/
	
        //завершение при нажатии Esc
       // if (cv::waitKey(1) == 27) break;
        //sensor_msgs::PointCloud2 output;
        //output = cloud;
      
	 // Очистить данные и выйти
	 cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile("222.pcd", *cloud);

    
	cout << "Point cloud saved." << endl;
	//cloud.reset(new pcl::PointCloud);
	pcl::toROSMsg(*cloud.get(),output );
 	output.header.frame_id = "id_f"; 
	ros::Rate loop_rate(1);
	pcl_pub.publish(output);
	ros::spinOnce(); 
	loop_rate.sleep();
  	//ros::Subscriber sub = nh.subscribe ("input", 1,cloud_cb);
  	//pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  	
  	
  	cloud->points.clear();
 
  // Spin
  	//ros::spin ();
    }
    
    return 0;
}
