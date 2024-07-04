#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <iostream>

#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <opencv2/imgproc/types_c.h>


using namespace std;
using namespace cv;

// 双目参数标定

vector< vector< Point3f > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2f > > left_img_points, right_img_points;
Mat img1, img2, gray1, gray2;


/*
 * board_width： 棋盘格宽方向上的内点数量，此处为9
 * board_height： 棋盘格高方向上的内点数量，此处为6
 * num_imgs： 图片数量
 * square_size： 黑白格真实世界中的边长，单位为米，此处为0.02423米
 * leftimg_dir： 左目图片存储文件夹位置，此处为"../calib_imgs/1/"
 * rightimg_dir： 右目图片存储文件夹位置，此处为"../calib_imgs/1/"
 * leftimg_filename： 左目相机拍摄的图片名称，此处为"left"
 * rightimg_filename： 右目相机拍摄的图片名称，此处为"right"
 * extension： 图片后缀，此处为"jpg"
 */

void load_image_points(int board_width, int board_height, int num_imgs, float square_size,
                       char* leftimg_dir, char* rightimg_dir,
                       char* leftimg_filename, char* rightimg_filename, char* extension) {

  Size board_size = Size(board_width, board_height);
  int board_n = board_width * board_height;
  int all = 0;
  
  for (int k = 1; k <= num_imgs; k++) {
      
    char left_img[1000], right_img[1000];
    
    sprintf(left_img, "%s%s%d.%s", leftimg_dir, leftimg_filename, k, extension);
    sprintf(right_img, "%s%s%d.%s", rightimg_dir, rightimg_filename, k, extension);

    cout << left_img << endl;
    cout << right_img << endl;
    
    img1 = imread(left_img, CV_LOAD_IMAGE_COLOR);
    img2 = imread(right_img, CV_LOAD_IMAGE_COLOR);
    
    if(img1.data==nullptr | img2.data==nullptr)
    {
        continue;
    }

    //色彩转换
    cvtColor(img1, gray1, CV_BGR2GRAY);
    cvtColor(img2, gray2, CV_BGR2GRAY);

    bool found1 = false, found2 = false;

    // 棋盘格检验
    found1 = cv::findChessboardCorners(img1, board_size, corners1);
    found2 = cv::findChessboardCorners(img2, board_size, corners2);
    
    // 必须要求左目图像、右目图像都检测到棋盘格
    if(!found1 || !found2){
      cout << "棋盘格未能找到!" << endl;
      continue;
    } 
    
    all = all + 1;
    cout << "The total: " << all << endl;
    
    if (found1)
    { 
      // 进一步refine检测到的网格内点的坐标精度 
      cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      
      // 作图，可视化
      drawChessboardCorners(img1, board_size, corners1, found1);
      cv::imshow("left", img1);
      cv::waitKey(10);
    }
    
    if (found2)
    { 
      // 进一步refine检测到的网格内点的坐标精度
      cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      
      // 作图，可视化
      cv::drawChessboardCorners(img2, board_size, corners2, found2);
      cv::imshow("right", img2);
      cv::waitKey(10);
    }
    
    // 将棋盘格真实3D坐标点，依次append进入obj中，这里需要将索引乘以棋盘格边长(0.02423米)，以固定真实世界尺度
    vector< Point3f > obj;
    for (int i = 0; i < board_height; i++)
      for (int j = 0; j < board_width; j++)
        obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
    
    if (found1 && found2) {
      cout << "第"  << k << "张图像，左右目均已经检测到棋盘格内点!" << endl;
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
    }
  }
  
  // 这里重复赋值一次，是为了保证两个容器存储的样本数相同，正好一一对应
  for (int i = 0; i < imagePoints1.size(); i++) {
    vector< Point2f > v1, v2;
    for (int j = 0; j < imagePoints1[i].size(); j++) {
        
      v1.push_back(Point2f((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
      v2.push_back(Point2f((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
        
    }
    left_img_points.push_back(v1);
    right_img_points.push_back(v2);
  }
}


int main(int argc, char const *argv[])
{
  int board_width = 8;
  int board_height = 5;
  int num_imgs = 20;
  float square_size = 0.0295;
  char leftimg_dir[] = "../images/origin_left/";
  char rightimg_dir[] = "../images/origin_right/";
  char leftimg_filename[] = "left_";
  char rightimg_filename[] = "right_";
  char extension[] = "jpg";
  
  // 将一系列2D观测坐标和3D观测坐标，对应存储到object_points, image_points中
  load_image_points(board_width, board_height, num_imgs, square_size,
                   leftimg_dir, rightimg_dir, leftimg_filename, rightimg_filename, extension);
  

  // 这是根据calib_left.cpp、calib_right.cpp标定得到的内参矩阵K1、K2，和畸变系数向量D1、D2
  cv::Mat K1, D1;
    cv::FileStorage fs_l("../left_param.xml", cv::FileStorage::READ);
    if (!fs_l.isOpened()) {
        std::cerr << "无法打开文件 camera_params.xml" << std::endl;
        return -1;
    }
    cv::Mat cameraMatrix, distCoeffs;
    fs_l["cameraMatrix"] >> K1;
    fs_l["distCoeffs"] >> D1;
    fs_l.release();

  cv::Mat K2, D2;
    cv::FileStorage fs_r("../right_param.xml", cv::FileStorage::READ);
    if (!fs_r.isOpened()) {
        std::cerr << "无法打开文件 right_params.xml" << std::endl;
        return -1;
    }
    fs_r["cameraMatrix"] >> K2;
    fs_r["distCoeffs"] >> D2;
    fs_r.release();


  
  // 左目摄像头和右目摄像头之间的旋转矩阵R、平移向量T，E是本质矩阵，F是基础矩阵
  Mat R, F, E;
  Vec3d T;
  
  // 双目相机标定，object_points, left_img_points, right_img_points, K1, D1, K2, D2, img1.size都是传入参数，R, T, E, F是输出结果
  stereoCalibrate(object_points, left_img_points, right_img_points, K1, D1, K2, D2, img1.size(), R, T, E, F);
  
  cout << "双目标定结果：" << endl;
  cout << "R:"<< endl <<R << endl;
  cout << "T:"<< endl <<T << endl;
  cout << "Essential:"<< endl <<E << endl;
  cout << "Fundamental:"<< endl <<F << endl;
  
  cv::FileStorage fs_1("../extrinsic.xml", cv::FileStorage::WRITE);

    // 写入数据
    fs_1 << "R" << R;
    fs_1 << "T" << T;
    fs_1 << "Essential" << E;
    fs_1 << "Fundamental" << F;

    // 释放文件
    fs_1.release();

  // 极线校正
  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(K1, D1, K2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
  
  cout << "进一步双目校正结果：" << endl;
  cout << "R1:"<< endl <<R1 << endl;
  cout << "P1:"<< endl <<P1 << endl;
  cout << "R2:"<< endl <<R2 << endl;
  cout << "P2:"<< endl <<P2 << endl;
  cout << "Q:"<< endl <<Q << endl;

  cv::FileStorage fs_2("../epipolar.xml", cv::FileStorage::WRITE);

    // 写入数据
    fs_2 << "R1" << R1;
    fs_2 << "P1" << P1;
    fs_2 << "R2" << R2;
    fs_2 << "P2" << P2;
    fs_2 << "Q" << Q;

    // 释放文件
    fs_2.release();

  return 0;
}