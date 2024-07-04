#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include<unistd.h> 
#include <chrono>

using namespace std;

// 左右相机内参标定
// 定义棋盘格维度，{6,4}代表行内点数为6，列内点数为4
int CHECKERBOARD[2]{8,5}; 
struct ReturnData {
	cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
};

ReturnData inter_calibration(string read_path_in, string save_path_in, string initrinsic_path)
{
// objpoints中每个元素都是一个小vector，每个小vector存储的每个元素都是opencv的cv::Point3f数据结构
  // n * 54 * 3 * 1
  std::vector<std::vector<cv::Point3f> > objpoints;

  // imgpoints中每个元素都是一个小vector，每个小vector存储的每个元素都是opencv的cv::Point2f数据结构
  // n * 54 * 2 * 1
  std::vector<std::vector<cv::Point2f> > imgpoints;

  // objp:,记录单张棋盘格内点的3d位置索引
  // 指定棋盘格坐标点时，按照先从上到下，后从左到右的顺序记录。每一行棋盘格的记录方式：(y索引, x索引， 0）
  std::vector<cv::Point3f> objp;

  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }


  // images_path，存储所有棋盘格图片的存储路径
  std::vector<cv::String> images_path;
  std::string path = read_path_in+"/*.jpg";
//   std::string path = "../images/origin_left/*.jpg";
  cv::glob(path, images_path);
//   std::string saved_path;

  cv::Mat frame, gray;
  
  // corner_pts，记录检测到的棋盘格54个内点的2D像素坐标 
  std::vector<cv::Point2f> corner_pts;
  // success，用于判断是否成功检测到棋盘格
  bool success;

  // 开始计时
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  
  for(int i{0}; i<images_path.size(); i++)
  { 
    chrono::steady_clock::time_point t11 = chrono::steady_clock::now();
    
    // 图像大小 640 x 480
    frame = cv::imread(images_path[i]);
    std::cout << images_path[i] << std::endl;
    cv::cvtColor(frame,gray, cv::COLOR_BGR2GRAY);

    // OpenCV函数寻找棋盘格
    success = cv::findChessboardCorners(gray,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    if(success)
    {
      cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

      // 进一步refine检测到的网格内点的坐标精度
      // 这里cornerSubPix函数直接在原有corner_pts基础上进行覆盖，不会多创建一个新的变量再赋值
      cv::cornerSubPix(gray, corner_pts, cv::Size(11,11), cv::Size(-1,-1), criteria);

      // 作图，棋盘格检测结果
      cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts, success);

      objpoints.push_back(objp);
      imgpoints.push_back(corner_pts);
    }
    
//     cv::imshow("Image", frame);
//     cv::waitKey(10);
    
    string saved_path = save_path_in + std::to_string(i) + ".jpg";
    cv::imwrite(saved_path, frame);
    
    chrono::steady_clock::time_point t22 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t22 - t11);
    cout << "每一张图片处理耗时: " << time_used.count() << " 秒. " << endl;

  }

//   cv::destroyAllWindows();
  
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used1 = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "整体耗时: " << time_used1.count() << " 秒. " << endl;

  
  // 内参矩阵、畸变系数、旋转矩阵R、平移向量T
  cv::Mat cameraMatrix, distCoeffs, R, T;

  chrono::steady_clock::time_point t111 = chrono::steady_clock::now();
  
  // 这里注意参数顺序，必须先cols后rows
  cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.cols,gray.rows), cameraMatrix, distCoeffs, R, T);
  
  chrono::steady_clock::time_point t222 = chrono::steady_clock::now();
  chrono::duration<double> time_used_cali = chrono::duration_cast<chrono::duration<double>>(t222 - t111);
  cout << "求畸变参数耗时: " << time_used_cali.count() << " 秒. " << endl;

  
  cv::FileStorage fs(initrinsic_path, cv::FileStorage::WRITE);

    // 写入数据
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;

    // 释放文件
    fs.release();
    return ReturnData{cameraMatrix,distCoeffs};
};


int main()
{
  //左侧相机内参
  printf("\n左侧相机内参\n:");
  string left_path = "../images/origin_left";
  string left_save_path = "../calib_images_left/";
  string left_intrinsic_path = "../left_param.xml";
  inter_calibration(left_path, left_save_path, left_intrinsic_path);
  
  //右侧相机内参
  printf("\n右侧相机内参:\n");
  string right_path = "../images/origin_right";
  string right_save_path = "../calib_images_right/";
  string right_intrinsic_path = "../right_param.xml";
  inter_calibration(right_path, right_save_path, right_intrinsic_path);
  return 0;
}


