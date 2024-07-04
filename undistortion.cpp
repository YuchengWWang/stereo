#include<iostream>
#include<opencv2/opencv.hpp>
#include<string.h>
#include <chrono>
 
using namespace std;
 
int main()
{

    //==================================left=======================================
    
    cv::Mat left_cameraMatrix, left_distCoeffs;
    cv::FileStorage fs_l("../left_param.xml", cv::FileStorage::READ);
    if (!fs_l.isOpened()) {
        std::cerr << "无法打开文件 camera_params.xml" << std::endl;
        return -1;
    }
    cv::Mat cameraMatrix, distCoeffs;
    fs_l["cameraMatrix"] >> left_cameraMatrix;
    fs_l["distCoeffs"] >> left_distCoeffs;
    fs_l.release();

    // left_img_read: 存储左相机原始图片路径
    // left_img_save: 存储左相机畸变校正后图片路径
    string left_img_read = "../images/origin_left";
    string left_img_save = "../image_undistort/left_image_undistort/left";
    std::vector<cv::String> left_images_path;
    std::string left_path = left_img_read+"/*.jpg";
    //   std::string path = "../images/origin_left/*.jpg";
    cv::glob(left_path, left_images_path);
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(int i{0}; i<left_images_path.size(); i++)
    {
        //读取图片
        cv::Mat image = cv::imread(left_images_path[i],cv::IMREAD_COLOR);
    
        //图像是否读取成功
        if(image.empty())
        {
            cout<<"图片读取失败"<<endl;
            return -1;
        }
        //去除畸变
        cv::Mat undistortedImage;
        //使用相机内参和畸变系数进行去畸变
        cv::undistort(image,undistortedImage,left_cameraMatrix,left_distCoeffs,cv::Mat());

        // 保存畸变校正后的图片
        string saved_path = left_img_save + std::to_string(i) + ".jpg";
        cv::imwrite(saved_path, undistortedImage);
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used1 = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "左相机所有采样图片校正畸变耗时: " << time_used1.count() << " 秒. " << endl;



    //==================================right=======================================

    //读取相机内参和畸变系数,这里的相机内参和畸变系数是相机标定的结果，需要提前标定
    cv::Mat right_cameraMatrix, right_distCoeffs;
    cv::FileStorage fs_r("../right_param.xml", cv::FileStorage::READ);
    if (!fs_r.isOpened()) {
        std::cerr << "无法打开文件 right_params.xml" << std::endl;
        return -1;
    }
    fs_r["cameraMatrix"] >> right_cameraMatrix;
    fs_r["distCoeffs"] >> right_distCoeffs;
    fs_r.release();

    // right_img_read: 存储左相机原始图片路径
    // right_img_save: 存储左相机畸变校正后图片路径
    string right_img_read = "../images/origin_right";
    string right_img_save = "../image_undistort/right_image_undistort/right";
    std::vector<cv::String> right_images_path;
    std::string right_path = right_img_read+"/*.jpg";
    //   std::string path = "../images/origin_right/*.jpg";
    cv::glob(right_path, right_images_path);
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    for(int i{0}; i<right_images_path.size(); i++)
    {
        //读取图片
        cv::Mat image = cv::imread(right_images_path[i],cv::IMREAD_COLOR);
    
        //图像是否读取成功
        if(image.empty())
        {
            cout<<"图片读取失败"<<endl;
            return -1;
        }
        //去除畸变
        cv::Mat undistortedImage;
        //使用相机内参和畸变系数进行去畸变
        cv::undistort(image,undistortedImage,right_cameraMatrix,right_distCoeffs,cv::Mat());

        // 保存畸变校正后的图片
        string saved_path = right_img_save + std::to_string(i) + ".jpg";
        cv::imwrite(saved_path, undistortedImage);
    }
    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> time_used2 = chrono::duration_cast<chrono::duration<double>>(t4 - t3);
    cout << "右相机所有采样图片校正畸变耗时: " << time_used2.count() << " 秒. " << endl;


    return 0;
}