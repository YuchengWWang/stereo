#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;


struct CameraIntrinsics {
    float fx;  // 焦距 x
    float fy;  // 焦距 y
    float cx;  // 光轴 x
    float cy;  // 光轴 y
    };
    
void depthToPointCloud(const Mat& depth_map, const CameraIntrinsics& camera_intrinsics, const string& output_file) 
    {
    ofstream ofs(output_file);  // 打开输出文件

    // 写入点云文件头
    ofs << "ply" << endl;
    ofs << "format ascii 1.0" << endl;
    ofs << "element vertex " << depth_map.rows * depth_map.cols << endl;
    ofs << "property float x" << endl;
    ofs << "property float y" << endl;
    ofs << "property float z" << endl;
    ofs << "end_header" << endl;

    for (int y = 0; y < depth_map.rows; y++) {
        for (int x = 0; x < depth_map.cols; x++) {
            float z = depth_map.at<float>(y, x);  // 深度值
            if (z > 0) {  // 忽略无效的深度值
                // 计算点在相机坐标系中的 3D 坐标
                float X = (x - camera_intrinsics.cx) * z / camera_intrinsics.fx;
                float Y = (y - camera_intrinsics.cy) * z / camera_intrinsics.fy;
                float Z = z;

                // 写入点云文件
                ofs << X << " " << Y << " " << Z << endl;
            }
        }
    }

    ofs.close();  // 关闭文件
}

int main()
{
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

    cv::Mat R;
    cv::Vec3d T;
    cv::FileStorage fs_e("../extrinsic.xml", cv::FileStorage::READ);
    if (!fs_e.isOpened()) {
        std::cerr << "无法打开文件 extrinsic.xml" << std::endl;
        return -1;
    }
    fs_e["R"] >> R;
    fs_e["T"] >> T;
    fs_e.release();

    cv::Mat R1, R2, P1, P2, Q;
    cv::FileStorage fs_p("../epipolar.xml", cv::FileStorage::READ);
    if (!fs_p.isOpened()) {
        std::cerr << "无法打开文件 epipolar.xml" << std::endl;
        return -1;
    }
    fs_p["R1"] >> R1;
    fs_p["P1"] >> P1;
    fs_p["R2"] >> R2;
    fs_p["P2"] >> P2;
    fs_p["Q"] >> Q;
    fs_p.release();

    

    string left_img_path = "../left_demo.jpg";
    string right_img_path = "../right_demo.jpg";

    //读取图片
    cv::Mat left_image = cv::imread(left_img_path,cv::IMREAD_COLOR);
    cv::Mat right_image = cv::imread(right_img_path,cv::IMREAD_COLOR);
 
    //图像是否读取成功
    if(left_image.empty() || right_image.empty())
    {
        cout<<"图片读取失败"<<endl;
        return -1;
    }
 

    //去除畸变
    cv::Mat left_undistorted;
    cv::Mat right_undistorted;

    //使用相机内参和畸变系数进行去畸变
    cv::undistort(left_image,left_undistorted,K1,D1,cv::Mat());
    cv::undistort(right_image,right_undistorted,K2,D2,cv::Mat());

    //显示原图和去畸变后的图片
    cv::imshow("left_originalImage",left_image);
    cv::imshow("left_undistortedImage",left_undistorted);
    cv::imshow("right_originalImage",right_image);
    cv::imshow("right_undistortedImage",right_undistorted);
    cv::waitKey(0);

    
    // 进行图像校正
    cv::Mat left_mapx, left_mapy, right_mapx, right_mapy;
    cv::Mat left_final_img, right_final_img;

    cv::initUndistortRectifyMap(K1, D1, R1, P1, left_undistorted.size(), CV_32F, left_mapx, left_mapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, right_undistorted.size(), CV_32F, right_mapx, right_mapy);
    cv::remap(left_undistorted, left_final_img, left_mapx, left_mapy, cv::INTER_LINEAR);
    cv::remap(right_undistorted, right_final_img, right_mapx, right_mapy, cv::INTER_LINEAR);

    Mat left_gray, right_gray;
    cvtColor(left_final_img, left_gray, COLOR_BGR2GRAY);
    cvtColor(right_final_img, right_gray, COLOR_BGR2GRAY);

    cv::imshow("left.jpg", left_final_img);
    cv::imshow("right.jpg", right_final_img);
    cv::waitKey(0);
    

    int minDisparity = 0;         // 最小视差值
    int numDisparities = ((left_final_img.cols/8)+15)&-16;  // 视差范围，必须是 16 的倍数
    int blockSize = 3;           // 匹配块大小
    int sP1 = 8 * left_gray.channels() * blockSize * blockSize;  // SGBM 的参数 P1
    int sP2 = 32 * left_gray.channels() * blockSize * blockSize; // SGBM 的参数 P2


    Ptr<StereoSGBM> sgbm = StereoSGBM::create(minDisparity, numDisparities, blockSize);
    sgbm->setP1(sP1);
    sgbm->setP2(sP2);
    sgbm->setPreFilterCap(63);
    sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);


    // 计算视差图
    Mat disparity;
    sgbm->compute(left_gray, right_gray, disparity);


    // 归一化视差图以便显示
    cv::Mat disparity_normalized;
    cv::normalize(disparity, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

    // 显示视差图
    cv::imshow("视差图", disparity_normalized);
    cv::waitKey(0);  // 按任意键退出

    float baseline = 0.0643562f;  
    float focal_length = 1392.794272729568f; 
    Mat depth_map = Mat::zeros(disparity.size(), CV_32F);

    // 遍历视差图像素，计算深度
    float scale_factor = 16.0f; 
    for (int y = 0; y < disparity.rows; y++) {
        for (int x = 0; x < disparity.cols; x++) {
            short raw_disp = disparity.at<short>(y, x);
            float disp = raw_disp / scale_factor; 
            if (disp > 40) { 
                depth_map.at<float>(y, x) = (focal_length * baseline) / disp;
            }
            else {
            depth_map.at<float>(y, x) = 0.0f; }
        }
    }



    // 归一化深度图以便显示
    Mat depth_normalized;
    normalize(depth_map, depth_normalized, 0, 255, NORM_MINMAX, CV_8U);
    Mat depth_colormap;
    applyColorMap(depth_normalized, depth_colormap, COLORMAP_JET);

    // 显示深度图
    imshow("深度图", depth_colormap);
    waitKey(0);  // 按任意键退出

    imwrite("left_final_img.jpg", left_final_img);
    imwrite("right_final_img.jpg", right_final_img);
    imwrite("left_originalImage.jpg",left_image);
    imwrite("left_undistortedImage.jpg",left_undistorted);
    imwrite("right_originalImage.jpg",right_image);
    imwrite("right_undistortedImage.jpg",right_undistorted);
    imwrite("disparity.jpg", disparity_normalized);
    imwrite("depth.jpg",depth_colormap);


    CameraIntrinsics camera_intrinsics = {1392.794272729568f, 1392.794272729568f, depth_map.cols / 2.0f, depth_map.rows / 2.0f};

    // 将深度图转换为点云并保存为 PLY 文件
    depthToPointCloud(depth_map, camera_intrinsics, "output_point_cloud.ply");

    cout << "点云已保存到 output_point_cloud.ply 文件" << endl;

    return 0;
}