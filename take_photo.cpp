#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

Mat  frame, img, gray, dst;
Mat leftImg, rightImg;

// 照片采集

int main(int argc, char** argv)
{

	//videocapture结构创建一个catture视频对象
	VideoCapture capture;
	//连接视频
	capture.open(0); 
	if (!capture.isOpened()) {
		printf("could not load video data...\n");
		return -1;
	}
	int frames = capture.get(CAP_PROP_FRAME_COUNT);//获取视频帧数目(一帧就是一张图片)
	int fps = capture.get(CAP_PROP_FPS);//获取每帧视频的频率
	capture.set(CAP_PROP_FRAME_WIDTH,1280);
	capture.set(CAP_PROP_FRAME_HEIGHT,480);
	int WIDTH = capture.get(CAP_PROP_FRAME_WIDTH);
	int HEIGHT = capture.get(CAP_PROP_FRAME_HEIGHT);
	// 获取帧的视频宽度，视频高度
	Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH), capture.get(CAP_PROP_FRAME_HEIGHT));
	cout << frames << endl;
	cout << fps << endl;
	cout << size << endl;
	capture >> frame;
	cout << frame.cols << "  " << frame.rows << endl;
	// 创建视频中每张图片对象
	namedWindow("video-demo", WINDOW_AUTOSIZE);
	// 循环显示视频中的每张图片
	int count = 1;

	for (;;)
	{
		//将视频转给每一张张图进行处理
		capture >> frame;
		//省略对图片的处理
		//视频播放完退出
		if (frame.empty())break;
		imshow("video-demo", frame);

		//拆分视屏为左右两框
		Rect leftRect(0, 0, WIDTH >> 1, HEIGHT);   //创建一个Rect框，属于cv中的类，四个参数代表x,y,width,height
		Rect rightRect(WIDTH >> 1, 0, WIDTH >> 1, HEIGHT);
		//Rect rightRect(WIDTH / 2, 0, WIDTH / 2, HEIGHT);

		frame(leftRect).copyTo(leftImg);
		frame(rightRect).copyTo(rightImg);
		namedWindow("left", WINDOW_AUTOSIZE);
		namedWindow("right", WINDOW_AUTOSIZE);
		imshow("left", leftImg);
		imshow("right", rightImg);

		//按空格键保存当前帧
		if (waitKey(33) == 32)
		{
			static int num = 1;
			String left_img_name, right_img_name;
			left_img_name = "left111_" + to_string(num) + ".jpg";
			right_img_name = "right111_" + to_string(num) + ".jpg";
			cout << left_img_name << " " << right_img_name << endl;
			num++;
			imwrite("../images/origin_left/" + left_img_name, leftImg);
			imwrite("../images/origin_right/" + right_img_name, rightImg);
		}
		//在视频播放期间按键退出
		if (waitKey(25) == 27) break;
	}
	//释放
	capture.release();
	return 0;
}


