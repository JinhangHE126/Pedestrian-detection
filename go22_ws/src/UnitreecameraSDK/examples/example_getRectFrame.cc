#include <UnitreeCameraSDK.hpp>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

// #include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>

using namespace std;
using namespace cv;

bool hasSkinColor(const Vec3b &pixel)
{
  int b = pixel[0];
  int g = pixel[1];
  int r = pixel[2];

  return (r > 95 && g > 40 && b > 20 && r > g && r > b && abs(r - g) > 15);
}
// 这个函数接受一个3通道的彩色像素Vec3b并检查它是否类似于肤色，基于一些条件。
// 如果条件满足，它返回true，表示像素具有肤色。

ros::Publisher publisher;

void publish_topic()
{
  std_msgs::Int32 msg;
  msg.data = 1;           // 设置消息的数据
  publisher.publish(msg); // 发布消息
  std::cout << "publish: publish: publish: publish: publish: publish: publish: " << std::endl;
}

int main(int argc, char *argv[])
{
  cv::VideoCapture cap; // 创建摄像头对象
  cv::Mat frame;        // 创建图像帧对象
  cap.open(1);

  ros::init(argc, argv, "camera_publisher_node");
  ros::NodeHandle nh;
  publisher = nh.advertise<std_msgs::Int32>("msg_topic", 10);

  int deviceNode = 0;            // 默认设备节点为0 -> /dev/video0
  cv::Size frameSize(1856, 800); // 默认帧尺寸为1856x800
  int fps = 30;                  // 默认相机每秒帧率为30

  if (argc >= 4)
  {
    deviceNode = std::atoi(argv[2]);
    frameSize = cv::Size(std::atoi(argv[2]), std::atoi(argv[3]));
    if (argc >= 5)
      fps = std::atoi(argv[4]);
  }

  //UnitreeCamera cam("stereo_camera_config.yaml"); // 使用设备节点号初始化相机
  if (!cap.isOpened()) // 获取相机是否成功打开的状态
    exit(EXIT_FAILURE);

  // cap.setRawFrameSize(frameSize); // 设置相机帧尺寸
  // cap.setRawFrameRate(fps); // 设置相机帧率
  // cap.setRectFrameSize(cv::Size(frameSize.width >> 2, frameSize.height >> 1)); // 设置相机校正帧尺寸
  // cap.startCapture(); // 禁用图像H264编码和共享内存共享

  usleep(500000);
  while (cap.isOpened())
  {
    // cv::Mat left, right;
    // if (!cap.getRectStereoFrame(left, right)) // 获取校正后的左右图像帧
    //{
    // usleep(1000);
    // continue;
    //}
    // 代码从此开始
    Mat frame, mask, maskCp;              // frame:存储捕获的视频帧 mask:用于二值图像遮罩 maskCp:作为遮罩的副本
    vector<vector<Point>> cnts;           // 用于存储轮廓点
    Rect maxRect;                         // 存储围绕轮廓的最大矩形
    const double RECT_HW_RATIO = 1.5;     // 人体长宽比阈值
    const double RECT_AREA_RATIO = 0.008; // 人体占整个图像最小比例阈值
    const double RECT_AREA_RATIO2 = 0.2;  // 人体占整体图像最大比例阈值

    Ptr<BackgroundSubtractorMOG2> bgsubtractor = createBackgroundSubtractorMOG2();
    bgsubtractor->setHistory(20);
    bgsubtractor->setVarThreshold(100);
    bgsubtractor->setDetectShadows(true);
    // 使用背景减法器进行背景减法

    bool hasPeople = false;

    while (true)
    {
      cap >> frame;
      if (frame.empty())
      {
        break;
      }
      // 捕获摄像头的帧进行处理，如若没有捕获到，则退出循环

      bgsubtractor->apply(frame, mask, 0.002);
      medianBlur(mask, mask, 3);
      threshold(mask, mask, 200, 255, cv::THRESH_BINARY);
      // 使用背景减法器apply()对当前帧进行背景减法，结果的遮罩执行中值模糊和阈值化操作。存储在mask中

      maskCp = mask.clone();                                                  // 克隆
      findContours(maskCp, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // 将找到的轮廓存储在cnts中

      vector<Point> maxCnt;
      for (int i = 0; i < cnts.size(); ++i)
      {
        maxCnt = maxCnt.size() > cnts[i].size() ? maxCnt : cnts[i];
      } // 找到的轮廓中寻找最大轮廓。通过遍历所有轮廓并比较它们的大小，确定哪个轮廓具有最多的点数。

      if (maxCnt.size() > 0)
      {
        maxRect = boundingRect(maxCnt); // 计算一个包围这个轮廓的最小矩形边界框
        double rectAreaRatio = (double)maxRect.area() / (frame.cols * frame.rows);
        // 计算轮廓的矩形面积以及面积占整个帧的比例

        if ((double)maxRect.height / maxRect.width > RECT_HW_RATIO &&
            rectAreaRatio > RECT_AREA_RATIO && rectAreaRatio < RECT_AREA_RATIO2)
        {
          // 判断矩形长宽比、矩形面积是否符合阈值

          // 颜色特征过滤
          Mat roi = frame(maxRect);
          bool hasSkin = false;
          for (int y = 0; y < roi.rows; ++y)
          {
            for (int x = 0; x < roi.cols; ++x)
            {
              Vec3b pixel = roi.at<Vec3b>(y, x);
              if (hasSkinColor(pixel))
              {
                hasSkin = true;
                break;
              }
            }
            if (hasSkin)
            {
              break;
            }
          }
          // 判断是否符合皮肤颜色，进行筛选

          if (hasSkin)
          {
            rectangle(frame, maxRect.tl(), maxRect.br(), Scalar(0, 255, 0), 2); // 如果符合则绘制一个绿色的矩形框
            hasPeople = true;
          }
          else
          {
            hasPeople = false;
          }
        }
        else
        {
          hasPeople = false;
        }
      }
      else
      {
        hasPeople = false;
      }

      resize(frame, frame, Size(), 2.0, 2.0);
      resize(mask, mask, Size(), 2.0, 2.0);

      imshow("frame", frame);
      imshow("mask", mask);

      if (hasPeople)
      {
        // cout << "有人" << endl;

        // 获取人的位置信息
        int personX = maxRect.x + maxRect.width / 2;
        int personY = maxRect.y + maxRect.height / 2;
        int frameCenterX = frame.cols / 2;
        int frameCenterY = frame.rows / 2;

        // 计算人与摄像头中心的距离
        double distance = sqrt(pow(personX - frameCenterX, 2) + pow(personY - frameCenterY, 2));

        if (personX < frameCenterX - 50)
        {

          // 人在摄像头左侧，机械狗向左转
          // your_robot_library::moveLeft();  // 调用机械狗控制库中的向左移动函数
          cout << "left" << endl;
          std_msgs::Int32 msg;
          msg.data = 1;
          publisher.publish(msg);
        }
        else if (personX > frameCenterX + 50)
        {
          // 人在摄像头右侧，机械狗向右转
          // your_robot_library::moveRight();  // 调用机械狗控制库中的向右移动函数
          cout << "right" << endl;
          std_msgs::Int32 msg;
          msg.data = 1;
          publisher.publish(msg);
        }
        else
        {
          // 人在摄像头中心附近，机械狗向前移动
          // your_robot_library::moveForward();  // 调用机械狗控制库中的向前移动函数
          cout << "walk1" << endl;
          std_msgs::Int32 msg;
          msg.data = 1;
          publisher.publish(msg);
        }
      }
      else
      {
        // 没有检测到人，机械狗停止移动
        // your_robot_library::stopMoving();  // 调用机械狗控制库中的停止移动函数
        cout << "walk2" << endl;
        std_msgs::Int32 msg;
        msg.data = 0;
        publisher.publish(msg);
      }

      if (waitKey(10) == 27)
      {
        break;
      }
    }

    // image_callback(right);
    std::cout << "step 15" << std::endl;
    char key = cv::waitKey(10);
    if (key == 27)
    {
      break;
    }
    ros::spinOnce();
    std::cout << "step 16" << std::endl;
  }

  // cap.stopCapture(); // 停止相机捕获
  cap.release(); // 释放相机资源
  return 0;
}

