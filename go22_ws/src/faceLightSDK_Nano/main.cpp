#include "FaceLightClient.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <ros/package.h>
#include <ros/sound_play/SoundRequest.h>

int flag = 0; // 声明并初始化 flag 变量
ros::Subscriber subscriber;
ros::Publisher soundPub;

void callback(const std_msgs::Int32::ConstPtr& msg) {
    std::cout << "step 3:subsubsubsubsubsubsubsubsub" << std::endl;
    flag = msg->data; // 更新 flag 的值
    if (flag == 1) {
        subscriber.shutdown();
    }
}

int main(int argc, char** argv) {
    FaceLightClient client;
    ros::init(argc, argv, "light_node");
    ros::NodeHandle nh;
   
    soundPub = nh.advertise<ros::sound_play::SoundRequest>("dog_sound", 1, true);
    ros::Duration(0.5).sleep(); // 等待初始化完成
    
    subscriber = nh.subscribe("msg_topic", 10, callback);
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    while (ros::ok()) {
        ros::spinOnce();

        if (flag == 1) {
            bool exitFlag = false;
            while (!exitFlag) {
                client.setLedColor(0, client.red);
                client.setLedColor(1, client.red);
                client.setLedColor(2, client.red);
                client.setLedColor(3, client.red);
                client.setLedColor(4, client.red);
                client.setLedColor(5, client.red);
                client.setLedColor(6, client.black);
                client.setLedColor(7, client.black);
                client.setLedColor(8, client.black);
                client.setLedColor(9, client.black);
                client.setLedColor(10, client.black);
                client.setLedColor(11, client.black);
                client.sendCmd();
                ros::Duration(0.1).sleep();
                
                ros::sound_play::SoundRequest soundReq;
                soundReq.command = soundReq.PLAY_FILE;
                soundReq.sound = ros::sound_play::SoundRequest::PLAY_FILE;
                soundReq.arg = ros::package::getPath("faceLightClient") + "/dogdog.wav";
                soundReq.volume = 1.0;//音量控制：0.0～1.0
                soundPub.publish(soundReq);
                
                client.setLedColor(0, client.black);
                client.setLedColor(1, client.black);
                client.setLedColor(2, client.black);
                client.setLedColor(3, client.black);
                client.setLedColor(4, client.black);
                client.setLedColor(5, client.black);
                client.setLedColor(6, client.blue);
                client.setLedColor(7, client.blue);
                client.setLedColor(8, client.blue);
                client.setLedColor(9, client.blue);
                client.setLedColor(10, client.blue);
                client.setLedColor(11, client.blue);
                client.sendCmd();
                ros::Duration(0.1).sleep();
                
                exitFlag = true;
            }
        }
    }

    spinner.stop();
    return 0;
}


/*#include <ros/package.h>
 #include <ros/ros.h>
 #include <ros/package.h>
 #include <ros/sound_play/SoundRequest.h>
 
 int main(int argc, char** argv) {
 ros::init(argc, argv, "audio_player");
 ros::NodeHandle nh;
 ros::Publisher soundPub = nh.advertise<ros::sound_play::SoundRequest>("/robotsound", 1, true);
 
 ros::Duration(0.5).sleep(); // 等待初始化完成
 
 while (ros::ok()) {
 ros::spinOnce();
 
 // 播放音频文件
 ros::sound_play::SoundRequest soundReq;
 soundReq.command = soundReq.PLAY_FILE;
 soundReq.sound = ros::sound_play::SoundRequest::PLAY_FILE;
 soundReq.arg = ros::package::getPath("your_package_name") + "/your_audio_file.wav";
 soundReq.volume = 1.0;//音量控制：0.0～1.0
 
 soundPub.publish(soundReq);
 
 // 在此添加其他逻辑或延时操作
 
 ros::Duration(1.0).sleep(); // 延时1秒
 }
 
 return 0;
 }*/
