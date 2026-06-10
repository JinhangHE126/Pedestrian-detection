#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <mutex>
#include <condition_variable>
using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
Custom(uint8_t level) : safe(LeggedType::Go1),
udp(level, 8090, "192.168.123.161", 8082)
{
udp.InitCmdData(cmd);
}
void UDPRecv();
void UDPSend();
void RobotControl();

Safety safe;
UDP udp;
HighCmd cmd = {0};
HighState state = {0};
int motiontime = 0;
float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
udp.Recv();
}

void Custom::UDPSend()
{
udp.Send();
}

// std::string  flag = 0;  // 声明并初始化 flag 变量
std::condition_variable flagCond;
int flag = 0;  // 声明并初始化 flag 变量
ros::Subscriber subscriber;
std::mutex flagMutex;  // 声明互斥锁
// std::condition_variable flagCond;  // 声明条件变量

// void callback(const std_msgs::Int32::ConstPtr& msg) {


void callback(const std_msgs::String::ConstPtr& msg) {
    try {
        // 将 msg->data 转换为整数
        int intValue = std::stoi(msg->data);
        
        // 使用整数值更新 flag
        flag = intValue;
        
        std::cout << "Received message: " << flag << std::endl;
    } catch (const std::invalid_argument& e) {
        // 处理 msg->data 不是有效整数的情况
        std::cerr << "Error converting to integer: " << e.what() << std::endl;
    } catch (const std::out_of_range& e) {
        // 处理转换后的整数超出范围的情况
        std::cerr << "Integer conversion out of range: " << e.what() << std::endl;
    }
}

// void callback(const std_msgs::String::ConstPtr& msg){
// //     std::unique_lock<std::mutex> lock(flagMutex);  // 加锁

//     flag = msg->data;// 更新 flag 的值

// //     flagCond.notify_one();  // 通知处理线程有新的 flag 值需要处理
//     std::cout << "Received message: " << flag << std::endl;

// //     lock.unlock();  // 解锁
// }


void Custom::RobotControl()
{
motiontime += 2;
udp.GetRecv(state);
cmd.mode = 0;
cmd.gaitType = 0;
cmd.speedLevel = 0;
cmd.footRaiseHeight = 0;
cmd.bodyHeight = 0;
cmd.euler[0] = 0;
cmd.euler[1] = 0;
cmd.euler[2] = 0;
cmd.velocity[0] = 0.0f;
cmd.velocity[1] = 0.0f;
cmd.yawSpeed = 0.0f;
cmd.reserve = 0;

int rate=1001;
ros::Rate loopRate(rate);
double duration=10;
int iterations=rate*duration;

std::unique_lock<std::mutex> lock(flagMutex); // 加锁

if(flag == 1)
{//前进
for(int i=0; i<iterations;++i){
std::cout << "step 1" << std::endl;
cmd.mode = 2;
cmd.gaitType = 1;
cmd.velocity[0] = 0.2f;
cmd.bodyHeight = 0.1;
}


}else if(flag == 2)
{//后退
for(int i=0; i<iterations;++i){
std::cout << "step 2" << std::endl;
cmd.mode = 2;
cmd.gaitType = 1;
cmd.velocity[0] = -0.2f;
cmd.bodyHeight = 0.1;
}

}else if(flag == 3)
{//逆时针转
for(int i=0; i<iterations;++i){
std::cout << "step 3" << std::endl;
cmd.mode = 2;
cmd.gaitType = 2;
cmd.velocity[0] = 0.2f;
cmd.yawSpeed = 2;
cmd.footRaiseHeight = 0.1;
}

}else if(flag == 4)
{//顺时针转
for(int i=0; i<iterations;++i){
std::cout << "step 4" << std::endl;
cmd.mode = 2;
cmd.gaitType = 2;
cmd.velocity[0] = 0.2f;
cmd.yawSpeed = -2;
cmd.footRaiseHeight = 0.1;
}
}else if(flag == 0){
        //停止
std::cout << "step 00000000000000000000000000000000" << std::endl;
        cmd.mode = 0;
        cmd.velocity[0] = 0;        
}else{

 cmd.mode = 0;
 cmd.velocity[0] = 0; 
}	


lock.unlock(); // 解锁
udp.SetSend(cmd);
}

int main(int argc, char **argv)
{
std::cout << "Communication level is set to HIGH-level." << std::endl
<< "WARNING: Make sure the robot is standing on the ground." << std::endl
<< "Press Enter to continue..." << std::endl;
std::cout << 111<< std::endl;
std::cin.ignore();
Custom custom(HIGHLEVEL);

ros::init(argc, argv, "legged_node");
ros::NodeHandle nh;
// subscriber = nh.subscribe("target_position", 10, callback);
ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter",10,callback);

ros::AsyncSpinner spinner(1);// 创建异步 spinner 指定线程数为 1
spinner.start(); // 启动 spinner
std::cout << 2222<< std::endl;

while (ros::ok())
{
std::cout << 3333<< std::endl;
ros::spinOnce(); //

std::unique_lock<std::mutex> lock(flagMutex); // 加锁

// 使用条件变量等待新的 flag 值的到来
// flagCond.wait(lock, [](){ return flag != 0; });
flagCond.wait(lock, [flag]() { return flag != 0; });
lock.unlock(); // 解锁
if(flag == 1 || flag == 2 ){
std::cout << "step 6:" <<flag<< std::endl;
std::cout << "step 6:" <<flag<< std::endl;
std::cout << "step 6:" <<flag<< std::endl;
LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

loop_udpSend.start();
loop_udpRecv.start();
loop_control.start();

} else{

custom.RobotControl();
custom.UDPSend();
custom.UDPRecv();

}
}
std::cout << 999<< std::endl;
spinner.stop();
return 0;
}

